/*
 * Multi-sensor MAXM86161 PPG application layer (Zephyr / nRF5340).
 *
 * Flow:
 *  init  → MUX init, GPIO setup, configure all sensors via broadcast
 *  start → arm INTB interrupt, start periodic GPIO-sync k_timer
 *  loop  → timer ISR pulses GPIO trigger → sensors sample → INTB fires →
 *           thread reads each enabled MUX channel → sends BLE packets
 *  stop  → stop timer, deselect MUX
 *
 * BLE packet format – one packet covers ALL active sensors and LEDs:
 *   [0x70][cnt_lo][cnt_hi][ts×4][sensor_mask][led_mask][trigger]
 *   Sample 1: [ch0_G,ch0_IR,ch0_R][ch1_G,...] ... (active sensors only)
 *   Sample 2: ...
 *   Sample 3: ...
 *   Sample 4: ...
 *   [0x71]
 *
 *   led_mask bits: 0=green, 1=IR, 2=red.  sensor_mask: bit N = sensor N active.
 *   trigger: GPIO state (0 or 1) of ppg_debug_gpio at packet assembly time.
 *   Each ADC value: 19-bit >> 3 → 16-bit, little-endian.
 *   Only bytes for active sensors/LEDs are emitted; disabled ones are omitted.
 *   Worst case (8 sensors, 3 LEDs, 4 samples): 1+2+4+1+1+1 + 4×8×6 + 1 = 203 B.
 */

#include "sensors/ppg_new/ppg_new_appl.h"

#include "bsp/i2c_mux/i2c_mux.h"
#include "maxm86161.h"
#include "maxm86161_i2c.h"
#include "ble/ble_appl.h"

#include <string.h>
#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(ppg_new, LOG_LEVEL_INF);

/*==============================================================================
 * Configuration
 *============================================================================*/

/** Samples per BLE packet (all sensors combined into one packet). */
#define PPG_SAMPLES_PER_PKT  4U

/** FIFO interrupt level per active LED slot. int_level = PPG_INT_LEVEL × active_leds. */
#define PPG_INT_LEVEL        PPG_SAMPLES_PER_PKT

/** Minimum accepted sample rate from the GUI (Hz). */
#define PPG_SAMPLE_RATE_MIN 8U

/** Proximity detection timeout: if no presence detected within this time,
 *  give up and start PPG streaming anyway. */
#define PPG_PROX_TIMEOUT_MS 10000U

/*
 * Worst-case packet size (8 sensors, 3 LEDs, 4 samples, 16-bit values):
 *   header(1) + counter(2) + timestamp(4) + sensor_mask(1) + led_mask(1) + trigger(1)
 *   + PPG_SAMPLES_PER_PKT × 8 sensors × 3 LEDs × 2 bytes + trailer(1)
 *   = 1+2+4+1+1+1 + 4×8×6 + 1 = 203 bytes (fits in 251-byte BLE ATT MTU)
 */
#define PPG_PKT_SIZE_MAX  (1U + 2U + 4U + 1U + 1U + 1U + \
                           PPG_SAMPLES_PER_PKT * I2C_MUX_MAX_CHANNELS * 3U * 2U + 1U)

#define PPG_DATA_HEADER  0x70U
#define PPG_DATA_TRAILER 0x71U

/** Thread tuning */
#define PPG_THREAD_STACK_SIZE  3072
#define PPG_THREAD_PRIORITY    6

/** INTB semaphore timeout – slightly longer than the longest sample period. */
#define PPG_INTB_TIMEOUT_MS  500

/*==============================================================================
 * Hardware resources
 *============================================================================*/

/* I2C_B bus (i2c1): SCL=P1.10, SDA=P0.24 – where the MUX lives. */
static const struct device *const i2c_b = DEVICE_DT_GET(DT_ALIAS(i2cb));

/*
 * GPIO sync output: P0.31 (active-low pulse to MAXM86161 GPIO input).
 * Exported for external control (e.g., from main.c).
 */
const struct gpio_dt_spec ppg_sync_gpio =
    GPIO_DT_SPEC_GET(DT_NODELABEL(ppg_sync), gpios);

/* INTB wired-AND input: all MAXM86161 INTB open-drain outputs → P0.12. */
static const struct gpio_dt_spec ppg_intb_gpio =
    GPIO_DT_SPEC_GET(DT_NODELABEL(ppg_intb), gpios);

/* Debug toggle output: P0.29 – toggles every PPG_DEBUG_TOGGLE_PKTS packets. */
static const struct gpio_dt_spec ppg_debug_gpio =
    GPIO_DT_SPEC_GET(DT_NODELABEL(ppg_debug), gpios);

/** Toggle ppg_debug_gpio every this many BLE packets (= 100 PPG samples). */
#define PPG_DEBUG_TOGGLE_PKTS  (100U / PPG_SAMPLES_PER_PKT)
//#define PPG_DEBUG_TOGGLE_PKTS  200

/*==============================================================================
 * Private state
 *============================================================================*/

static volatile ppg_new_state_t ppg_state        = PPG_NEW_STATE_IDLE;
static volatile bool            ppg_keep_running = false;
static bool                     ppg_initialized  = false;

/* Runtime configuration – written by ppg_new_start_streaming(), read by thread. */
static ppg_config_t ppg_runtime_cfg;

/* Semaphore given by INTB ISR → wakes streaming thread. */
static K_SEM_DEFINE(ppg_intb_sem, 0, 1);

/* Semaphore given by ppg_new_start_streaming() → starts thread loop. */
static K_SEM_DEFINE(ppg_start_sem, 0, 1);

/* Periodic timer that generates the GPIO sync trigger. */
static struct k_timer ppg_sync_timer;

/* GPIO callback struct for INTB interrupt. */
static struct gpio_callback ppg_intb_cb;

/* Global packet counter (2 bytes, wraps at 65536). */
static uint16_t ppg_pkt_counter;

/* Counts packets since last debug-pin toggle; resets at PPG_DEBUG_TOGGLE_PKTS. */
static uint16_t ppg_debug_toggle_cnt;

/* Software mirror of debug GPIO state. Use when physical read is unreliable. */
static bool ppg_debug_state;

/* TX buffer sized for worst-case packet. */
static uint8_t ppg_tx_buf[PPG_PKT_SIZE_MAX];

/*
 * Per-channel dequeued sample storage.
 * Channels are read sequentially; samples are held here until the combined
 * packet is assembled.
 */
static maxm86161_ppg_sample_t ppg_ch_samples[I2C_MUX_MAX_CHANNELS][PPG_SAMPLES_PER_PKT];
static uint8_t                ppg_ch_n_samples[I2C_MUX_MAX_CHANNELS];

/*
 * Shared FIFO queue – cleared and re-used for each channel read in turn.
 */
static maxm86161_ppg_sample_t ppg_queue_storage[PPG_SAMPLES_PER_PKT + 4];
static maxm86161_fifo_queue_t ppg_queue;

/*==============================================================================
 * Sensor configuration helpers
 *============================================================================*/

/**
 * MAXM86161 internal sample rate register value.
 * In GPIO trigger mode this sets the MINIMUM inter-trigger guard period.
 * We always set it to 1024 sps (≈0.977 ms), well below the shortest
 * trigger period we support (1 000 000 / 8 = 125 000 µs at 8 Hz minimum
 * up to 1 000 000 / 255 ≈ 3921 µs at 255 Hz max).
 */
static uint8_t ppg_smp_rate_reg(void)
{
    return MAXM86161_PPG_CFG_SMP_RATE_P1_1024sps;
}

/** Convert runtime sample_rate_hz to timer period in microseconds. */
static uint32_t ppg_timer_period_us(uint8_t rate_hz)
{
    if (rate_hz < PPG_SAMPLE_RATE_MIN) {
        rate_hz = PPG_SAMPLE_RATE_MIN;
    }
    return 1000000U / rate_hz;
}

/**
 * Build the LED sequence config from runtime LED current values.
 *
 * IMPORTANT: slots 1–3 must NEVER be set to LEDSQ_OFF, even when a LED's
 * drive current (PA) is 0.  The FIFO parser in maxm86161_read_samples_in_fifo()
 * only enqueues a complete sample when it sees tag==3 (slot 3).  If slot 3 is
 * OFF, tag 3 never appears in the FIFO and the queue stays empty.
 *
 * With PA=0 the LED emits no light but the ADC still takes an ambient reading
 * and writes a FIFO entry with the correct tag.  We exclude those channels from
 * the BLE packet based on led_current being 0, so no data is wasted on-air.
 */
static void ppg_build_led_seq(const ppg_config_t *cfg,
                               maxm86161_ledsq_cfg_t *seq)
{
    seq->ledsq1 = MAXM86161_LEDSQ_GREEN;  /* always slot 1 – PA may be 0 */
    seq->ledsq2 = MAXM86161_LEDSQ_IR;     /* always slot 2 – PA may be 0 */
    seq->ledsq3 = MAXM86161_LEDSQ_RED;    /* always slot 3 – PA may be 0 */
    seq->ledsq4 = MAXM86161_LEDSQ_OFF;
    seq->ledsq5 = MAXM86161_LEDSQ_OFF;
    seq->ledsq6 = MAXM86161_LEDSQ_OFF;
    (void)cfg; /* cfg still used for LED currents in ledpa_cfg */
}

/*==============================================================================
 * GPIO / timer callbacks (ISR context)
 *============================================================================*/

/** INTB falling-edge ISR: at least one sensor has data ready. */
static void ppg_intb_isr(const struct device *dev,
                         struct gpio_callback *cb,
                         uint32_t pins)
{
    ARG_UNUSED(dev);
    ARG_UNUSED(cb);
    ARG_UNUSED(pins);
    k_sem_give(&ppg_intb_sem);
}

/**
 * Periodic timer handler – runs in system ISR context.
 * Pulses the GPIO sync pin LOW for 100 µs to trigger all sensors.
 *
 * Datasheet constraint (PPG_SYNC_CONTROL = 0x02):
 *   t_PLGPIO minimum low pulse width = 64 µs.
 *   100 µs gives ~36 µs margin. Do not shorten below 64 µs.
 */
static void ppg_timer_handler(struct k_timer *timer)
{
    ARG_UNUSED(timer);
    /* Active-low pulse: gpio_pin_set_dt value 1 → pin LOW (active). */
    gpio_pin_set_dt(&ppg_sync_gpio, 1);
    k_busy_wait(100U);
    gpio_pin_set_dt(&ppg_sync_gpio, 0);
}

/*==============================================================================
 * Packet helper
 *============================================================================*/

/**
 * Build one BLE packet from ppg_ch_samples[] and submit it.
 *
 * Layout:
 *   [0x70][cnt_lo][cnt_hi][ts×4][sensor_mask][led_mask][trigger]
 *   for s in 0..3:
 *     for each active sensor (by sensor_mask, low-bit first):
 *       [G_lo,G_hi]  (if led_mask bit0)
 *       [IR_lo,IR_hi] (if led_mask bit1)
 *       [R_lo,R_hi]  (if led_mask bit2)
 *   [0x71]
 *
 *   trigger: GPIO state (0=low, 1=high) of ppg_debug_gpio at packet assembly time.
 *   Values: 19-bit ADC >> 3 → uint16_t, little-endian.
 *   Inactive sensors/LEDs produce no bytes.
 */
static void ppg_send_combined_packet(void)
{
    const ppg_config_t *cfg = &ppg_runtime_cfg;

    uint8_t led_mask = ((cfg->led_green != 0) ? 0x01U : 0U)
                     | ((cfg->led_ir    != 0) ? 0x02U : 0U)
                     | ((cfg->led_red   != 0) ? 0x04U : 0U);

    uint32_t ts = k_cyc_to_us_floor32(k_cycle_get_32());

    uint16_t idx = 0;
    ppg_tx_buf[idx++] = PPG_DATA_HEADER;
    ppg_tx_buf[idx++] = (uint8_t)(ppg_pkt_counter & 0xFF);
    ppg_tx_buf[idx++] = (uint8_t)(ppg_pkt_counter >> 8);
    ppg_pkt_counter++;
    ppg_tx_buf[idx++] = (uint8_t)(ts & 0xFF);
    ppg_tx_buf[idx++] = (uint8_t)((ts >>  8) & 0xFF);
    ppg_tx_buf[idx++] = (uint8_t)((ts >> 16) & 0xFF);
    ppg_tx_buf[idx++] = (uint8_t)((ts >> 24) & 0xFF);
    ppg_tx_buf[idx++] = cfg->sensor_mask;
    ppg_tx_buf[idx++] = led_mask;
    /* Use software mirror of debug GPIO state for the trigger byte. */
    uint8_t trigger = (uint8_t)(ppg_debug_state ? 1U : 0U);
    /* Diagnostic: log the software mirror value every packet to help tracing. */
    LOG_INF("PPG debug (sw) read=%u pkt=%u", trigger, ppg_pkt_counter);
    ppg_tx_buf[idx++] = trigger;

    for (uint8_t s = 0; s < PPG_SAMPLES_PER_PKT; s++) {
        for (uint8_t ch = 0; ch < I2C_MUX_MAX_CHANNELS; ch++) {
            if (!(cfg->sensor_mask & (1U << ch))) {
                continue;
            }

            /* Use zero if this channel delivered fewer samples than expected. */
            uint32_t p1 = 0U, p2 = 0U, p3 = 0U;
            if (s < ppg_ch_n_samples[ch]) {
                p1 = ppg_ch_samples[ch][s].ppg1;
                p2 = ppg_ch_samples[ch][s].ppg2;
                p3 = ppg_ch_samples[ch][s].ppg3;
            }

            if (cfg->led_green != 0) {
                uint16_t v = (uint16_t)(p1 >> 3);
                ppg_tx_buf[idx++] = (uint8_t)(v & 0xFF);
                ppg_tx_buf[idx++] = (uint8_t)(v >> 8);
            }
            if (cfg->led_ir != 0) {
                uint16_t v = (uint16_t)(p2 >> 3);
                ppg_tx_buf[idx++] = (uint8_t)(v & 0xFF);
                ppg_tx_buf[idx++] = (uint8_t)(v >> 8);
            }
            if (cfg->led_red != 0) {
                uint16_t v = (uint16_t)(p3 >> 3);
                ppg_tx_buf[idx++] = (uint8_t)(v & 0xFF);
                ppg_tx_buf[idx++] = (uint8_t)(v >> 8);
            }
        }
    }

    ppg_tx_buf[idx++] = PPG_DATA_TRAILER;
    add_data_to_send_buffer(ppg_tx_buf, idx);
}

/*==============================================================================
 * Sensor initialisation via broadcast
 *============================================================================*/

/**
 * Configure all selected sensors via MUX broadcast using runtime config.
 * After this function returns the sensors are in GPIO-trigger PPG mode,
 * ready to accept sync pulses.
 */
static int ppg_configure_sensors(const ppg_config_t *cfg)
{
    /* 1. Broadcast to all desired channels simultaneously. */
    int ret = i2c_mux_broadcast(cfg->sensor_mask);
    if (ret) {
        LOG_ERR("MUX broadcast failed: %d", ret);
        return ret;
    }

    maxm86161_i2c_set_device(i2c_b, MAXM86161_I2C_ADDR);

    /* 2. Build LED sequence – disable slots for LEDs with current == 0. */
    maxm86161_ledsq_cfg_t ledsq;
    ppg_build_led_seq(cfg, &ledsq);

    /*
     * All 3 LED sequencer slots are always active (slots 1/2/3 = G/IR/R).
     * LEDs with PA=0 take ambient readings but still generate FIFO entries,
     * which is required for the FIFO parser to assemble complete samples.
     * int_level = PPG_INT_LEVEL * 3 → INTB after PPG_INT_LEVEL sample sets.
     */
    uint8_t active_leds = 3U;

    /* Require at least one LED to actually emit light. */
    if (cfg->led_green == 0 && cfg->led_ir == 0 && cfg->led_red == 0) {
        LOG_ERR("All LED currents are zero – no signal will be emitted");
        i2c_mux_deselect_all();
        return -EINVAL;
    }

    /* 3. Build full device config from runtime parameters. */
    maxm86161_device_config_t dev_cfg = {
        .int_level = PPG_INT_LEVEL * active_leds,

        .ledsq_cfg = ledsq,

        .ledpa_cfg = {
            .green = cfg->led_green,
            .ir    = cfg->led_ir,
            .red   = cfg->led_red,
        },

        .ppg_cfg = {
            .alc       = cfg->alc_enable ? MAXM86161_PPG_CFG_ALC_EN
                                         : MAXM86161_PPG_CFG_ALC_DS,
            .offset    = MAXM86161_PPG_CFG_OFFSET_NO,
            .ppg_tint  = (uint8_t)cfg->tint,       /* 0–3 index → enum value */
            .adc_range = (uint8_t)cfg->adc_range,  /* 0–3 index → enum value */
            .smp_rate  = ppg_smp_rate_reg(),
            .smp_freq  = (uint8_t)cfg->sample_avg, /* 0–7 index → enum value */
        },

        .int_cfg = {
            .full_fifo     = MAXM86161_INT_ENABLE,
            .data_rdy      = MAXM86161_INT_DISABLE,
            .alc_ovf       = MAXM86161_INT_DISABLE,
            .proxy         = MAXM86161_INT_DISABLE,
            .led_compliant = MAXM86161_INT_DISABLE,
            .die_temp      = MAXM86161_INT_DISABLE,
            .pwr_rdy       = MAXM86161_INT_DISABLE,
            .sha           = MAXM86161_INT_DISABLE,
        },
    };

    /* 4. Apply configuration (software reset + register writes). */
    sl_status_t sl_ret = maxm86161_init_device(dev_cfg);
    if (sl_ret != SL_STATUS_OK) {
        LOG_ERR("maxm86161_init_device failed: 0x%04X", sl_ret);
        i2c_mux_deselect_all();
        return -EIO;
    }

    /* 4b. Apply LED current range (separate register, not in device_config_t).
     *     The same range index is used for all three LEDs.
     *     0=31mA  1=62mA  2=93mA  3=124mA full-scale. */
    maxm86161_led_range_curr_t led_range = {
        .green = cfg->led_range,
        .ir    = cfg->led_range,
        .red   = cfg->led_range,
    };
    maxm86161_led_range_config(&led_range);

    /*
     * 5. Proximity detection (optional pre-stream step).
     *
     * maxm86161_init_device() sets PROX_INT_THRESHOLD = 0x01 if the PROXIMITY
     * define is set in maxm86161_hrm_config.h, which puts the sensor into
     * proximity mode (uses pilot LED only, no FIFO data until an object is
     * detected).  We handle this explicitly here:
     *
     *  proximity_enable = 0 → clear threshold immediately so normal PPG starts.
     *  proximity_enable = 1 → leave threshold set; the streaming thread will
     *                         poll the PROXY interrupt flag, wait for detection,
     *                         then clear the threshold to switch to PPG.
     */
    if (!cfg->proximity_enable) {
        maxm86161_i2c_write_to_register(MAXM86161_REG_PROX_INT_THRESHOLD, 0x00U);
    }
    /* If proximity_enable = 1, threshold stays at 0x01 from init_device().
     * The streaming thread handles the transition (see below). */

    /* 6. GPIO trigger mode: PPG_SYNC_CONTROL[2:0] = 0b010.
     *    Falling edge on the GPIO pin triggers one sample cycle. */
    maxm86161_i2c_write_to_register(MAXM86161_REG_PPG_SYNC_CONTROL, 0x02U);

    /* 7. Clear stale interrupt flags to prevent spurious INTB assertion. */
    maxm86161_int_t dummy_status;
    maxm86161_get_irq_status(&dummy_status);

    i2c_mux_deselect_all();

    LOG_INF("PPG configured: mask=0x%02X rate=%uHz G=0x%02X IR=0x%02X R=0x%02X "
            "range=%u tint=%u adc=%u avg=%u alc=%u prox=%u",
            cfg->sensor_mask, cfg->sample_rate_hz,
            cfg->led_green, cfg->led_ir, cfg->led_red,
            cfg->led_range, cfg->tint, cfg->adc_range,
            cfg->sample_avg, cfg->alc_enable, cfg->proximity_enable);
    return 0;
}

/*==============================================================================
 * Streaming thread
 *============================================================================*/

static void ppg_streaming_thread(void *arg1, void *arg2, void *arg3)
{
    ARG_UNUSED(arg1);
    ARG_UNUSED(arg2);
    ARG_UNUSED(arg3);

    LOG_INF("PPG streaming thread started");

    while (1) {
        /* Wait for start signal from ppg_new_start_streaming(). */
        k_sem_take(&ppg_start_sem, K_FOREVER);

        const ppg_config_t *rcfg = &ppg_runtime_cfg;

        LOG_INF("PPG starting (mask=0x%02X, %u Hz)",
                rcfg->sensor_mask, rcfg->sample_rate_hz);
        ppg_state = PPG_NEW_STATE_STARTING;

        /* (Re-)configure sensors using runtime config. */
        int ret = ppg_configure_sensors(rcfg);
        if (ret) {
            ppg_state = PPG_NEW_STATE_ERROR;
            continue;
        }

        maxm86161_allocate_ppg_data_queue(
            &ppg_queue,
            ppg_queue_storage,
            (int16_t)(sizeof(ppg_queue_storage)));

        /* Reset packet counter, debug toggle counter, and per-channel sample counts. */
        ppg_pkt_counter = 0;
        ppg_debug_toggle_cnt = 0;
        memset(ppg_ch_n_samples, 0, sizeof(ppg_ch_n_samples));

        /*
         * Proximity pre-detection (optional).
         * When enabled, the sensor is left in proximity mode after init.
         * We poll the PROXY interrupt flag by waiting for INTB; when fired
         * we check if it was a proximity event. Once detected (or timed out)
         * we clear PROX_INT_THRESHOLD to switch to normal PPG operation.
         */
        if (rcfg->proximity_enable) {
            LOG_INF("PPG proximity detection: waiting for object...");

            /* Arm INTB to catch the proximity interrupt. */
            k_sem_reset(&ppg_intb_sem);
            gpio_pin_interrupt_configure_dt(&ppg_intb_gpio,
                                            GPIO_INT_EDGE_TO_ACTIVE);

            bool detected = false;
            int64_t deadline = k_uptime_get() + PPG_PROX_TIMEOUT_MS;

            while (k_uptime_get() < deadline && ppg_keep_running) {
                ret = k_sem_take(&ppg_intb_sem, K_MSEC(100));
                if (ret == 0) {
                    /* Read IRQ status to find out what fired. */
                    i2c_mux_broadcast(rcfg->sensor_mask);
                    maxm86161_i2c_set_device(i2c_b, MAXM86161_I2C_ADDR);
                    maxm86161_int_t irq;
                    maxm86161_get_irq_status(&irq);
                    i2c_mux_deselect_all();

                    if (irq.proxy) {
                        detected = true;
                        LOG_INF("PPG proximity detected – switching to PPG");
                        break;
                    }
                }
            }

            if (!detected) {
                LOG_WRN("PPG proximity timeout – starting PPG anyway");
            }

            /* Disable proximity mode → sensor switches to normal PPG. */
            i2c_mux_broadcast(rcfg->sensor_mask);
            maxm86161_i2c_set_device(i2c_b, MAXM86161_I2C_ADDR);
            maxm86161_i2c_write_to_register(MAXM86161_REG_PROX_INT_THRESHOLD, 0x00U);
            /* Re-enable GPIO trigger mode (init_device set it; ensure it's still set). */
            maxm86161_i2c_write_to_register(MAXM86161_REG_PPG_SYNC_CONTROL, 0x02U);
            maxm86161_int_t dummy;
            maxm86161_get_irq_status(&dummy);
            i2c_mux_deselect_all();

            /* Disarm INTB until the timer loop re-arms it below. */
            gpio_pin_interrupt_configure_dt(&ppg_intb_gpio, GPIO_INT_DISABLE);
        }

        if (!ppg_keep_running) {
            ppg_state = PPG_NEW_STATE_IDLE;
            continue;
        }

        /* Arm INTB interrupt and start the GPIO sync timer. */
        k_sem_reset(&ppg_intb_sem);
        gpio_pin_interrupt_configure_dt(&ppg_intb_gpio,
                                        GPIO_INT_EDGE_TO_ACTIVE);

        uint32_t period_us = ppg_timer_period_us(rcfg->sample_rate_hz);
        k_timer_start(&ppg_sync_timer,
                      K_USEC(period_us),
                      K_USEC(period_us));

        ppg_state = PPG_NEW_STATE_STREAMING;

        /* Re-configure as output every session: any SYS_INIT-level driver
         * (e.g. gpio-keys) may have reconfigured this pin as input. */
        ret = gpio_pin_configure_dt(&ppg_debug_gpio, GPIO_OUTPUT_INACTIVE);
        if (ret) {
            LOG_ERR("PPG debug GPIO re-configure failed: %d", ret);
        }
        /* Ensure software mirror starts cleared. */
        ppg_debug_state = false;
        ppg_debug_toggle_cnt = 0;

        LOG_INF("PPG streaming active (%u Hz, mask=0x%02X)",
                rcfg->sample_rate_hz, rcfg->sensor_mask);

        /* ---- Main streaming loop ---- */
        while (ppg_keep_running) {
            /* Wait for INTB (falling edge → any sensor FIFO full). */
            ret = k_sem_take(&ppg_intb_sem,
                             K_MSEC(PPG_INTB_TIMEOUT_MS));
            if (ret == -EAGAIN) {
                /* Timeout – check keep_running and retry. */
                continue;
            }

            /* Read PPG_SAMPLES_PER_PKT samples from every active channel. */
            for (uint8_t ch = 0; ch < I2C_MUX_MAX_CHANNELS; ch++) {
                if (!(rcfg->sensor_mask & (1U << ch))) {
                    ppg_ch_n_samples[ch] = 0;
                    continue;
                }

                i2c_mux_select_channel(ch);
                maxm86161_i2c_set_device(i2c_b, MAXM86161_I2C_ADDR);

                maxm86161_clear_queue(&ppg_queue);
                maxm86161_read_samples_in_fifo(&ppg_queue);

                uint16_t n = maxm86161_num_samples_in_queue(&ppg_queue);
                if (n > PPG_SAMPLES_PER_PKT) {
                    n = PPG_SAMPLES_PER_PKT;
                }
                ppg_ch_n_samples[ch] = (uint8_t)n;
                for (uint8_t s = 0; s < n; s++) {
                    maxm86161_dequeue_ppg_sample_data(&ppg_queue,
                                                      &ppg_ch_samples[ch][s]);
                }
            }

            i2c_mux_deselect_all();



            /* Assemble and send one combined packet for all sensors. */
            ppg_send_combined_packet();

            if (++ppg_debug_toggle_cnt >= PPG_DEBUG_TOGGLE_PKTS) {
                ppg_debug_toggle_cnt = 0;
                int tret = gpio_pin_toggle_dt(&ppg_debug_gpio);
                if (tret == 0) {
                    /* Mirror the toggled state in software (flip boolean). */
                    ppg_debug_state = !ppg_debug_state;
                }
                LOG_INF("PPG debug toggle, ret=%d sw=%d", tret, (int)ppg_debug_state);
            }
        }

        /* ---- Stop ---- */
        ppg_state = PPG_NEW_STATE_STOPPING;

        k_timer_stop(&ppg_sync_timer);
        gpio_pin_interrupt_configure_dt(&ppg_intb_gpio, GPIO_INT_DISABLE);
        gpio_pin_set_dt(&ppg_debug_gpio, 0);
        ppg_debug_state = false;

        /* Shutdown all sensors (broadcast). */
        i2c_mux_broadcast(rcfg->sensor_mask);
        maxm86161_i2c_set_device(i2c_b, MAXM86161_I2C_ADDR);
        maxm86161_shutdown_device(true);
        i2c_mux_deselect_all();

        ppg_state = PPG_NEW_STATE_IDLE;
        LOG_INF("PPG streaming stopped");
    }
}

K_THREAD_DEFINE(ppg_new_thread_id,
                PPG_THREAD_STACK_SIZE,
                ppg_streaming_thread,
                NULL, NULL, NULL,
                PPG_THREAD_PRIORITY, 0, 0);

/*==============================================================================
 * Public API
 *============================================================================*/

int ppg_new_init(void)
{
    if (ppg_initialized) {
        return 0;
    }

    /* Verify I2C bus is ready. */
    if (!device_is_ready(i2c_b)) {
        LOG_ERR("I2C_B not ready");
        return -ENODEV;
    }

    /* Initialise TCA9548A MUX. */
    int ret = i2c_mux_init(i2c_b, I2C_MUX_ADDR_DEFAULT);
    if (ret) {
        LOG_ERR("MUX init failed: %d", ret);
        return ret;
    }


    // Sanity test PPG sensor there?
    uint8_t part = 0x00;
    uint8_t rev  = 0x00;
    i2c_mux_select_channel(7);
    maxm86161_i2c_set_device(i2c_b, MAXM86161_I2C_ADDR);
    maxm86161_i2c_read_from_register(MAXM86161_REG_PART_ID, &part);
    maxm86161_i2c_read_from_register(MAXM86161_REG_REV_ID,  &rev);
    LOG_INF("PPG ch%u: PART_ID=0x%02X REV_ID=0x%02X", 0, part, rev);
    i2c_mux_deselect_all();

    /* Configure GPIO sync output pin (P0.13, active-low). */
    if (!gpio_is_ready_dt(&ppg_sync_gpio)) {
        LOG_ERR("PPG sync GPIO not ready");
        return -ENODEV;
    }
    ret = gpio_pin_configure_dt(&ppg_sync_gpio, GPIO_OUTPUT_INACTIVE);
    if (ret) {
        LOG_ERR("PPG sync GPIO config failed: %d", ret);
        return ret;
    }

    /* Configure INTB input (P0.31, active-low, pull-up). */
    if (!gpio_is_ready_dt(&ppg_intb_gpio)) {
        LOG_ERR("PPG INTB GPIO not ready");
        return -ENODEV;
    }
    ret = gpio_pin_configure_dt(&ppg_intb_gpio, GPIO_INPUT | GPIO_PULL_UP);
    if (ret) {
        LOG_ERR("PPG INTB GPIO config failed: %d", ret);
        return ret;
    }

    /* Register INTB falling-edge interrupt (armed later, on start). */
    gpio_init_callback(&ppg_intb_cb, ppg_intb_isr,
                       BIT(ppg_intb_gpio.pin));
    ret = gpio_add_callback(ppg_intb_gpio.port, &ppg_intb_cb);
    if (ret) {
        LOG_ERR("INTB callback add failed: %d", ret);
        return ret;
    }

    /* Configure debug toggle output (P0.29, active-high). */
    if (!gpio_is_ready_dt(&ppg_debug_gpio)) {
        LOG_ERR("PPG debug GPIO not ready");
        return -ENODEV;
    }
    ret = gpio_pin_configure_dt(&ppg_debug_gpio, GPIO_OUTPUT_INACTIVE);
    if (ret) {
        LOG_ERR("PPG debug GPIO config failed: %d", ret);
        return ret;
    }
    ppg_debug_state = false;

    /* Initialise the periodic sync timer (handler set, not started yet). */
    k_timer_init(&ppg_sync_timer, ppg_timer_handler, NULL);

    ppg_initialized = true;
    LOG_INF("PPG subsystem initialised (config applied at start)");
    return 0;
}

int ppg_new_start_streaming(const ppg_config_t *cfg)
{
    if (cfg == NULL) {
        LOG_ERR("ppg_new_start_streaming: cfg is NULL");
        return -EINVAL;
    }
    if (ppg_state == PPG_NEW_STATE_STREAMING) {
        LOG_WRN("PPG already streaming");
        return -EALREADY;
    }
    if (ppg_state != PPG_NEW_STATE_IDLE) {
        LOG_ERR("PPG not idle (state=%d)", ppg_state);
        return -EBUSY;
    }

    /* Validate sample rate. */
    if (cfg->sample_rate_hz < PPG_SAMPLE_RATE_MIN) {
        LOG_ERR("PPG sample rate %u Hz too low (min %u Hz)",
                cfg->sample_rate_hz, PPG_SAMPLE_RATE_MIN);
        return -EINVAL;
    }

    /* Store config for the streaming thread to pick up. */
    ppg_runtime_cfg = *cfg;

    ppg_keep_running = true;
    k_sem_give(&ppg_start_sem);
    return 0;
}

int ppg_new_stop_streaming(void)
{
    if (ppg_state != PPG_NEW_STATE_STREAMING) {
        LOG_WRN("PPG not streaming");
        return -EINVAL;
    }

    ppg_keep_running = false;

    /* Unblock the thread if it is waiting on INTB. */
    k_sem_give(&ppg_intb_sem);

    int timeout = 100; /* up to 1 s */
    while (ppg_state != PPG_NEW_STATE_IDLE && timeout-- > 0) {
        k_msleep(10);
    }
    if (timeout <= 0) {
        LOG_ERR("Timeout waiting for PPG to stop");
        return -ETIMEDOUT;
    }
    return 0;
}

ppg_new_state_t ppg_new_get_state(void) { return ppg_state; }
bool ppg_new_is_streaming(void) { return ppg_state == PPG_NEW_STATE_STREAMING; }
