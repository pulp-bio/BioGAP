/*
 * ----------------------------------------------------------------------
 *
 * File: mmWave_appl.c
 *
 * Copyright (C) 2026, ETH Zurich
 *
 * Authors:
 * - Benjamin Löliger, ETH Zurich
 *
 * ----------------------------------------------------------------------
 * SPDX-License-Identifier: Apache-2.0
 *
 * Licensed under the Apache License, Version 2.0 (the License); you may
 * not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an AS IS BASIS, WITHOUT
 * WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

/**
 * @file mmWave_appl.c
 * @brief mmWave radar application layer implementation
 */

/* Must come before the first mmWave_config.h inclusion: it is what makes the
 * generated register header emit register_list[] into this translation unit. */
#define XENSIV_BGT60TRXX_CONF_IMPL
#include "sensors/mmWave/mmWave_config.h"

#include <stdbool.h>
#include <stdint.h>
#include <string.h>

#include <hal/nrf_saadc.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/__assert.h>
#include <zephyr/sys/atomic.h>
#include <zephyr/toolchain.h>

#include "ble/ble_appl.h"
#include "bsp/pwr_bsp.h"
#include "sensors/mmWave/driver/xensiv_bgt60trxx.h"
/* Declares the platform callbacks implemented below, so the compiler checks
 * them against the signatures the driver expects. */
#include "sensors/mmWave/driver/xensiv_bgt60trxx_platform.h"
#include "sensors/mmWave/mmWave_appl.h"
#include "sensors/mmWave/mmWave_spi.h"

LOG_MODULE_REGISTER(mmWave_appl, LOG_LEVEL_INF);

/*==============================================================================
 * Private Definitions
 *============================================================================*/

/** @brief Stack size of the mmWave streaming thread in bytes */
#define mmWave_THREAD_STACK_SIZE 4096

/** @brief Thread priority of the mmWave streaming thread */
#define mmWave_PRIORITY 6

/** @brief Timeout waiting for a data-ready interrupt while streaming */
#define MMWAVE_IRQ_TIMEOUT_MS 1000

/** @brief Attempts to identify the radar before giving up, in case its
 *  power-on reset has not finished when the first read goes out */
#define MMWAVE_PROBE_ATTEMPTS 3

/** @brief Delay between identification attempts */
#define MMWAVE_PROBE_RETRY_MS 5

/** @brief Consecutive lost frames tolerated before the capture is rebuilt.
 *  A single loss happens when a FIFO read slips past the next frame; a run of
 *  them means a latched error bit that only a FIFO reset clears. */
#define MMWAVE_FRAME_ERROR_LIMIT 5U

/** @brief Capture restarts attempted before the session is abandoned */
#define MMWAVE_RECOVERY_LIMIT 3U

/** @brief Enables the BGT60TR13C internal test-pattern validation */
#define MMWAVE_TESTMODE IS_ENABLED(CONFIG_MMWAVE_TEST_PATTERN)

/* Helpers for splitting the packed register values of the generated header
 * into the address and data fields the driver expects. */
#define XENSIV_BGT60TRXX_SPI_REGADR_MSK (0xFE000000UL)
#define XENSIV_BGT60TRXX_SPI_REGADR_POS (25U)
#define XENSIV_BGT60TRXX_SPI_DATA_MSK (0x00FFFFFFUL)
#define XENSIV_BGT60TRXX_SPI_DATA_POS (0U)

/** @brief Mask of the TX power field inside register 0x11 */
#define TX_POWER_MASK 0x1F

/*==============================================================================
 * BLE Packet Layout
 *============================================================================*/

/*
 * One radar frame is larger than a BLE notification, so it is split into
 * chunks that the host reassembles:
 *
 *   [0]        header (MMWAVE_DATA_HEADER)
 *   [1:5]      frame timestamp, microseconds, big endian.
 *              Bit 0 carries the external sync state (see CONFIG_MMWAVE_
 *              FINAPRES_SYNC) and is masked out of the timestamp itself, so
 *              all chunks of one frame share the same value and the host can
 *              detect chunk loss across frame boundaries.
 *   [5]        chunk index, 0-based
 *   [6]        total number of chunks in this frame
 *   [7:243]    payload (zero padded in the last chunk)
 *   [243]      trailer (MMWAVE_DATA_TRAILER)
 */

/** @brief Bytes of framing overhead per packet (7 leading + 1 trailing) */
#define MMWAVE_PACKET_OVERHEAD 8U

/** @brief Total size of one mmWave BLE packet */
#define MMWAVE_PACKET_SIZE BLE_PCKT_MAX_SIZE

/** @brief Payload bytes carried per packet */
#define MMWAVE_PAYLOAD_SIZE (MMWAVE_PACKET_SIZE - MMWAVE_PACKET_OVERHEAD)

/** @brief Offset of the payload within a packet */
#define MMWAVE_PAYLOAD_OFFSET 7U

/** @brief Offset of the trailer within a packet */
#define MMWAVE_TRAILER_OFFSET (MMWAVE_PACKET_SIZE - 1U)

/* The payload is an exact fit between the 7 leading framing bytes and the
 * trailer. Catch a future edit of MMWAVE_PACKET_OVERHEAD that would let
 * mmwave_send_payload() run past the payload region or over the trailer. */
BUILD_ASSERT(MMWAVE_PAYLOAD_OFFSET + MMWAVE_PAYLOAD_SIZE == MMWAVE_TRAILER_OFFSET,
             "mmWave packet layout is inconsistent: payload would overlap the trailer");

/*==============================================================================
 * Register Tables
 *============================================================================*/

/** @brief Register 0x06 values for the supported radar frame rates */
static const uint32_t fps_reg0x06[] = {
    0x0d10207fUL, /* 0: 25 fps */
    0x0d1020ffUL, /* 1: 50 fps */
    0x0d1021ffUL, /* 2: 100 fps */
    0x0d10237fUL, /* 3: 150 fps */
    0x0d1024ffUL  /* 4: 200 fps */
};

/** @brief Register 0x2d values for the supported radar frame rates */
static const uint32_t fps_reg0x2d[] = {
    0x5b5de40aUL, /* 0: 25 fps */
    0x5b616c0aUL, /* 1: 50 fps */
    0x5b4d2c0aUL, /* 2: 100 fps */
    0x5b46440aUL, /* 3: 150 fps */
    0x5b4a1c0aUL  /* 4: 200 fps */
};

/** @brief Register 0x12 values for the supported IF gain settings */
static const uint32_t if_gain_regs[] = {
    0x25700c63UL, /* 0: 18 dB */
    0x25701ce7UL, /* 1: 23 dB */
    0x25702d6bUL, /* 2: 28 dB */
    0x25000c63UL, /* 3: 30 dB */

    0x25703defUL, /* 4: 33 dB */
    0x25001ce7UL, /* 5: 35 dB */
    0x25704e73UL, /* 6: 38 dB */
    0x25002d6bUL, /* 7: 40 dB */

    0x25705ef7UL, /* 8: 43 dB */
    0x25003defUL, /* 9: 45 dB */
    0x25706f7bUL, /* 10: 48 dB */
    0x25004e73UL, /* 11: 50 dB */

    0x25005ef7UL, /* 12: 55 dB */
    0x25006f7bUL  /* 13: 60 dB */
};

/*==============================================================================
 * Private Variables
 *============================================================================*/

/** @brief Current state of the mmWave application state machine */
static volatile mmWave_state_t mmWave_state = mmWave_STATE_NO_HW;

/** @brief Currently selected IF gain register value */
static uint32_t current_selected_gain_reg = 0x25703defUL;

/** @brief Currently selected TX power register value */
static uint32_t current_tx_power_reg = 0x231ff41fUL;

/** @brief Currently selected frame-rate register value for register 0x06 */
static uint32_t current_fps_reg06 = 0x0d1021ffUL;

/** @brief Currently selected frame-rate register value for register 0x2d */
static uint32_t current_fps_reg2d = 0x5b4d2c0aUL;

/** @brief Controls whether the streaming loop should continue running */
static volatile bool mmWave_keep_running = false;

/** @brief True while the radar drives the power pin shared with the battery ADC.
 *
 * Set before the pin is claimed and cleared after it is released, rather than
 * derived from mmWave_state, so the battery-measurement gate closes for the
 * whole window in which the pin is a GPIO -- including during power_on() itself,
 * before the state advances. */
static volatile bool mmWave_owns_pwr_pin = false;

/** @brief Semaphore used to start the mmWave streaming thread */
static K_SEM_DEFINE(mmWave_start_sem, 0, 1);

/** @brief Semaphore signalled by the radar data-ready interrupt */
static K_SEM_DEFINE(data_ready_mmWave_sem, 0, 1);

/** @brief GPIO callback data for the BGT60TR13C data-ready interrupt */
static struct gpio_callback irq_cb_data;

/** @brief BLE transmission buffer for one mmWave packet */
static uint8_t mmWave_tx_buf[MMWAVE_PACKET_SIZE];

/** @brief Unpacked radar samples of one complete frame */
static uint16_t samples[MMWAVE_NUM_SAMPLES_PER_FRAME];

/** @brief BGT60TR13C driver instance */
static xensiv_bgt60trxx_t dev;

/* Control GPIOs. Chip select is owned by mmWave_spi.c, since it gates access
 * to the shared bus. */
static const struct gpio_dt_spec irq_pin =
    GPIO_DT_SPEC_GET(DT_NODELABEL(bgt60), irq_gpios);
static const struct gpio_dt_spec rst_pin =
    GPIO_DT_SPEC_GET(DT_NODELABEL(bgt60), reset_gpios);
static const struct gpio_dt_spec pwr_pin =
    GPIO_DT_SPEC_GET(DT_NODELABEL(bgt60), power_gpios);

/* The radar's RST and IRQ are P0.04 and P0.12, which are also the PDM
 * microphone's CLK and DATA. If &pdm0 is enabled, pinctrl hands those pins to
 * the PDM peripheral at boot and every gpio_pin_set_dt() on RST below is
 * silently ignored -- the radar then sits in reset and answers every register
 * read with zeros, which is indistinguishable from a wiring fault. Fail the
 * build instead of shipping that. */
BUILD_ASSERT(!DT_NODE_HAS_STATUS(DT_NODELABEL(pdm0), okay),
             "PDM microphone must be disabled when CONFIG_SENSOR_MMWAVE=y: it "
             "owns P0.04/P0.12, the radar's RST and IRQ. Configure with "
             "-DMMWAVE_SHIELD=ON so overlays/mmwave.overlay is applied.");

/*==============================================================================
 * External Sync Output (optional)
 *============================================================================*/

/*
 * Square wave on a spare GPIO used to time-align BioGAP recordings with an
 * external reference device (Finapres / NovaScope) during validation
 * measurements. The current level is mirrored into bit 0 of every packet
 * timestamp, so the host can recover the alignment from the data stream alone.
 *
 * When the feature is disabled the reported sync bit is always 0, which keeps
 * the packet layout identical for both build variants.
 */

/** @brief Current level of the external sync output */
static atomic_t ext_sync_state = ATOMIC_INIT(0);

#if defined(CONFIG_MMWAVE_EXT_SYNC)

#define EXT_SYNC_NODE DT_ALIAS(mmwave_ext_sync)

#if !DT_NODE_HAS_STATUS(EXT_SYNC_NODE, okay)
#error "CONFIG_MMWAVE_EXT_SYNC=y requires an mmwave-ext-sync devicetree alias"
#endif

static const struct gpio_dt_spec ext_sync = GPIO_DT_SPEC_GET(EXT_SYNC_NODE, gpios);

static struct k_timer ext_sync_timer;
static struct k_work ext_sync_work;

static void ext_sync_work_handler(struct k_work *work) {
  ARG_UNUSED(work);
  gpio_pin_set_dt(&ext_sync, (int)atomic_get(&ext_sync_state));
}

static void ext_sync_timer_handler(struct k_timer *timer) {
  ARG_UNUSED(timer);
  atomic_set(&ext_sync_state, !atomic_get(&ext_sync_state));
  k_work_submit(&ext_sync_work);
}

static int ext_sync_init(void) {
  if (!gpio_is_ready_dt(&ext_sync)) {
    LOG_ERR("External sync GPIO not ready");
    return -ENODEV;
  }

  int ret = gpio_pin_configure_dt(&ext_sync, GPIO_OUTPUT_INACTIVE);
  if (ret != 0) {
    LOG_ERR("Failed to configure external sync GPIO: %d", ret);
    return ret;
  }

  k_work_init(&ext_sync_work, ext_sync_work_handler);
  k_timer_init(&ext_sync_timer, ext_sync_timer_handler, NULL);

  LOG_INF("External sync output initialized (%d ms half period)",
          CONFIG_MMWAVE_EXT_SYNC_PERIOD_MS);
  return 0;
}

static void ext_sync_start(void) {
  atomic_set(&ext_sync_state, 0);
  gpio_pin_set_dt(&ext_sync, 0);
  k_timer_start(&ext_sync_timer, K_MSEC(CONFIG_MMWAVE_EXT_SYNC_PERIOD_MS),
                K_MSEC(CONFIG_MMWAVE_EXT_SYNC_PERIOD_MS));
}

static void ext_sync_stop(void) {
  k_timer_stop(&ext_sync_timer);
  atomic_set(&ext_sync_state, 0);
  gpio_pin_set_dt(&ext_sync, 0);
}

#else /* !CONFIG_MMWAVE_EXT_SYNC */

static inline int ext_sync_init(void) { return 0; }
static inline void ext_sync_start(void) {}
static inline void ext_sync_stop(void) {}

#endif /* CONFIG_MMWAVE_EXT_SYNC */

/*==============================================================================
 * xensiv_bgt60trxx platform callbacks
 *============================================================================*/

void xensiv_bgt60trxx_platform_rst_set(const void *iface, bool val) {
  ARG_UNUSED(iface);
  gpio_pin_set_raw(rst_pin.port, rst_pin.pin, val ? 1 : 0);
}

void xensiv_bgt60trxx_platform_delay(uint32_t ms) { k_msleep(ms); }

uint32_t xensiv_bgt60trxx_platform_word_reverse(uint32_t x) {
  return __builtin_bswap32(x);
}

void xensiv_bgt60trxx_platform_assert(bool expr) { __ASSERT_NO_MSG(expr); }

/*==============================================================================
 * Private Functions
 *============================================================================*/

/**
 * @brief GPIO interrupt callback for the BGT60TR13C data-ready signal
 *
 * Signals the streaming thread that a full frame can be read from the FIFO.
 */
static void bgt60tr13c_irq_callback(const struct device *dev,
                                    struct gpio_callback *cb, uint32_t pins) {
  ARG_UNUSED(dev);
  ARG_UNUSED(cb);
  ARG_UNUSED(pins);

  k_sem_give(&data_ready_mmWave_sem);
}

/**
 * @brief Report why the radar failed to identify itself
 *
 * xensiv_bgt60trxx_init() only reports "device error" when the CHIP_ID read
 * comes back as something it does not recognise, which is the same symptom for
 * a radar that is unpowered, one on a miswired bus, and one whose bus timing is
 * wrong. This narrows it down: it prints the raw ID, decodes the two fields the
 * driver matches on (both must read 3 for a BGT60TR13C), dumps the SPI
 * peripheral state, and re-reads the ID at 4 MHz.
 *
 * Reading the log:
 *   - 0x000000 -> MISO never driven: no supply, held in reset, or MISO not
 *     reaching the nRF. Check VD2 and the shield's enable pin.
 *   - 0xFFFFFF -> MISO stuck high: shield absent, or CS never asserted.
 *   - Changes between runs, or differs between 8 MHz and 4 MHz -> bus timing.
 *     Pin CONFIG_MMWAVE_SPI_FREQ_MHZ to the value that works.
 *   - Stable but unrecognised -> a different XENSIV variant than BGT60TR13C.
 */
static void mmwave_log_probe_failure(void) {
  uint32_t chipid = 0;

  if (xensiv_bgt60trxx_get_reg(&dev, XENSIV_BGT60TRXX_REG_CHIP_ID, &chipid) ==
      XENSIV_BGT60TRXX_STATUS_OK) {
    uint32_t digital = (chipid & XENSIV_BGT60TRXX_REG_CHIP_ID_DIGITAL_ID_MSK) >>
                       XENSIV_BGT60TRXX_REG_CHIP_ID_DIGITAL_ID_POS;
    uint32_t rf = (chipid & XENSIV_BGT60TRXX_REG_CHIP_ID_RF_ID_MSK) >>
                  XENSIV_BGT60TRXX_REG_CHIP_ID_RF_ID_POS;
    LOG_ERR("CHIP_ID at %d MHz = 0x%06x (digital=%u, rf=%u; expected 3 and 3)",
            CONFIG_MMWAVE_SPI_FREQ_MHZ, chipid, digital, rf);
  } else {
    LOG_ERR("CHIP_ID re-read failed outright (SPI transfer error)");
  }

  /* Logged after the read above, so the captured per-transfer values belong to
   * a transfer whose result we just printed. */
  mmwave_spi_log_bus_state();

  /* Re-probe slower: a value that only decodes correctly at 4 MHz points at
   * MISO sampling rather than power or wiring. */
  if (CONFIG_MMWAVE_SPI_FREQ_MHZ != 4) {
    mmwave_spi_set_clock_mhz(4);
    if (xensiv_bgt60trxx_get_reg(&dev, XENSIV_BGT60TRXX_REG_CHIP_ID, &chipid) ==
        XENSIV_BGT60TRXX_STATUS_OK) {
      LOG_ERR("CHIP_ID at 4 MHz = 0x%06x", chipid);
    }
    mmwave_spi_set_clock_mhz(CONFIG_MMWAVE_SPI_FREQ_MHZ);
  }

#if defined(CONFIG_MMWAVE_PROBE_LOOP)
  /* Bring-up aid: a single register read is one ~4 us burst tens of
   * milliseconds after the command, which a logic analyser in auto-trigger mode
   * will almost never land on. Repeat the read for a couple of seconds so any
   * capture window contains traffic, and report whether MISO ever came up.
   *
   * Probe SCK (P0.08), MOSI (P0.09), MISO (P0.10) and CS (P1.11) during this
   * window: SCK and MOSI toggling while MISO stays flat means the radar is not
   * answering, which is wiring or radar state, not firmware. */
  LOG_INF("Probe loop: reading CHIP_ID for %d ms at %d MHz - capture now",
          CONFIG_MMWAVE_PROBE_LOOP_MS, CONFIG_MMWAVE_SPI_FREQ_MHZ);

  uint32_t reads = 0;
  uint32_t nonzero = 0;
  uint32_t first_nonzero = 0;

  for (int elapsed = 0; elapsed < CONFIG_MMWAVE_PROBE_LOOP_MS; elapsed++) {
    if (xensiv_bgt60trxx_get_reg(&dev, XENSIV_BGT60TRXX_REG_CHIP_ID, &chipid) ==
        XENSIV_BGT60TRXX_STATUS_OK) {
      reads++;
      if (chipid != 0) {
        if (nonzero == 0) {
          first_nonzero = chipid;
        }
        nonzero++;
      }
    }
    k_msleep(1);
  }

  LOG_INF("Probe loop done: %u reads, %u non-zero (first 0x%06x)", reads,
          nonzero, first_nonzero);
#endif /* CONFIG_MMWAVE_PROBE_LOOP */
}

/**
 * @brief Send one radar frame over BLE, split into chunked packets
 *
 * @param data     Frame payload
 * @param data_len Payload length in bytes
 */
static void mmwave_send_payload(const uint8_t *data, uint32_t data_len) {
  uint8_t total_chunks =
      (uint8_t)((data_len + MMWAVE_PAYLOAD_SIZE - 1U) / MMWAVE_PAYLOAD_SIZE);

  uint32_t timestamp = k_cyc_to_us_floor32(k_cycle_get_32());
  uint8_t sync_state = (uint8_t)atomic_get(&ext_sync_state) & 0x01U;

  /* Bit 0 of the timestamp carries the sync level (see the packet layout). */
  timestamp = (timestamp & ~0x01UL) | sync_state;

  mmWave_tx_buf[0] = MMWAVE_DATA_HEADER;
  mmWave_tx_buf[1] = (uint8_t)((timestamp >> 24) & 0xFF);
  mmWave_tx_buf[2] = (uint8_t)((timestamp >> 16) & 0xFF);
  mmWave_tx_buf[3] = (uint8_t)((timestamp >> 8) & 0xFF);
  mmWave_tx_buf[4] = (uint8_t)((timestamp) & 0xFF);
  mmWave_tx_buf[6] = total_chunks;
  mmWave_tx_buf[MMWAVE_TRAILER_OFFSET] = MMWAVE_DATA_TRAILER;

  for (uint8_t chunk = 0; chunk < total_chunks; chunk++) {
    uint32_t offset = chunk * MMWAVE_PAYLOAD_SIZE;
    uint32_t chunk_len = MIN(MMWAVE_PAYLOAD_SIZE, data_len - offset);

    mmWave_tx_buf[5] = chunk;

    memcpy(&mmWave_tx_buf[MMWAVE_PAYLOAD_OFFSET], &data[offset], chunk_len);

    if (chunk_len < MMWAVE_PAYLOAD_SIZE) {
      memset(&mmWave_tx_buf[MMWAVE_PAYLOAD_OFFSET + chunk_len], 0,
             MMWAVE_PAYLOAD_SIZE - chunk_len);
    }

    add_data_to_send_buffer(mmWave_tx_buf, MMWAVE_PACKET_SIZE);
  }
}

/**
 * @brief Send the current radar frame, 12-bit packed as it left the FIFO
 */
static void mmwave_send_frame_packed(void) {
  mmwave_send_payload(mmwave_spi_get_packed_frame(),
                      MMWAVE_FRAME_SIZE_BYTES_PACKED);
}

/**
 * @brief Send the current radar frame as unpacked uint16_t samples
 */
static void mmwave_send_frame_u16(void) {
  mmwave_send_payload((const uint8_t *)samples, MMWAVE_FRAME_SIZE_BYTES_U16);
}

/**
 * @brief Adopt the configurable register values from the generated header
 *
 * The runtime setters mutate the IF gain, TX power and frame-rate registers,
 * so they must start from whatever the compiled-in register list contains
 * rather than from hardcoded defaults.
 */
static void mmWave_sync_config_from_header(void) {
  bool found_gain = false;

  for (int i = 0; i < XENSIV_BGT60TRXX_CONF_NUM_REGS; i++) {
    uint8_t addr = (uint8_t)(register_list[i] >> XENSIV_BGT60TRXX_SPI_REGADR_POS);

    switch (addr) {
    case 0x06:
      current_fps_reg06 = register_list[i];
      LOG_DBG("Synced fps reg 0x06 from header: 0x%08x", current_fps_reg06);
      break;
    case 0x11:
      current_tx_power_reg = register_list[i];
      LOG_DBG("Synced tx power from header: 0x%08x", current_tx_power_reg);
      break;
    case 0x12:
      current_selected_gain_reg = register_list[i];
      found_gain = true;
      LOG_DBG("Synced gain from header: 0x%08x", current_selected_gain_reg);
      break;
    case 0x2d:
      current_fps_reg2d = register_list[i];
      LOG_DBG("Synced fps reg 0x2d from header: 0x%08x", current_fps_reg2d);
      break;
    default:
      break;
    }
  }

  if (!found_gain) {
    LOG_WRN("Gain register 0x12 not found in header, using hardcoded default");
  }
}

/**
 * @brief Write one packed register value from the generated header to the radar
 *
 * @param dev      Driver instance
 * @param full_reg Packed value carrying both the address and the data field
 *
 * @return XENSIV_BGT60TRXX_STATUS_OK on success
 */
static int mmWave_apply_reg_to_hw(xensiv_bgt60trxx_t *dev, uint32_t full_reg) {
  uint32_t reg_addr = ((full_reg & XENSIV_BGT60TRXX_SPI_REGADR_MSK) >>
                       XENSIV_BGT60TRXX_SPI_REGADR_POS);
  uint32_t reg_data = ((full_reg & XENSIV_BGT60TRXX_SPI_DATA_MSK) >>
                       XENSIV_BGT60TRXX_SPI_DATA_POS);

  int ret = xensiv_bgt60trxx_set_reg(dev, reg_addr, reg_data);
  if (ret != XENSIV_BGT60TRXX_STATUS_OK) {
    LOG_ERR("Register 0x%02x write failed: %d", (unsigned int)reg_addr, ret);
  }
  return ret;
}

/** @brief Apply the currently selected IF gain register value */
static int mmWave_apply_gain_to_hw(xensiv_bgt60trxx_t *dev) {
  return mmWave_apply_reg_to_hw(dev, current_selected_gain_reg);
}

/** @brief Apply the currently selected TX power register value */
static int mmWave_apply_tx_power_to_hw(xensiv_bgt60trxx_t *dev) {
  return mmWave_apply_reg_to_hw(dev, current_tx_power_reg);
}

/** @brief Apply both currently selected frame-rate register values */
static int mmWave_apply_fps_to_hw(xensiv_bgt60trxx_t *dev) {
  int ret = mmWave_apply_reg_to_hw(dev, current_fps_reg06);
  if (ret != XENSIV_BGT60TRXX_STATUS_OK) {
    LOG_ERR("Failed to apply FPS register 0x06: %d", ret);
    return ret;
  }

  ret = mmWave_apply_reg_to_hw(dev, current_fps_reg2d);
  if (ret != XENSIV_BGT60TRXX_STATUS_OK) {
    LOG_ERR("Failed to apply FPS register 0x2d: %d", ret);
  }
  return ret;
}

/**
 * @brief Main mmWave streaming thread
 *
 * Waits for a start signal, starts frame acquisition, reads a frame on every
 * data-ready interrupt and forwards it over BLE until streaming is stopped.
 */
/**
 * @brief Report why a FIFO read failed, naming the latched error bits.
 *
 * The driver collapses every GSR0 error into a single status code, so the
 * status register has to be re-read to say which one it was. FOF/FUF mean the
 * FIFO over- or underflowed, i.e. the reads did not keep up with the frame
 * rate; SPI_BURST and CLK_NUM point at the bus instead.
 */
static void mmwave_log_fifo_error(int32_t status, uint32_t frame_idx) {
  uint32_t fstat = 0;

  if (status == XENSIV_BGT60TRXX_STATUS_GSR0_ERROR &&
      xensiv_bgt60trxx_get_fifo_status(&dev, &fstat) ==
          XENSIV_BGT60TRXX_STATUS_OK) {
    LOG_ERR("FIFO read failed at frame %u: FSTAT 0x%06x%s%s%s%s", frame_idx,
            fstat,
            (fstat & XENSIV_BGT60TRXX_REG_FSTAT_FOF_ERR_MSK) ? " overflow" : "",
            (fstat & XENSIV_BGT60TRXX_REG_FSTAT_FUF_ERR_MSK) ? " underflow" : "",
            (fstat & XENSIV_BGT60TRXX_REG_FSTAT_SPI_BURST_ERR_MSK)
                ? " spi-burst"
                : "",
            (fstat & XENSIV_BGT60TRXX_REG_FSTAT_CLK_NUM_ERR_MSK) ? " clk-num"
                                                                 : "");
  } else {
    LOG_ERR("FIFO read failed at frame %u: status %d", frame_idx, status);
  }
}

/**
 * @brief Check whether the radar is still answering, and report its state.
 *
 * Every write-then-poll helper in the driver treats an all-zero read as a bit
 * that has cleared, so a link that returns nothing makes soft_reset() and
 * start_frame() report success without anything having happened. Their return
 * codes therefore say nothing about whether the radar is alive.
 *
 * CHIP_ID is the cheapest read with a known non-zero answer, which makes it the
 * one thing that separates a radar which stopped generating frames (CHIP_ID
 * still correct) from a link which stopped carrying answers (CHIP_ID zero).
 * MAIN shows whether FRAME_START is actually set, and FSTAT whether the FIFO
 * reports itself empty -- an all-zero FSTAT is itself impossible, since an empty
 * FIFO must set its EMPTY flag.
 *
 * @param when Short label naming the point in the sequence, for the log.
 * @return true when CHIP_ID identifies a BGT60TR13C.
 */
static bool mmwave_probe_link(const char *when) {
  uint32_t chipid = 0;
  uint32_t main_reg = 0;
  uint32_t fstat = 0;
  bool alive;

  if (xensiv_bgt60trxx_get_reg(&dev, XENSIV_BGT60TRXX_REG_CHIP_ID, &chipid) !=
      XENSIV_BGT60TRXX_STATUS_OK) {
    LOG_ERR("Link probe (%s): CHIP_ID read failed outright", when);
    return false;
  }

  (void)xensiv_bgt60trxx_get_reg(&dev, XENSIV_BGT60TRXX_REG_MAIN, &main_reg);
  (void)xensiv_bgt60trxx_get_fifo_status(&dev, &fstat);

  alive = (((chipid & XENSIV_BGT60TRXX_REG_CHIP_ID_DIGITAL_ID_MSK) >>
            XENSIV_BGT60TRXX_REG_CHIP_ID_DIGITAL_ID_POS) == 3U) &&
          (((chipid & XENSIV_BGT60TRXX_REG_CHIP_ID_RF_ID_MSK) >>
            XENSIV_BGT60TRXX_REG_CHIP_ID_RF_ID_POS) == 3U);

  LOG_ERR("Link probe (%s): CHIP_ID 0x%06x %s, MAIN 0x%06x, FSTAT 0x%06x", when,
          chipid, alive ? "radar answering" : "NO ANSWER", main_reg, fstat);

  return alive;
}

/**
 * @brief Rebuild frame capture after repeated frame losses.
 *
 * A FIFO overflow latches a sticky error bit that makes every later burst read
 * fail, so waiting does not help: the capture has to be torn down and started
 * again. Stop the chirp FSM, clear the FIFO (which also clears the error bits),
 * discard any interrupt the aborted frame left pending, then resume framing --
 * the same order the session start uses.
 *
 * Escalates on later attempts. A FIFO reset cannot revive a device whose
 * register state has been corrupted, nor one whose chirp FSM will not restart
 * from FRAME_START alone; from the second attempt the whole profile is rewritten
 * instead, which begins with a full software reset of the device.
 *
 * @param attempt 1-based recovery attempt; >1 rewrites the register profile.
 * @return XENSIV_BGT60TRXX_STATUS_OK when capture is running again.
 */
static int32_t mmwave_recover_capture(uint32_t attempt) {
  const bool full = (attempt > 1U);
  int32_t ret;

  /* What a restart can possibly achieve depends on whether the radar is still
   * answering, and that is not visible from any return code below. */
  (void)mmwave_probe_link(full ? "before profile rewrite" : "before restart");

  ret = xensiv_bgt60trxx_start_frame(&dev, false);
  if (ret != XENSIV_BGT60TRXX_STATUS_OK) {
    LOG_ERR("Recovery: stopping frame capture failed: %d", ret);
    return ret;
  }

  if (full) {
    ret = xensiv_bgt60trxx_config(&dev, register_list,
                                  XENSIV_BGT60TRXX_CONF_NUM_REGS);
    if (ret != XENSIV_BGT60TRXX_STATUS_OK) {
      LOG_ERR("Recovery: rewriting the register profile failed: %d", ret);
      return ret;
    }

    /* config() restores the compiled-in profile, so the runtime settings the
     * host chose have to be reapplied on top of it. */
    if (mmWave_apply_gain_to_hw(&dev) != XENSIV_BGT60TRXX_STATUS_OK ||
        mmWave_apply_tx_power_to_hw(&dev) != XENSIV_BGT60TRXX_STATUS_OK ||
        mmWave_apply_fps_to_hw(&dev) != XENSIV_BGT60TRXX_STATUS_OK) {
      LOG_ERR("Recovery: reapplying runtime settings failed");
      return XENSIV_BGT60TRXX_STATUS_COM_ERROR;
    }

    xensiv_bgt60trxx_set_fifo_limit(&dev, MMWAVE_NUM_SAMPLES_PER_FRAME);
    xensiv_bgt60trxx_enable_data_test_mode(&dev, MMWAVE_TESTMODE);
  } else {
    ret = xensiv_bgt60trxx_soft_reset(&dev, XENSIV_BGT60TRXX_RESET_FIFO);
    if (ret != XENSIV_BGT60TRXX_STATUS_OK) {
      LOG_ERR("Recovery: FIFO reset failed: %d", ret);
      return ret;
    }
  }

  k_sem_reset(&data_ready_mmWave_sem);

  /* Re-arm the edge interrupt now that the FIFO is empty and the data-ready
   * line is therefore low. Arming while it is still high would latch nothing,
   * since the rising edge has already passed. */
  gpio_pin_interrupt_configure_dt(&irq_pin, GPIO_INT_DISABLE);
  gpio_pin_interrupt_configure_dt(&irq_pin, GPIO_INT_EDGE_TO_ACTIVE);

  ret = xensiv_bgt60trxx_start_frame(&dev, true);
  if (ret != XENSIV_BGT60TRXX_STATUS_OK) {
    LOG_ERR("Recovery: restarting frame capture failed: %d", ret);
    return ret;
  }

  /* start_frame() cannot fail visibly on a silent link, so confirm separately
   * that the radar is answering and that FRAME_START actually took. */
  (void)mmwave_probe_link("after restart");

  return ret;
}

static void mmWave_streaming_thread(void *arg1, void *arg2, void *arg3) {
  ARG_UNUSED(arg1);
  ARG_UNUSED(arg2);
  ARG_UNUSED(arg3);

  int ret;
  uint32_t frame_idx;
  uint32_t errors;
  uint16_t test_word;
  uint32_t frame_errors;
  uint32_t recoveries;
  uint32_t level_reads;
  bool frame_ready;

  LOG_INF("mmWave streaming thread started");

  while (1) {
    k_sem_take(&mmWave_start_sem, K_FOREVER);

    /* Per-session state: the radar restarts its test-pattern LFSR from the
     * initial word on every capture, so it must be re-seeded here rather than
     * carried over from the previous session. */
    frame_idx = 0;
    errors = 0;
    test_word = XENSIV_BGT60TRXX_INITIAL_TEST_WORD;
    frame_errors = 0;
    recoveries = 0;
    level_reads = 0;

    ret = xensiv_bgt60trxx_soft_reset(&dev, XENSIV_BGT60TRXX_RESET_FIFO);
    if (ret != XENSIV_BGT60TRXX_STATUS_OK) {
      LOG_ERR("FIFO reset failed: %d", ret);
      mmWave_state = mmWave_STATE_ERROR;
      continue;
    }

    /* The FIFO-limit interrupt stays armed between stop and power-off, so a
     * give left over from the previous session would make the first take below
     * return immediately and read a partially filled FIFO. */
    k_sem_reset(&data_ready_mmWave_sem);

    ext_sync_start();

    ret = xensiv_bgt60trxx_start_frame(&dev, true);
    if (ret != XENSIV_BGT60TRXX_STATUS_OK) {
      LOG_ERR("Starting frame capture failed: %d", ret);
      ext_sync_stop();
      mmWave_state = mmWave_STATE_ERROR;
      continue;
    }

    LOG_INF("Starting mmWave capture");
    mmWave_state = mmWave_STATE_STREAMING;

    while (mmWave_keep_running) {
      /* The radar's data-ready output is a level, not a pulse: it stays
       * asserted while the FIFO holds at least one frame (fill > FIFO_CREF).
       * The GPIO is armed for a rising edge, which only occurs when the fill
       * crosses the limit from below -- so if a second frame lands before the
       * previous read finishes, the line never falls and no further edge is
       * ever produced. The interrupt would then be gone for good while the
       * radar keeps running and SPI keeps answering.
       *
       * Testing the level first makes that unobservable: an asserted line
       * already means a full frame is waiting, so read it instead of waiting
       * for an edge that cannot come. A backlog drains one frame per iteration
       * until the line falls, and only then is the edge waited for again. */
      if (gpio_pin_get_dt(&irq_pin) == 1) {
        /* Absorb the give belonging to this frame, if the ISR did fire, so a
         * stale count cannot later trigger a read of an empty FIFO. */
        k_sem_take(&data_ready_mmWave_sem, K_NO_WAIT);
        frame_ready = true;
        level_reads++;
      } else {
        frame_ready = (k_sem_take(&data_ready_mmWave_sem,
                                  K_MSEC(MMWAVE_IRQ_TIMEOUT_MS)) == 0);
        if (!frame_ready) {
          /* No edge, and the line is not raised either, so this is not a missed
           * edge: the radar has genuinely stopped producing frames. */
          LOG_ERR("IRQ timeout at frame %u - no radar data (irq line %d)",
                  frame_idx, gpio_pin_get_dt(&irq_pin));
        }
      }

      if (frame_ready) {
        ret = xensiv_bgt60trxx_get_fifo_data(&dev, samples,
                                             MMWAVE_NUM_SAMPLES_PER_FRAME);
        if (ret == XENSIV_BGT60TRXX_STATUS_OK) {
          frame_errors = 0;

          if (MMWAVE_TESTMODE) {
            /* Validate the FIFO content against the radar's internal LFSR
             * pattern */
            for (int32_t i = 0; i < MMWAVE_NUM_SAMPLES_PER_FRAME; ++i) {
              if (test_word != samples[i]) {
                if (errors == 0) {
                  LOG_WRN("Frame %u mismatch: expected %u, received %u",
                          frame_idx, test_word, samples[i]);
                }
                errors++;
              }
              test_word = xensiv_bgt60trxx_get_next_test_word(test_word);
            }
          }

          if (IS_ENABLED(CONFIG_MMWAVE_SEND_PACKED_12BIT)) {
            mmwave_send_frame_packed();
          } else {
            mmwave_send_frame_u16();
          }
          frame_idx++;

          if (MMWAVE_TESTMODE && (frame_idx % 25U) == 0U) {
            LOG_INF("%u test-pattern errors in the last 25 frames", errors);
            errors = 0;
          }
          continue;
        }

        mmwave_log_fifo_error(ret, frame_idx);
      }

      /* This frame is lost. An isolated loss only costs the host a gap, so it
       * is not worth ending the session over -- but a run of them means the
       * radar has latched a FIFO error or stopped signalling, and neither
       * clears by waiting. Rebuild the capture instead of spinning. */
      frame_errors++;
      if (frame_errors == 1U) {
        /* Probe immediately, while the failure is fresh and before any reset has
         * disturbed the evidence. This is what says whether the radar stopped or
         * the link went silent. */
        (void)mmwave_probe_link("first loss");
      }
      if (frame_errors < MMWAVE_FRAME_ERROR_LIMIT) {
        continue;
      }

      recoveries++;
      if (recoveries > MMWAVE_RECOVERY_LIMIT) {
        LOG_ERR("Radar unrecoverable after %u restarts; ending capture",
                MMWAVE_RECOVERY_LIMIT);
        break;
      }

      LOG_WRN("Restarting radar capture (%u/%u) after %u consecutive frame "
              "errors at frame %u",
              recoveries, MMWAVE_RECOVERY_LIMIT, frame_errors, frame_idx);

      if (mmwave_recover_capture(recoveries) != XENSIV_BGT60TRXX_STATUS_OK) {
        break;
      }

      frame_errors = 0;
      /* Framing restarted, so the radar's test-pattern LFSR restarts too. */
      test_word = XENSIV_BGT60TRXX_INITIAL_TEST_WORD;
    }

    /* The loop also exits on its own when the radar cannot be recovered, in
     * which case the flag is still set; clear it so it always agrees with the
     * state. Written before the state reaches CONFIGURED again, so it cannot
     * clobber a start request. */
    mmWave_keep_running = false;
    mmWave_state = mmWave_STATE_STOPPING;

    ret = xensiv_bgt60trxx_start_frame(&dev, false);
    if (ret != XENSIV_BGT60TRXX_STATUS_OK) {
      LOG_ERR("Stopping frame capture failed: %d", ret);
      mmWave_state = mmWave_STATE_ERROR;
      continue;
    }

    ext_sync_stop();

    ret = xensiv_bgt60trxx_soft_reset(&dev, XENSIV_BGT60TRXX_RESET_FIFO);
    if (ret != XENSIV_BGT60TRXX_STATUS_OK) {
      LOG_ERR("FIFO reset failed: %d", ret);
      mmWave_state = mmWave_STATE_ERROR;
      continue;
    }

    mmWave_state = mmWave_STATE_CONFIGURED;
    /* level_reads counts frames taken because the data-ready line was already
     * asserted, i.e. edges that would otherwise have been missed. A steady
     * non-zero count is normal under load; before the level check each one of
     * them would have ended the session. */
    LOG_INF("mmWave streaming stopped after %u frames (%u read on level)",
            frame_idx, level_reads);
  }
}

/*==============================================================================
 * Thread Definition
 *============================================================================*/

K_THREAD_DEFINE(mmWave_thread_id, mmWave_THREAD_STACK_SIZE,
                mmWave_streaming_thread, NULL, NULL, NULL, mmWave_PRIORITY, 0, 0);

/*==============================================================================
 * Public Functions
 *============================================================================*/

int mmWave_HW_init(void) {
  int ret;

  if (mmWave_state != mmWave_STATE_NO_HW) {
    LOG_DBG("mmWave HW already initialized");
    return -EALREADY;
  }

  LOG_INF("Starting mmWave HW initialization...");

  if (!gpio_is_ready_dt(&irq_pin) || !gpio_is_ready_dt(&rst_pin) ||
      !gpio_is_ready_dt(&pwr_pin)) {
    LOG_ERR("mmWave control GPIOs not ready");
    return -ENODEV;
  }

  /* Bring the shield's VD2 rail up here, at boot, rather than inside
   * mmWave_power_on(). The radar only sees this rail once its local enable pin
   * is asserted, but raising a PMIC buck-boost is not instantaneous, and doing
   * it moments before the enable pin leaves the radar's power-on reset racing a
   * still-settling supply. Bringing it up at init gives it seconds to settle and
   * matches the single-owner reference implementation, which configured this
   * rail during boot-time PMIC setup.
   *
   * The rail draws nothing extra while the radar is off: the shield gates it
   * locally with the enable pin. */
  ret = mmwave_shield_power_on();
  if (ret != 0) {
    LOG_ERR("Failed to power the mmWave shield rail: %d", ret);
    return ret;
  }

  ret = ext_sync_init();
  if (ret != 0) {
    return ret;
  }

  ret = mmwave_spi_init();
  if (ret != 0) {
    return ret;
  }

  ret = gpio_pin_configure_dt(&irq_pin, GPIO_INPUT);
  if (ret < 0) {
    LOG_ERR("Failed to configure IRQ pin %d: %d", irq_pin.pin, ret);
    return ret;
  }
  gpio_init_callback(&irq_cb_data, bgt60tr13c_irq_callback, BIT(irq_pin.pin));
  gpio_add_callback(irq_pin.port, &irq_cb_data);

  /* Drive RST active (physically low, the pin is active-low) so the radar sees
   * no voltage on its inputs while its supply rail is still off. */
  ret = gpio_pin_configure_dt(&rst_pin, GPIO_OUTPUT_ACTIVE);
  if (ret < 0) {
    LOG_ERR("Failed to configure RST pin %d: %d", rst_pin.pin, ret);
    return ret;
  }

  /* Leave the power pin disconnected so the battery-monitor ADC keeps using
   * it until the radar is actually powered on. */
  ret = gpio_pin_configure_dt(&pwr_pin, GPIO_DISCONNECTED);
  if (ret < 0) {
    LOG_ERR("Failed to configure PWR pin %d: %d", pwr_pin.pin, ret);
    return ret;
  }

  mmWave_sync_config_from_header();

  mmWave_state = mmWave_STATE_HW_ACTIVE;
  LOG_INF("mmWave HW initialized (%d chirps x %d samples x %d RX = %d samples/frame)",
          XENSIV_BGT60TRXX_CONF_NUM_CHIRPS_PER_FRAME,
          XENSIV_BGT60TRXX_CONF_NUM_SAMPLES_PER_CHIRP,
          XENSIV_BGT60TRXX_CONF_NUM_RX_ANTENNAS, MMWAVE_NUM_SAMPLES_PER_FRAME);
  return 0;
}

int mmWave_power_on(void) {
  if (mmWave_state != mmWave_STATE_HW_ACTIVE) {
    LOG_ERR("mmWave HW not ready or already powered on (state %d)", mmWave_state);
    return -EPERM;
  }

  /* Take the power pin over from the battery-monitor ADC: it is shared with
   * SAADC AIN3, so battery telemetry is suppressed until power_off() (see
   * mmWave_is_powered(), which gates the PMIC measurement in main.c). Claim it
   * before driving the pin, so no measurement can slip in between.
   *
   * Disconnecting the SAADC input as well is what the reference implementation
   * does. The gate alone stops new conversions, but it leaves the analog mux
   * pointed at this pad; releasing it too keeps the radar's enable line free of
   * the converter's sampling network entirely. */
  mmWave_owns_pwr_pin = true;
  nrf_saadc_channel_input_set(NRF_SAADC, 0, NRF_SAADC_INPUT_DISABLED,
                              NRF_SAADC_INPUT_DISABLED);

  gpio_pin_configure_dt(&pwr_pin, GPIO_OUTPUT_ACTIVE);
  gpio_pin_set_dt(&pwr_pin, 1);

  /* Order here is deliberate and matches the reference implementation exactly:
   * the rail comes up with CS still parked LOW, the supply is given 5 ms to
   * settle, and only then is CS raised, immediately before the reset pulse.
   *
   * Do not "improve" this by raising CS earlier. The radar multiplexes its
   * reset input with the quad-SPI DIO3 line, so the state of these pins while
   * its power-on reset runs is what selects the interface mode it comes up in.
   * Raising CS before the supply settles was tried, and the radar then never
   * answered a single register read: 2000 CHIP_ID reads returned zero with a
   * perfect clock, chip select and MOSI on the wire. */
  k_msleep(5);
  xensiv_bgt60trxx_platform_spi_cs_set(mmwave_spi_iface, true);

  gpio_pin_set_dt(&rst_pin, 0);
  k_msleep(1);
  gpio_pin_set_dt(&rst_pin, 1);
  k_msleep(1);
  gpio_pin_set_dt(&rst_pin, 0);
  k_msleep(1);

  int ret = XENSIV_BGT60TRXX_STATUS_DEV_ERROR;
  for (int attempt = 1; attempt <= MMWAVE_PROBE_ATTEMPTS; attempt++) {
    ret = xensiv_bgt60trxx_init(&dev, mmwave_spi_iface, false);
    if (ret == XENSIV_BGT60TRXX_STATUS_OK) {
      break;
    }
    LOG_WRN("BGT60 probe attempt %d/%d failed: %d", attempt,
            MMWAVE_PROBE_ATTEMPTS, ret);
    k_msleep(MMWAVE_PROBE_RETRY_MS);
  }

  if (ret != XENSIV_BGT60TRXX_STATUS_OK) {
    LOG_ERR("BGT60 init error: %d", ret);
    mmwave_log_probe_failure();
    return -EIO;
  }

  mmWave_state = mmWave_STATE_IDLE;
  LOG_INF("mmWave powered on and ready for config");
  return 0;
}

bool mmWave_is_powered(void) { return mmWave_owns_pwr_pin; }

int mmWave_configure(void) {
  if (mmWave_state != mmWave_STATE_IDLE &&
      mmWave_state != mmWave_STATE_CONFIGURED) {
    LOG_ERR("mmWave not ready for configuration (state %d)", mmWave_state);
    return -EPERM;
  }

  int ret = xensiv_bgt60trxx_config(&dev, register_list,
                                    XENSIV_BGT60TRXX_CONF_NUM_REGS);
  if (ret != XENSIV_BGT60TRXX_STATUS_OK) {
    LOG_ERR("BGT60 config error: %d", ret);
    return -EIO;
  }

  if (mmWave_apply_gain_to_hw(&dev) != XENSIV_BGT60TRXX_STATUS_OK) {
    return -EIO;
  }

  if (mmWave_apply_tx_power_to_hw(&dev) != XENSIV_BGT60TRXX_STATUS_OK) {
    return -EIO;
  }

  if (mmWave_apply_fps_to_hw(&dev) != XENSIV_BGT60TRXX_STATUS_OK) {
    return -EIO;
  }

  xensiv_bgt60trxx_set_fifo_limit(&dev, MMWAVE_NUM_SAMPLES_PER_FRAME);
  xensiv_bgt60trxx_enable_data_test_mode(&dev, MMWAVE_TESTMODE);

  gpio_pin_interrupt_configure_dt(&irq_pin, GPIO_INT_EDGE_TO_ACTIVE);

  k_msleep(3);
  mmWave_state = mmWave_STATE_CONFIGURED;
  LOG_INF("mmWave configured successfully");
  return 0;
}

int mmWave_power_off(void) {
  if (mmWave_state == mmWave_STATE_NO_HW) {
    /* Without mmWave_HW_init() the GPIOs are unconfigured and SPI_A may not be
     * up; claiming HW_ACTIVE here would let a later power_on() drive an
     * uninitialised peripheral. */
    LOG_ERR("mmWave HW not initialized");
    return -EPERM;
  }

  if (mmWave_state == mmWave_STATE_STREAMING ||
      mmWave_state == mmWave_STATE_STOPPING) {
    LOG_ERR("mmWave still streaming");
    return -EPERM;
  }

  gpio_pin_interrupt_configure_dt(&irq_pin, GPIO_INT_DISABLE);

  /* Drop the supply and drive every radar input low, so no voltage reaches an
   * unpowered device. Both pins are active-low in the devicetree, hence the
   * logical 1 on RST; CS is parked through its own helper because the driver's
   * deassert callback drives it physically high, which is right for a powered
   * radar but not for one about to lose its rail. */
  gpio_pin_set_dt(&rst_pin, 1);
  mmwave_spi_park_cs();
  gpio_pin_set_dt(&pwr_pin, 0);

  k_msleep(5);

  /* Hand the pin back to the battery-monitor ADC: release the GPIO, restore the
   * analog input, and only then reopen the measurement gate. */
  gpio_pin_configure_dt(&pwr_pin, GPIO_DISCONNECTED);
  nrf_saadc_channel_input_set(NRF_SAADC, 0, NRF_SAADC_INPUT_AIN3,
                              NRF_SAADC_INPUT_DISABLED);
  mmWave_owns_pwr_pin = false;

  mmWave_state = mmWave_STATE_HW_ACTIVE;
  LOG_INF("mmWave powered down, pin freed for ADC");
  return 0;
}

int mmWave_set_ifGain(uint8_t ifGain) {
  switch (ifGain) {
  case 18: current_selected_gain_reg = if_gain_regs[0]; break;
  case 23: current_selected_gain_reg = if_gain_regs[1]; break;
  case 28: current_selected_gain_reg = if_gain_regs[2]; break;
  case 30: current_selected_gain_reg = if_gain_regs[3]; break;
  case 33: current_selected_gain_reg = if_gain_regs[4]; break;
  case 35: current_selected_gain_reg = if_gain_regs[5]; break;
  case 38: current_selected_gain_reg = if_gain_regs[6]; break;
  case 40: current_selected_gain_reg = if_gain_regs[7]; break;
  case 43: current_selected_gain_reg = if_gain_regs[8]; break;
  case 45: current_selected_gain_reg = if_gain_regs[9]; break;
  case 48: current_selected_gain_reg = if_gain_regs[10]; break;
  case 50: current_selected_gain_reg = if_gain_regs[11]; break;
  case 55: current_selected_gain_reg = if_gain_regs[12]; break;
  case 60: current_selected_gain_reg = if_gain_regs[13]; break;
  default:
    LOG_ERR("Invalid IF gain: %d dB", ifGain);
    return -EINVAL;
  }

  if (mmWave_state == mmWave_STATE_CONFIGURED) {
    if (mmWave_apply_gain_to_hw(&dev) != XENSIV_BGT60TRXX_STATUS_OK) {
      LOG_ERR("Failed to update IF gain on the fly");
      return -EIO;
    }
    LOG_INF("IF gain updated to %d dB", ifGain);
  } else {
    LOG_INF("IF gain saved, will be applied at the next configure");
  }
  return 0;
}

int mmWave_set_txPower(uint8_t txPower) {
  if (txPower > TX_POWER_MASK) {
    LOG_ERR("TX power %d out of range 0-31", txPower);
    return -EINVAL;
  }

  current_tx_power_reg &= ~(uint32_t)TX_POWER_MASK;
  current_tx_power_reg |= (txPower & TX_POWER_MASK);

  if (mmWave_state == mmWave_STATE_CONFIGURED) {
    if (mmWave_apply_tx_power_to_hw(&dev) != XENSIV_BGT60TRXX_STATUS_OK) {
      LOG_ERR("Failed to update TX power on the fly");
      return -EIO;
    }
    LOG_INF("TX power updated to %d", txPower);
  } else {
    LOG_INF("TX power saved, will be applied at the next configure");
  }
  return 0;
}

int mmWave_set_fps(uint8_t fps) {
  switch (fps) {
  case 25:
    current_fps_reg06 = fps_reg0x06[0];
    current_fps_reg2d = fps_reg0x2d[0];
    break;
  case 50:
    current_fps_reg06 = fps_reg0x06[1];
    current_fps_reg2d = fps_reg0x2d[1];
    break;
  case 100:
    current_fps_reg06 = fps_reg0x06[2];
    current_fps_reg2d = fps_reg0x2d[2];
    break;
  case 150:
    current_fps_reg06 = fps_reg0x06[3];
    current_fps_reg2d = fps_reg0x2d[3];
    break;
  case 200:
    current_fps_reg06 = fps_reg0x06[4];
    current_fps_reg2d = fps_reg0x2d[4];
    break;
  default:
    LOG_ERR("Invalid frame rate: %d fps", fps);
    return -EINVAL;
  }

  if (mmWave_state == mmWave_STATE_CONFIGURED) {
    if (mmWave_apply_fps_to_hw(&dev) != XENSIV_BGT60TRXX_STATUS_OK) {
      LOG_ERR("Failed to update frame rate on the fly");
      return -EIO;
    }
    LOG_INF("Frame rate updated to %d fps", fps);
  } else {
    LOG_INF("Frame rate saved, will be applied at the next configure");
  }
  return 0;
}

int mmWave_start_streaming(void) {
  if (mmWave_state == mmWave_STATE_STREAMING) {
    LOG_WRN("mmWave already streaming");
    return -EALREADY;
  }

  if (mmWave_state != mmWave_STATE_CONFIGURED) {
    LOG_ERR("mmWave not in configured state (current: %d)", mmWave_state);
    return -EBUSY;
  }

  mmWave_keep_running = true;
  k_sem_give(&mmWave_start_sem);

  return 0;
}

int mmWave_stop_streaming(void) {
  if (mmWave_state != mmWave_STATE_STREAMING) {
    LOG_WRN("mmWave not streaming");
    return -EINVAL;
  }

  mmWave_keep_running = false;

  /* The streaming thread only notices the flag after its next frame, so the
   * worst case is a full MMWAVE_IRQ_TIMEOUT_MS plus the stop sequence (FSM
   * reset, FIFO reset, several SPI round trips). Allow twice the IRQ timeout. */
  int timeout = 200;
  while (mmWave_state != mmWave_STATE_CONFIGURED && timeout > 0) {
    k_msleep(10);
    timeout--;
  }

  if (timeout == 0) {
    LOG_ERR("Timeout waiting for mmWave to stop");
    return -ETIMEDOUT;
  }

  return 0;
}
