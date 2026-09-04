/*
 * Multi-sensor MAXM86161 PPG application layer.
 *
 * Supports 1–8 MAXM86161 sensors connected via a TCA9548A I2C MUX on I2C_B.
 * Active sensors are selected by CONFIG_PPG_SENSOR_MASK (bit N = channel N).
 * All sensors are configured simultaneously using MUX broadcast mode, then
 * sampled synchronously via a GPIO trigger pulse (P0.13 → MAXM86161 GPIO pin).
 * Data is streamed per-channel over BLE NUS.
 *
 * Hardware connections assumed:
 *   I2C_B (i2c1):  SCL=P1.10, SDA=P0.24
 *   TCA9548A:       I2C_B @ 0x70, channel N → MAXM86161 sensor N
 *   GPIO sync out:  P0.31 → all MAXM86161 GPIO inputs (shorted)
 *   INTB wired-OR:  all MAXM86161 INTB (open-drain) → P0.12
 *
 * NOTE: P0.12 is shared with the PDM microphone DATA line in this overlay.
 */

#ifndef PPG_NEW_APPL_H
#define PPG_NEW_APPL_H

#include <stdint.h>
#include <stdbool.h>
#include <zephyr/drivers/gpio.h>

/* Exported GPIO resource for external sync control. */
extern const struct gpio_dt_spec ppg_sync_gpio;

typedef enum {
    PPG_NEW_STATE_IDLE      = 0,
    PPG_NEW_STATE_STARTING  = 1,
    PPG_NEW_STATE_STREAMING = 2,
    PPG_NEW_STATE_STOPPING  = 3,
    PPG_NEW_STATE_ERROR     = 4,
} ppg_new_state_t;

/**
 * @brief Runtime sensor configuration passed with the START_PPG_STREAMING command.
 *
 * Byte mapping in the BLE packet (bytes 2–12, i.e. after the command code):
 *   [0] sensor_mask      – bitmask of active MUX channels (bit N = channel N)
 *   [1] sample_rate_hz   – trigger rate in Hz (must be >= 8)
 *   [2] led_green        – green LED PA (0x00 = disable, 0x01–0xFF = current)
 *   [3] led_ir           – IR    LED PA (0x00 = disable, 0x01–0xFF = current)
 *   [4] led_red          – red   LED PA (0x00 = disable, 0x01–0xFF = current)
 *   [5] led_range        – 0=4k 1=8k 2=16k 3=32k  (enum maxm86161_led_current)
 *   [6] tint             – 0=14.8µs 1=29.4µs 2=58.7µs 3=117.3µs
 *   [7] adc_range        – 0=4k 1=8k 2=16k 3=32k  (enum maxm86161_ppg_cfg_led_range)
 *   [8] sample_avg       – 0=1x 1=2x 2=4x 3=8x 4=16x 5=32x 6=64x 7=128x
 *   [9] alc_enable       – 0=off 1=on
 *  [10] proximity_enable – 0=off 1=run proximity detection before streaming
 */
typedef struct {
    uint8_t sensor_mask;
    uint8_t sample_rate_hz;
    uint8_t led_green;
    uint8_t led_ir;
    uint8_t led_red;
    uint8_t led_range;
    uint8_t tint;
    uint8_t adc_range;
    uint8_t sample_avg;
    uint8_t alc_enable;
    uint8_t proximity_enable;
} ppg_config_t;

/**
 * @brief Initialise PPG subsystem (MUX, GPIO pins, sensor hardware).
 * @return 0 on success, negative errno on failure.
 */
int ppg_new_init(void);

/**
 * @brief Start PPG streaming with the supplied runtime configuration.
 *
 * Stores @p cfg and signals the streaming thread to begin. The thread
 * will (optionally) run proximity detection before switching to PPG.
 *
 * @param cfg  Pointer to runtime configuration. Must not be NULL.
 * @return 0 on success, -EALREADY if already streaming, -EBUSY otherwise.
 */
int ppg_new_start_streaming(const ppg_config_t *cfg);

/**
 * @brief Stop PPG streaming and return to idle.
 * @return 0 on success, -EINVAL if not currently streaming.
 */
int ppg_new_stop_streaming(void);

ppg_new_state_t ppg_new_get_state(void);
bool ppg_new_is_streaming(void);

#endif /* PPG_NEW_APPL_H */
