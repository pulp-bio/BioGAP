/*
 * TCA9548A 8-channel I2C multiplexer driver.
 *
 * The TCA9548A is controlled by a single-byte register:
 *   bit N = 1  →  channel N is enabled
 *   bit N = 0  →  channel N is disabled
 * Multiple bits can be set simultaneously (broadcast mode): all enabled
 * downstream devices receive subsequent I2C traffic at once.
 *
 * Default I2C address: 0x70 (A2=A1=A0=0).
 */

#ifndef I2C_MUX_H
#define I2C_MUX_H

#include <stdint.h>
#include <zephyr/device.h>

#define I2C_MUX_ADDR_DEFAULT  0x70U
#define I2C_MUX_MAX_CHANNELS  8U

/**
 * @brief Initialise the MUX driver and deselect all channels.
 *
 * @param i2c_dev  Ready Zephyr I2C device for the bus the MUX is on.
 * @param mux_addr 7-bit I2C address of the TCA9548A (default 0x70).
 * @return 0 on success, negative errno on failure.
 */
int i2c_mux_init(const struct device *i2c_dev, uint8_t mux_addr);

/**
 * @brief Enable exactly one channel, disabling all others.
 *
 * @param channel Channel index 0–7.
 * @return 0 on success, negative errno on failure.
 */
int i2c_mux_select_channel(uint8_t channel);

/**
 * @brief Deselect all channels (write 0x00 to control register).
 *
 * @return 0 on success, negative errno on failure.
 */
int i2c_mux_deselect_all(void);

/**
 * @brief Enable all channels indicated by channel_mask simultaneously.
 *
 * Any I2C transaction issued after this call is forwarded to every
 * enabled channel at once ("broadcast mode"). Useful for configuring
 * multiple identical sensors with a single write sequence.
 *
 * @param channel_mask Bitmask of channels to enable (bit N = channel N).
 * @return 0 on success, negative errno on failure.
 */
int i2c_mux_broadcast(uint8_t channel_mask);

#endif /* I2C_MUX_H */
