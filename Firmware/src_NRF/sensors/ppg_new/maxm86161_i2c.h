/*
 * Platform-specific I2C layer for the MAXM86161 driver on Zephyr/nRF.
 *
 * The driver (maxm86161.c) calls these global functions without a context
 * parameter. Before calling any driver function, select the active I2C
 * bus and device address via maxm86161_i2c_set_device().  The PPG
 * application layer calls this every time it switches MUX channels.
 */

#ifndef MAXM86161_I2C_H
#define MAXM86161_I2C_H

#include <stdint.h>
#include <zephyr/device.h>

/** 7-bit I2C address of MAXM86161 (write byte = 0xC4, read byte = 0xC5) */
#define MAXM86161_I2C_ADDR  0x62U

/**
 * @brief Set the I2C device and address used by all subsequent driver calls.
 *
 * Must be called before maxm86161_i2c_write_to_register /
 * maxm86161_i2c_read_from_register / maxm86161_i2c_block_read.
 *
 * @param i2c_dev  Ready Zephyr I2C device (e.g. DEVICE_DT_GET(DT_ALIAS(i2cb)))
 * @param addr     7-bit I2C address of the active sensor
 */
void maxm86161_i2c_set_device(const struct device *i2c_dev, uint8_t addr);

/** Write one byte to a MAXM86161 register. */
void maxm86161_i2c_write_to_register(uint8_t reg, uint8_t value);

/** Read one byte from a MAXM86161 register. */
void maxm86161_i2c_read_from_register(uint8_t reg, uint8_t *value);

/** Burst-read len bytes starting at reg into buf. */
void maxm86161_i2c_block_read(uint8_t reg, uint16_t len, uint8_t *buf);

#endif /* MAXM86161_I2C_H */
