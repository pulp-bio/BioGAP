/*
 * Platform-specific I2C layer for the MAXM86161 driver (Zephyr/nRF5340).
 *
 * Wraps Zephyr I2C API; device/address set at runtime so the application
 * layer can switch between sensors by changing the MUX channel then calling
 * maxm86161_i2c_set_device() before touching the sensor driver.
 */

#include "maxm86161_i2c.h"

#include <zephyr/drivers/i2c.h>
#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(maxm86161_i2c, LOG_LEVEL_WRN);

static const struct device *s_i2c_dev = NULL;
static uint8_t              s_addr    = MAXM86161_I2C_ADDR;

void maxm86161_i2c_set_device(const struct device *i2c_dev, uint8_t addr)
{
    s_i2c_dev = i2c_dev;
    s_addr    = addr;
}

void maxm86161_i2c_write_to_register(uint8_t reg, uint8_t value)
{
    if (!s_i2c_dev) {
        return;
    }
    uint8_t buf[2] = { reg, value };
    int ret = i2c_write(s_i2c_dev, buf, sizeof(buf), s_addr);
    if (ret) {
        LOG_WRN("I2C write reg 0x%02X failed: %d", reg, ret);
    }
}

void maxm86161_i2c_read_from_register(uint8_t reg, uint8_t *value)
{
    if (!s_i2c_dev || !value) {
        return;
    }
    int ret = i2c_write_read(s_i2c_dev, s_addr, &reg, 1, value, 1);
    if (ret) {
        LOG_WRN("I2C read reg 0x%02X failed: %d", reg, ret);
    }
}

void maxm86161_i2c_block_read(uint8_t reg, uint16_t len, uint8_t *buf)
{
    if (!s_i2c_dev || !buf || len == 0) {
        return;
    }
    int ret = i2c_write_read(s_i2c_dev, s_addr, &reg, 1, buf, len);
    if (ret) {
        LOG_WRN("I2C block read reg 0x%02X len %u failed: %d", reg, len, ret);
    }
}
