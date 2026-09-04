/*
 * TCA9548A 8-channel I2C multiplexer driver (Zephyr/nRF5340).
 *
 * All channel selection boils down to writing one byte to the MUX
 * control register.  The MUX address and I2C device are set once at
 * init time and stored in static variables.
 */

#include "bsp/i2c_mux/i2c_mux.h"

#include <zephyr/drivers/i2c.h>
#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(i2c_mux, LOG_LEVEL_INF);

static const struct device *s_i2c_dev  = NULL;
static uint8_t              s_mux_addr = I2C_MUX_ADDR_DEFAULT;

int i2c_mux_init(const struct device *i2c_dev, uint8_t mux_addr)
{
    if (!device_is_ready(i2c_dev)) {
        LOG_ERR("I2C device not ready");
        return -ENODEV;
    }
    s_i2c_dev  = i2c_dev;
    s_mux_addr = mux_addr;
    LOG_INF("TCA9548A at 0x%02X on %s", mux_addr, i2c_dev->name);
    return i2c_mux_deselect_all();
}

int i2c_mux_select_channel(uint8_t channel)
{
    if (channel >= I2C_MUX_MAX_CHANNELS) {
        return -EINVAL;
    }
    uint8_t ctrl = (uint8_t)(1U << channel);
    int ret = i2c_write(s_i2c_dev, &ctrl, 1, s_mux_addr);
    if (ret) {
        LOG_ERR("MUX select ch%u failed: %d", channel, ret);
    }
    return ret;
}

int i2c_mux_deselect_all(void)
{
    uint8_t ctrl = 0x00;
    int ret = i2c_write(s_i2c_dev, &ctrl, 1, s_mux_addr);
    if (ret) {
        LOG_ERR("MUX deselect failed: %d", ret);
    }
    return ret;
}

int i2c_mux_broadcast(uint8_t channel_mask)
{
    int ret = i2c_write(s_i2c_dev, &channel_mask, 1, s_mux_addr);
    if (ret) {
        LOG_ERR("MUX broadcast mask=0x%02X failed: %d", channel_mask, ret);
    }
    return ret;
}
