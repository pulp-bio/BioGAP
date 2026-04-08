/*
 * ----------------------------------------------------------------------
 *
 * File: lis2duxs12_sensor.c
 *
 * Last edited: 08.12.2025
 *
 * Copyright (C) 2025, ETH Zurich and University of Bologna.
 *
 * Authors:
 * - Philip Wiese (wiesep@iis.ee.ethz.ch), ETH Zurich
 * - Sebastian Frey (sefrey@iis.ee.ethz.ch), ETH Zurich
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
 * @file lis2duxs12_sensor.c
 * @brief LIS2DUXS12 Low-Level Driver Implementation
 *
 * This module provides low-level driver functions for the LIS2DUXS12
 * 3-axis accelerometer. For high-level streaming control, use the
 * imu_appl.h API instead.
 */

#include "sensors/imu/lis2duxs12_sensor.h"

#include "core/i2c_helpers.h"
#include "sensors/imu/driver/lis2duxs12_reg.h"

#include <errno.h>
#include <zephyr/device.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/kernel.h>

#include <zephyr/logging/log.h>

#include "core/common.h"

LOG_MODULE_DECLARE(sensors, LOG_LEVEL_INF);

/*==============================================================================
 * Private Definitions
 *============================================================================*/

#define LIS2DUXS12_INT_NODE DT_NODELABEL(gpio_lis2duxs12_int1)

/*==============================================================================
 * Private Variables
 *============================================================================*/

/** @brief GPIO spec for LIS2DUXS12 INT1 pin */
static const struct gpio_dt_spec lis2duxs12_int_gpio = GPIO_DT_SPEC_GET(LIS2DUXS12_INT_NODE, gpios);

/** @brief GPIO callback data */
static struct gpio_callback lis2duxs12_cb_data;

/** @brief Semaphore signaled on INT1 interrupt (data ready) */
K_SEM_DEFINE(lis2duxs12_drdy_sem, 0, 1);

/** @brief LIS2DUXS12 driver context */
static stmdev_ctx_t lis2duxs12_ctx;

/** @brief I2C context for the sensor */
static i2c_ctx_t i2c_ctx;

/** @brief Sensor mode configuration */
static lis2duxs12_md_t sensor_mode;

/** @brief I2C device handle */
static const struct device *const i2c_dev = DEVICE_DT_GET(DT_ALIAS(i2ca));

/** @brief Flag indicating if sensor has been initialized */
static bool sensor_initialized = false;

/*==============================================================================
 * Private Functions
 *============================================================================*/

/**
 * @brief GPIO interrupt callback for LIS2DUXS12 INT1 pin
 *
 * Called when the sensor signals data ready via INT1 pin.
 */
static void lis2duxs12_irq_callback(const struct device *dev, struct gpio_callback *cb, 
                                     uint32_t pins) {
  ARG_UNUSED(dev);
  ARG_UNUSED(cb);
  ARG_UNUSED(pins);
  
  k_sem_give(&lis2duxs12_drdy_sem);
}

/*==============================================================================
 * Public Functions - Initialization & Configuration
 *============================================================================*/

int lis2duxs12_init(void) {
  int ret;
  int16_t error;

  if (sensor_initialized) {
    LOG_DBG("LIS2DUXS12 already initialized");
    return 0;
  }

  LOG_INF("Starting LIS2DUXS12 initialization...");

  /* Configure the interrupt pin */
  if (!device_is_ready(lis2duxs12_int_gpio.port)) {
    LOG_ERR("GPIO device %s is not ready", lis2duxs12_int_gpio.port->name);
    return -ENODEV;
  }

  LOG_INF("Configuring LIS2DUXS12 interrupt pin %d...", lis2duxs12_int_gpio.pin);

  ret = gpio_pin_configure_dt(&lis2duxs12_int_gpio, GPIO_INPUT);
  if (ret < 0) {
    LOG_ERR("Failed to configure GPIO pin %d (error %d)", lis2duxs12_int_gpio.pin, ret);
    return ret;
  }

  ret = gpio_pin_interrupt_configure_dt(&lis2duxs12_int_gpio, GPIO_INT_EDGE_TO_ACTIVE);
  if (ret < 0) {
    LOG_ERR("Failed to configure interrupt on GPIO pin %d (error %d)", 
            lis2duxs12_int_gpio.pin, ret);
    return ret;
  }

  gpio_init_callback(&lis2duxs12_cb_data, lis2duxs12_irq_callback, 
                     BIT(lis2duxs12_int_gpio.pin));
  gpio_add_callback(lis2duxs12_int_gpio.port, &lis2duxs12_cb_data);

  LOG_INF("Interrupt configured on %s pin %d", 
          lis2duxs12_int_gpio.port->name, lis2duxs12_int_gpio.pin);

  /* Create an I2C context and sensor context */
  i2c_ctx.i2c_handle = i2c_dev;
  i2c_ctx.i2c_addr = 0x19;  /* I2C address for LIS2DUXS12 */

  lis2duxs12_ctx.write_reg = i2c_write_reg;
  lis2duxs12_ctx.read_reg = i2c_read_reg;
  lis2duxs12_ctx.handle = &i2c_ctx;

  /* Exit deep power down */
  error = lis2duxs12_exit_deep_power_down(&lis2duxs12_ctx);
  if (error != 0) {
    LOG_ERR("Error exiting deep power down: %d", error);
    return -EIO;
  }

  LOG_INF("LIS2DUXS12 exited deep power down");

  /* Read device ID to verify communication */
  uint8_t sensor_id;
  error = lis2duxs12_device_id_get(&lis2duxs12_ctx, &sensor_id);
  if (error != 0) {
    LOG_ERR("Error getting sensor ID: %d", error);
    return -EIO;
  }
  LOG_INF("LIS2DUXS12 ID: 0x%02X", sensor_id);

  /* Reset sensor to default configuration */
  LOG_INF("Resetting LIS2DUXS12 to default configuration...");
  error = lis2duxs12_init_set(&lis2duxs12_ctx, LIS2DUXS12_RESET);
  if (error != 0) {
    LOG_ERR("Error during reset: %d", error);
    return -EIO;
  }

  /* Wait for reset to complete */
  lis2duxs12_status_t status;
  do {
    lis2duxs12_status_get(&lis2duxs12_ctx, &status);
  } while (status.sw_reset);

  LOG_INF("LIS2DUXS12 reset complete.");
  sensor_initialized = true;

  return 0;
}

int lis2duxs12_start_sampling(void) {
  int16_t error;

  if (!sensor_initialized) {
    LOG_ERR("LIS2DUXS12 not initialized");
    return -ENODEV;
  }

  /* Set sensor-only mode (disable embedded functions) */
  error = lis2duxs12_init_set(&lis2duxs12_ctx, LIS2DUXS12_SENSOR_ONLY_ON);
  if (error != 0) {
    LOG_ERR("Error during sensor init: %d", error);
    return -EIO;
  }

  /* Set DRDY mode to pulsed (interrupt mode) */
  error = lis2duxs12_data_ready_mode_set(&lis2duxs12_ctx, LIS2DUXS12_DRDY_PULSED);
  if (error != 0) {
    LOG_ERR("Error setting DRDY mode: %d", error);
    return -EIO;
  }

  /* Set sensor operating mode: full-scale, bandwidth, and ODR */
  sensor_mode.fs = LIS2DUXS12_8g;
  sensor_mode.bw = LIS2DUXS12_ODR_div_16;
  sensor_mode.odr = LIS2DUXS12_400Hz_LP;
  error = lis2duxs12_mode_set(&lis2duxs12_ctx, &sensor_mode);
  if (error != 0) {
    LOG_ERR("Error setting sensor mode: %d", error);
    return -EIO;
  }

  /* Configure interrupt behavior (using latched interrupt mode) */
  lis2duxs12_int_config_t int_cfg = {0};
  int_cfg.int_cfg = LIS2DUXS12_INT_LATCHED;
  int_cfg.dis_rst_lir_all_int = 0;
  int_cfg.sleep_status_on_int = 0;
  error = lis2duxs12_int_config_set(&lis2duxs12_ctx, &int_cfg);
  if (error != 0) {
    LOG_ERR("Error configuring interrupt mode: %d", error);
    return -EIO;
  }

  /* Route DRDY interrupt to INT1 pin */
  lis2duxs12_pin_int_route_t int_route = {0};
  int_route.int_on_res = 1;
  int_route.drdy = 1;
  error = lis2duxs12_pin_int1_route_set(&lis2duxs12_ctx, &int_route);
  if (error != 0) {
    LOG_ERR("Error routing interrupt to INT1: %d", error);
    return -EIO;
  }

  /* Clear any pending semaphore */
  k_sem_reset(&lis2duxs12_drdy_sem);

  LOG_INF("LIS2DUXS12 sampling started at 400 Hz");
  return 0;
}

int lis2duxs12_stop_sampling(void) {
  int16_t error;

  if (!sensor_initialized) {
    LOG_ERR("LIS2DUXS12 not initialized");
    return -ENODEV;
  }

  /* Set ODR to power-down mode */
  sensor_mode.odr = LIS2DUXS12_OFF;
  error = lis2duxs12_mode_set(&lis2duxs12_ctx, &sensor_mode);
  if (error != 0) {
    LOG_ERR("Error setting power-down mode: %d", error);
    return -EIO;
  }

  LOG_INF("LIS2DUXS12 sampling stopped");
  return 0;
}

/*==============================================================================
 * Public Functions - Data Reading
 *============================================================================*/

int lis2duxs12_wait_data_ready(uint32_t timeout_ms) {
  int ret;

  if (!sensor_initialized) {
    return -ENODEV;
  }

  if (timeout_ms == 0) {
    ret = k_sem_take(&lis2duxs12_drdy_sem, K_NO_WAIT);
  } else {
    ret = k_sem_take(&lis2duxs12_drdy_sem, K_MSEC(timeout_ms));
  }

  if (ret == -EBUSY || ret == -EAGAIN) {
    return -EAGAIN;  /* Timeout or not ready */
  }

  return ret;
}

int lis2duxs12_read_accel(int16_t *x, int16_t *y, int16_t *z) {
  int16_t error;
  lis2duxs12_xl_data_t data_xl;

  if (!sensor_initialized) {
    return -ENODEV;
  }

  if (x == NULL || y == NULL || z == NULL) {
    return -EINVAL;
  }

  error = lis2duxs12_xl_data_get(&lis2duxs12_ctx, &sensor_mode, &data_xl);
  if (error != 0) {
    LOG_ERR("Error getting accelerometer data: %d", error);
    return -EIO;
  }

  *x = data_xl.raw[0];
  *y = data_xl.raw[1];
  *z = data_xl.raw[2];

  return 0;
}

int lis2duxs12_read_temperature(float *temp_celsius) {
  int16_t error;
  lis2duxs12_outt_data_t data_temp;

  if (!sensor_initialized) {
    return -ENODEV;
  }

  if (temp_celsius == NULL) {
    return -EINVAL;
  }

  error = lis2duxs12_outt_data_get(&lis2duxs12_ctx, &sensor_mode, &data_temp);
  if (error != 0) {
    LOG_ERR("Error getting temperature data: %d", error);
    return -EIO;
  }

  *temp_celsius = data_temp.heat.deg_c;
  return 0;
}

/*==============================================================================
 * Public Functions - Tap Detection
 *============================================================================*/

int lis2duxs12_enable_double_tap(void) {
  int16_t error;
  uint8_t value;

  if (!sensor_initialized) {
    return -ENODEV;
  }

  /* Step 1: Switch to power-down mode */
  value = 0x00;
  error = lis2duxs12_write_reg(&lis2duxs12_ctx, LIS2DUXS12_CTRL5, &value, 1);
  if (error != 0) {
    return -EIO;
  }

  /* Step 2: TAP_CFG0 = 0xC8 */
  value = 0xC8;
  error = lis2duxs12_write_reg(&lis2duxs12_ctx, LIS2DUXS12_TAP_CFG0, &value, 1);
  if (error != 0) {
    return -EIO;
  }

  /* Step 3: TAP_CFG1 = 0x28 */
  value = 0x28;
  error = lis2duxs12_write_reg(&lis2duxs12_ctx, LIS2DUXS12_TAP_CFG1, &value, 1);
  if (error != 0) {
    return -EIO;
  }

  /* Step 4: TAP_CFG2 = 0x03 */
  value = 0x03;
  error = lis2duxs12_write_reg(&lis2duxs12_ctx, LIS2DUXS12_TAP_CFG2, &value, 1);
  if (error != 0) {
    return -EIO;
  }

  /* Step 5: TAP_CFG3 = 0x84 */
  value = 0x84;
  error = lis2duxs12_write_reg(&lis2duxs12_ctx, LIS2DUXS12_TAP_CFG3, &value, 1);
  if (error != 0) {
    return -EIO;
  }

  /* Step 6: TAP_CFG4 = 0x88 */
  value = 0x88;
  error = lis2duxs12_write_reg(&lis2duxs12_ctx, LIS2DUXS12_TAP_CFG4, &value, 1);
  if (error != 0) {
    return -EIO;
  }

  /* Step 7: TAP_CFG5 = 0xE0 - Enable single-, double-, and triple-tap events */
  value = 0xE0;
  error = lis2duxs12_write_reg(&lis2duxs12_ctx, LIS2DUXS12_TAP_CFG5, &value, 1);
  if (error != 0) {
    return -EIO;
  }

  /* Step 8: TAP_CFG6 = 0x0A */
  value = 0x0A;
  error = lis2duxs12_write_reg(&lis2duxs12_ctx, LIS2DUXS12_TAP_CFG6, &value, 1);
  if (error != 0) {
    return -EIO;
  }

  /* Step 9: MD1_CFG = 0x08 - Route TAP_IA signal to INT1 pin */
  value = 0x08;
  error = lis2duxs12_write_reg(&lis2duxs12_ctx, LIS2DUXS12_MD1_CFG, &value, 1);
  if (error != 0) {
    return -EIO;
  }

  /* Step 10: INTERRUPT_CFG = 0x01 - Enable basic interrupts */
  value = 0x01;
  error = lis2duxs12_write_reg(&lis2duxs12_ctx, LIS2DUXS12_INTERRUPT_CFG, &value, 1);
  if (error != 0) {
    return -EIO;
  }

  /* Step 11: CTRL5 = 0xA2 - ODR = 400 Hz, full scale = +/-8g */
  value = 0xA2;
  error = lis2duxs12_write_reg(&lis2duxs12_ctx, LIS2DUXS12_CTRL5, &value, 1);
  if (error != 0) {
    return -EIO;
  }

  LOG_INF("Double-tap detection enabled");
  return 0;
}

int lis2duxs12_disable_tap(void) {
  int16_t error;
  uint8_t value;

  if (!sensor_initialized) {
    return -ENODEV;
  }

  /* Disable tap detection in TAP_CFG5 */
  value = 0x00;
  error = lis2duxs12_write_reg(&lis2duxs12_ctx, LIS2DUXS12_TAP_CFG5, &value, 1);
  if (error != 0) {
    return -EIO;
  }

  /* Clear tap interrupt routing in MD1_CFG */
  value = 0x00;
  error = lis2duxs12_write_reg(&lis2duxs12_ctx, LIS2DUXS12_MD1_CFG, &value, 1);
  if (error != 0) {
    return -EIO;
  }

  LOG_INF("Tap detection disabled");
  return 0;
}
