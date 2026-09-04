/*
 * ----------------------------------------------------------------------
 *
 * File: lsm6dsv16bx_sensor.c
 *
 * Last edited: 08.07.2026
 *
 * Copyright (C) 2026, ETH Zurich and University of Bologna.
 *
 * Authors:
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
 * @file lsm6dsv16bx_sensor.c
 * @brief LSM6DSV16BX Low-Level Driver Implementation
 *
 * This module provides low-level driver functions for the LSM6DSV16BX
 * 6-axis IMU (accelerometer + gyroscope). For high-level streaming
 * control, use the imu_appl.h API instead.
 */

#include "sensors/imu/lsm6dsv16bx_sensor.h"

#include "core/i2c_helpers.h"
#include "sensors/imu/driver/lsm6dsv16bx_reg.h"

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

#define LSM6DSV16BX_INT_NODE DT_NODELABEL(gpio_imu_int1)

/*==============================================================================
 * Private Variables
 *============================================================================*/

/** @brief GPIO spec for LSM6DSV16BX INT1 pin */
static const struct gpio_dt_spec lsm6dsv16bx_int_gpio = GPIO_DT_SPEC_GET(LSM6DSV16BX_INT_NODE, gpios);

/** @brief GPIO callback data */
static struct gpio_callback lsm6dsv16bx_cb_data;

/** @brief Semaphore signaled on INT1 interrupt (data ready) */
K_SEM_DEFINE(lsm6dsv16bx_drdy_sem, 0, 1);

/** @brief LSM6DSV16BX driver context */
static stmdev_ctx_t lsm6dsv16bx_ctx;

/** @brief I2C context for the sensor */
static i2c_ctx_t i2c_ctx;

/** @brief I2C device handle */
static const struct device *const i2c_dev = DEVICE_DT_GET(DT_ALIAS(i2ca));

/** @brief Flag indicating if sensor has been initialized */
static bool sensor_initialized = false;

/*==============================================================================
 * Private Functions
 *============================================================================*/

/**
 * @brief GPIO interrupt callback for LSM6DSV16BX INT1 pin
 *
 * Called when the sensor signals data ready via INT1 pin.
 */
static void lsm6dsv16bx_irq_callback(const struct device *dev, struct gpio_callback *cb,
                                     uint32_t pins) {
  ARG_UNUSED(dev);
  ARG_UNUSED(cb);
  ARG_UNUSED(pins);

  k_sem_give(&lsm6dsv16bx_drdy_sem);
}

/**
 * @brief Millisecond delay hook for the ST register driver
 */
static void lsm6dsv16bx_mdelay(uint32_t millisec) {
  k_msleep(millisec);
}

/*==============================================================================
 * Public Functions - Initialization & Configuration
 *============================================================================*/

int lsm6dsv16bx_sensor_init(void) {
  int ret;
  int32_t error;

  if (sensor_initialized) {
    LOG_DBG("LSM6DSV16BX already initialized");
    return 0;
  }

  LOG_INF("Starting LSM6DSV16BX initialization...");

  /* Configure the interrupt pin */
  if (!device_is_ready(lsm6dsv16bx_int_gpio.port)) {
    LOG_ERR("GPIO device %s is not ready", lsm6dsv16bx_int_gpio.port->name);
    return -ENODEV;
  }

  LOG_INF("Configuring LSM6DSV16BX interrupt pin %d...", lsm6dsv16bx_int_gpio.pin);

  ret = gpio_pin_configure_dt(&lsm6dsv16bx_int_gpio, GPIO_INPUT);
  if (ret < 0) {
    LOG_ERR("Failed to configure GPIO pin %d (error %d)", lsm6dsv16bx_int_gpio.pin, ret);
    return ret;
  }

  ret = gpio_pin_interrupt_configure_dt(&lsm6dsv16bx_int_gpio, GPIO_INT_EDGE_TO_ACTIVE);
  if (ret < 0) {
    LOG_ERR("Failed to configure interrupt on GPIO pin %d (error %d)",
            lsm6dsv16bx_int_gpio.pin, ret);
    return ret;
  }

  gpio_init_callback(&lsm6dsv16bx_cb_data, lsm6dsv16bx_irq_callback,
                     BIT(lsm6dsv16bx_int_gpio.pin));
  gpio_add_callback(lsm6dsv16bx_int_gpio.port, &lsm6dsv16bx_cb_data);

  LOG_INF("Interrupt configured on %s pin %d",
          lsm6dsv16bx_int_gpio.port->name, lsm6dsv16bx_int_gpio.pin);

  /* Create an I2C context and sensor context */
  i2c_ctx.i2c_handle = i2c_dev;
  i2c_ctx.i2c_addr = LSM6DSV16BX_I2C_ADDR;

  lsm6dsv16bx_ctx.write_reg = i2c_write_reg;
  lsm6dsv16bx_ctx.read_reg = i2c_read_reg;
  lsm6dsv16bx_ctx.mdelay = lsm6dsv16bx_mdelay;
  lsm6dsv16bx_ctx.handle = &i2c_ctx;

  /* Read device ID to verify communication */
  uint8_t sensor_id;
  error = lsm6dsv16bx_device_id_get(&lsm6dsv16bx_ctx, &sensor_id);
  if (error != 0) {
    LOG_ERR("Error getting sensor ID: %d", error);
    return -EIO;
  }
  if (sensor_id != LSM6DSV16BX_ID) {
    LOG_ERR("Unexpected sensor ID: 0x%02X (expected 0x%02X)", sensor_id, LSM6DSV16BX_ID);
    return -ENODEV;
  }
  LOG_INF("LSM6DSV16BX ID: 0x%02X", sensor_id);

  /* Reset sensor to default configuration */
  LOG_INF("Resetting LSM6DSV16BX to default configuration...");
  error = lsm6dsv16bx_reset_set(&lsm6dsv16bx_ctx, LSM6DSV16BX_RESTORE_CTRL_REGS);
  if (error != 0) {
    LOG_ERR("Error during reset: %d", error);
    return -EIO;
  }

  /* Wait for reset to complete */
  lsm6dsv16bx_reset_t rst;
  do {
    lsm6dsv16bx_reset_get(&lsm6dsv16bx_ctx, &rst);
  } while (rst != LSM6DSV16BX_READY);

  LOG_INF("LSM6DSV16BX reset complete.");
  sensor_initialized = true;

  return 0;
}

int lsm6dsv16bx_start_sampling(void) {
  int32_t error;

  if (!sensor_initialized) {
    LOG_ERR("LSM6DSV16BX not initialized");
    return -ENODEV;
  }

  /* Enable block data update (no mixed-sample reads) */
  error = lsm6dsv16bx_block_data_update_set(&lsm6dsv16bx_ctx, PROPERTY_ENABLE);
  if (error != 0) {
    LOG_ERR("Error enabling block data update: %d", error);
    return -EIO;
  }

  /* Set full scales: +/-8 g accelerometer, +/-2000 dps gyroscope */
  error = lsm6dsv16bx_xl_full_scale_set(&lsm6dsv16bx_ctx, LSM6DSV16BX_8g);
  if (error != 0) {
    LOG_ERR("Error setting accelerometer full scale: %d", error);
    return -EIO;
  }

  error = lsm6dsv16bx_gy_full_scale_set(&lsm6dsv16bx_ctx, LSM6DSV16BX_2000dps);
  if (error != 0) {
    LOG_ERR("Error setting gyroscope full scale: %d", error);
    return -EIO;
  }

  /* Set DRDY mode to pulsed (interrupt mode) */
  error = lsm6dsv16bx_data_ready_mode_set(&lsm6dsv16bx_ctx, LSM6DSV16BX_DRDY_PULSED);
  if (error != 0) {
    LOG_ERR("Error setting DRDY mode: %d", error);
    return -EIO;
  }

  /* Route the accelerometer DRDY interrupt to INT1. The gyroscope runs at
   * the same ODR, so each accelerometer DRDY also delivers a fresh gyro
   * sample (both are read back-to-back with BDU enabled). */
  lsm6dsv16bx_pin_int_route_t int_route = {0};
  int_route.drdy_xl = 1;
  error = lsm6dsv16bx_pin_int1_route_set(&lsm6dsv16bx_ctx, int_route);
  if (error != 0) {
    LOG_ERR("Error routing interrupt to INT1: %d", error);
    return -EIO;
  }

  /* Clear any pending semaphore */
  k_sem_reset(&lsm6dsv16bx_drdy_sem);

  /* Set ODRs last: this powers on both sensors and starts sampling */
  error = lsm6dsv16bx_xl_data_rate_set(&lsm6dsv16bx_ctx, LSM6DSV16BX_XL_ODR_AT_480Hz);
  if (error != 0) {
    LOG_ERR("Error setting accelerometer ODR: %d", error);
    return -EIO;
  }

  error = lsm6dsv16bx_gy_data_rate_set(&lsm6dsv16bx_ctx, LSM6DSV16BX_GY_ODR_AT_480Hz);
  if (error != 0) {
    LOG_ERR("Error setting gyroscope ODR: %d", error);
    return -EIO;
  }

  LOG_INF("LSM6DSV16BX sampling started at 480 Hz (XL +/-8 g, GY +/-2000 dps)");
  return 0;
}

int lsm6dsv16bx_stop_sampling(void) {
  int32_t error;

  if (!sensor_initialized) {
    LOG_ERR("LSM6DSV16BX not initialized");
    return -ENODEV;
  }

  /* Put both sensors into power-down mode */
  error = lsm6dsv16bx_xl_data_rate_set(&lsm6dsv16bx_ctx, LSM6DSV16BX_XL_ODR_OFF);
  if (error != 0) {
    LOG_ERR("Error powering down accelerometer: %d", error);
    return -EIO;
  }

  error = lsm6dsv16bx_gy_data_rate_set(&lsm6dsv16bx_ctx, LSM6DSV16BX_GY_ODR_OFF);
  if (error != 0) {
    LOG_ERR("Error powering down gyroscope: %d", error);
    return -EIO;
  }

  LOG_INF("LSM6DSV16BX sampling stopped");
  return 0;
}

/*==============================================================================
 * Public Functions - Data Reading
 *============================================================================*/

int lsm6dsv16bx_wait_data_ready(uint32_t timeout_ms) {
  int ret;

  if (!sensor_initialized) {
    return -ENODEV;
  }

  if (timeout_ms == 0) {
    ret = k_sem_take(&lsm6dsv16bx_drdy_sem, K_NO_WAIT);
  } else {
    ret = k_sem_take(&lsm6dsv16bx_drdy_sem, K_MSEC(timeout_ms));
  }

  if (ret == -EBUSY || ret == -EAGAIN) {
    return -EAGAIN; /* Timeout or not ready */
  }

  return ret;
}

int lsm6dsv16bx_read_accel_gyro(int16_t accel[3], int16_t gyro[3]) {
  int32_t error;

  if (!sensor_initialized) {
    return -ENODEV;
  }

  if (accel == NULL || gyro == NULL) {
    return -EINVAL;
  }

  error = lsm6dsv16bx_acceleration_raw_get(&lsm6dsv16bx_ctx, accel);
  if (error != 0) {
    LOG_ERR("Error getting accelerometer data: %d", error);
    return -EIO;
  }

  error = lsm6dsv16bx_angular_rate_raw_get(&lsm6dsv16bx_ctx, gyro);
  if (error != 0) {
    LOG_ERR("Error getting gyroscope data: %d", error);
    return -EIO;
  }

  return 0;
}

int lsm6dsv16bx_read_temperature(float *temp_celsius) {
  int32_t error;
  int16_t raw_temp;

  if (!sensor_initialized) {
    return -ENODEV;
  }

  if (temp_celsius == NULL) {
    return -EINVAL;
  }

  error = lsm6dsv16bx_temperature_raw_get(&lsm6dsv16bx_ctx, &raw_temp);
  if (error != 0) {
    LOG_ERR("Error getting temperature data: %d", error);
    return -EIO;
  }

  *temp_celsius = lsm6dsv16bx_from_lsb_to_celsius(raw_temp);
  return 0;
}
