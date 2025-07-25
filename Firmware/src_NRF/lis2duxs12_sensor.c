/*
 * ----------------------------------------------------------------------
 *
 * File: lis2duxs12_sensor.c
 *
 * Last edited: 24.03.2025
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



#include "lis2duxs12_sensor.h"

#include "i2c_helpers.h"

#include <zephyr/device.h>
#include <zephyr/drivers/gpio.h>

#include <zephyr/logging/log.h>
#include <zephyr/logging/log_ctrl.h>

#include "common.h"

LOG_MODULE_DECLARE(sensors, LOG_LEVEL_INF);

/* Define stack sizes and priorities */
#define IMU_STACK_SIZE    1024
#define IMU_PRIORITY      6

#define LIS2DUXS12_INT_NODE DT_NODELABEL(gpio_lis2duxs12_int1)
static const struct gpio_dt_spec lis2duxs12_int_gpio = GPIO_DT_SPEC_GET(LIS2DUXS12_INT_NODE, gpios);

static struct gpio_callback lis2duxs12_cb_data;
K_SEM_DEFINE(imu_int1, 0, 1);

lis2duxs12_xl_data_t data_xl;
lis2duxs12_outt_data_t data_temp;
lis2duxs12_md_t md;
stmdev_ctx_t lis2duxs12_ctx;
i2c_ctx_t i2c_ctx;

static const struct device *const i2c_a = DEVICE_DT_GET(DT_ALIAS(i2ca));


void lis2duxs12_irq_callback(const struct device *dev,
  struct gpio_callback *cb,
  uint32_t pins)
{
// Give the semaphore to the PPG thread
k_sem_give(&imu_int1);
}

void imu_receive_thread_temp_tap(void *arg1, void *arg2, void *arg3)
{
  k_sleep(K_MSEC(2000));
  //init_lis2duxs12();
  while (1) {
    // busy loop for loop
    k_sem_take(&imu_int1, K_FOREVER);
    for(int i = 0; i < 1000000; i++) {
      // Do nothing, just busy wait
    }
  }
}
void imu_receive_thread(void *arg1, void *arg2, void *arg3)
{
  k_sleep(K_MSEC(2000));
  //init_lis2duxs12();
  enable_acc_sampling();
  while (1) {
    k_sem_take(&imu_int1, K_FOREVER);

    int16_t error = NO_ERROR;
    error = lis2duxs12_xl_data_get(&lis2duxs12_ctx, &md, &data_xl);
    if (error != NO_ERROR) {
      LOG_ERR(" * Error %d getting data", error);
    } else {
      LOG_INF(" - Acceleration X                      : % 7.2f mg" SPACES, data_xl.mg[0]);
      LOG_INF(" - Acceleration Y                      : % 7.2f mg" SPACES, data_xl.mg[1]);
      LOG_INF(" - Acceleration Z                      : % 7.2f mg" SPACES, data_xl.mg[2]);
    }
  

    error = lis2duxs12_outt_data_get(&lis2duxs12_ctx, &md, &data_temp);
    if (error != NO_ERROR) {
      LOG_ERR(" * Error %d getting temperature", error);
    } else {
      LOG_INF(" - Temperature                         : %3.2f °C" SPACES, data_temp.heat.deg_c);
    }



    LOG_INF("LIS2DUXS12 interrupt triggered");
  }
}

void init_lis2duxs12(void)
{

  // Configure the interrupt pin
  if (!device_is_ready(lis2duxs12_int_gpio.port)) {
    LOG_ERR("GPIO device %s is not ready", lis2duxs12_int_gpio.port->name);
    return false;
  }

  int ret = gpio_pin_configure_dt(&lis2duxs12_int_gpio, GPIO_INPUT);
  if (ret < 0) {
      LOG_ERR("Failed to configure GPIO pin %d (error %d)", lis2duxs12_int_gpio.pin, ret);
      return false;
  }

  ret = gpio_pin_interrupt_configure_dt(&lis2duxs12_int_gpio, GPIO_INT_EDGE_TO_ACTIVE);
  if (ret < 0) {
      LOG_ERR("Failed to configure interrupt on GPIO pin %d (error %d)", lis2duxs12_int_gpio.pin, ret);
      return false;
  }

  gpio_init_callback(&lis2duxs12_cb_data, lis2duxs12_irq_callback, BIT(lis2duxs12_int_gpio.pin));
  gpio_add_callback(lis2duxs12_int_gpio.port, &lis2duxs12_cb_data);
  LOG_INF("Interrupt configured on %s pin %d", lis2duxs12_int_gpio.port->name, lis2duxs12_int_gpio.pin);




    int16_t error = NO_ERROR;
    /* Create an I2C context and sensor context */
    i2c_ctx.i2c_handle = i2c_a;  // Provided device pointer from DTS
    i2c_ctx.i2c_addr = 0x19;     // I2C address for LIS2DUXS12

    
    lis2duxs12_ctx.write_reg = i2c_write_reg;
    lis2duxs12_ctx.read_reg  = i2c_read_reg;
    lis2duxs12_ctx.handle    = &i2c_ctx;

    /* Exit deep power down */
    error = lis2duxs12_exit_deep_power_down(&lis2duxs12_ctx);
    if (error != NO_ERROR) {
        LOG_ERR("Error exiting deep power down: %d", error);
    }

    /* Read device ID */
    uint8_t sensor_id;
    error = lis2duxs12_device_id_get(&lis2duxs12_ctx, &sensor_id);
    if (error != NO_ERROR) {
        LOG_ERR("Error getting sensor ID: %d", error);
    } else {
        LOG_INF("LIS2DUXS12 ID: 0x%02X", sensor_id);
    }

    /* Reset sensor */
    error = lis2duxs12_init_set(&lis2duxs12_ctx, LIS2DUXS12_RESET);
    if (error != NO_ERROR) {
        LOG_ERR("Error during reset: %d", error);
        return;
    }
    /* Wait for reset to complete */
    lis2duxs12_status_t status;
    do {
        lis2duxs12_status_get(&lis2duxs12_ctx, &status);
    } while (status.sw_reset);
}


void enable_acc_sampling(){
  int16_t error = NO_ERROR;
  /* Set sensor-only mode (disable embedded functions) */
  error = lis2duxs12_init_set(&lis2duxs12_ctx, LIS2DUXS12_SENSOR_ONLY_ON);
  if (error != NO_ERROR) {
      LOG_ERR("Error during sensor init: %d", error);
  }

  /* Set DRDY mode to pulsed (interrupt mode) */
  error = lis2duxs12_data_ready_mode_set(&lis2duxs12_ctx, LIS2DUXS12_DRDY_PULSED);
  if (error != NO_ERROR) {
      LOG_ERR("Error setting DRDY mode: %d", error);
  }

  /* Set sensor operating mode: full–scale, bandwidth, and ODR */
  md.fs  = LIS2DUXS12_8g;
  md.bw  = LIS2DUXS12_ODR_div_16;
  md.odr = LIS2DUXS12_400Hz_LP;// LIS2DUXS12_400Hz_LP;  // Example ODR; adjust as needed
  error = lis2duxs12_mode_set(&lis2duxs12_ctx, &md);
  if (error != NO_ERROR) {
      LOG_ERR("Error setting sensor mode: %d", error);
  }

  /* Configure interrupt behavior (using latched interrupt mode) */
  lis2duxs12_int_config_t int_cfg;
  int_cfg.int_cfg = LIS2DUXS12_INT_LATCHED;
  int_cfg.dis_rst_lir_all_int = 0;
  int_cfg.sleep_status_on_int = 0;
  error = lis2duxs12_int_config_set(&lis2duxs12_ctx, &int_cfg);
  if (error != NO_ERROR) {
      LOG_ERR("Error configuring interrupt mode: %d", error);
  }

  /* Route DRDY interrupt to RES (INT1) pin */
  lis2duxs12_pin_int_route_t int_route = {0};
  int_route.int_on_res = 1;
  int_route.drdy       = 1;
  int_route.fifo_ovr   = 0;
  int_route.fifo_th    = 0;
  int_route.fifo_full  = 0;
  int_route.boot       = 0;
  int_route.free_fall  = 0;
  int_route.six_d      = 0;
  int_route.tap        = 0;
  int_route.wake_up    = 0;
  int_route.sleep_change = 0;
  int_route.emb_function = 0;
  int_route.timestamp  = 0;
  error = lis2duxs12_pin_int1_route_set(&lis2duxs12_ctx, &int_route);
  if (error != NO_ERROR) {
      LOG_ERR("Error routing interrupt to INT1: %d", error);
  }

}


void test_lis2duxs12() {
  LOG_INF("Testing LIS2DUXS12 (Accelerometer)" SPACES);

  int16_t error = NO_ERROR;

  i2c_ctx_t i2c_ctx;
  i2c_ctx.i2c_handle = i2c_a;
  i2c_ctx.i2c_addr = 0x19;

  lis2duxs12_ctx.write_reg = i2c_write_reg;
  lis2duxs12_ctx.read_reg = i2c_read_reg;
  lis2duxs12_ctx.handle = &i2c_ctx;

  error = lis2duxs12_exit_deep_power_down(&lis2duxs12_ctx);
  if (error != NO_ERROR) {
    LOG_ERR(" * Error %d exiting deep power down", error);
  }

  uint8_t lis2duxs12_id;
  error = lis2duxs12_device_id_get(&lis2duxs12_ctx, &lis2duxs12_id);
  if (error != NO_ERROR) {
    LOG_ERR(" * Error %d getting ID", error);
  } else {
    LOG_INF(" - ID                                  : 0x%02X" SPACES, lis2duxs12_id);
  }

  /* Restore default configuration */
  error = lis2duxs12_init_set(&lis2duxs12_ctx, LIS2DUXS12_RESET);
  if (error != NO_ERROR) {
    LOG_ERR(" * Error %d during reset", error);
    return;
  }

  lis2duxs12_status_t status;
  do {
    lis2duxs12_status_get(&lis2duxs12_ctx, &status);
  } while (status.sw_reset);

  /* Set bdu and if_inc recommended for driver usage */
  error = lis2duxs12_init_set(&lis2duxs12_ctx, LIS2DUXS12_SENSOR_ONLY_ON);
  if (error != NO_ERROR) {
    LOG_ERR(" * Error %d during init", error);
  }

  /* Set Output Data Rate */
  md.fs = LIS2DUXS12_2g;
  md.bw = LIS2DUXS12_ODR_div_16;
  md.odr = LIS2DUXS12_1Hz6_ULP;
  error = lis2duxs12_mode_set(&lis2duxs12_ctx, &md);
  if (error != NO_ERROR) {
    LOG_ERR(" * Error %d setting mode", error);
  }

  error = lis2duxs12_xl_data_get(&lis2duxs12_ctx, &md, &data_xl);
  if (error != NO_ERROR) {
    LOG_ERR(" * Error %d getting data", error);
  } else {
    LOG_INF(" - Acceleration X                      : % 7.2f mg" SPACES, data_xl.mg[0]);
    LOG_INF(" - Acceleration Y                      : % 7.2f mg" SPACES, data_xl.mg[1]);
    LOG_INF(" - Acceleration Z                      : % 7.2f mg" SPACES, data_xl.mg[2]);
  }

  error = lis2duxs12_outt_data_get(&lis2duxs12_ctx, &md, &data_temp);
  if (error != NO_ERROR) {
    LOG_ERR(" * Error %d getting temperature", error);
  } else {
    LOG_INF(" - Temperature                         : %3.2f °C" SPACES, data_temp.heat.deg_c);
  }
}
int lis2duxs12_enable_double_tap(void)
{
    int16_t error;
    uint8_t value;

    // Step 1: Switch to power-down mode (CTRL5 = 0x00)
    value = 0x00;
    error = lis2duxs12_write_reg(&lis2duxs12_ctx, LIS2DUXS12_CTRL5, &value, 1);
    if (error != 0) return error;

    // Step 2: TAP_CFG0 = 0xC8
    value = 0xC8;
    error = lis2duxs12_write_reg(&lis2duxs12_ctx, LIS2DUXS12_TAP_CFG0, &value, 1);
    if (error != 0) return error;

    // Step 3: TAP_CFG1 = 0x28
    value = 0x28;
    error = lis2duxs12_write_reg(&lis2duxs12_ctx, LIS2DUXS12_TAP_CFG1, &value, 1);
    if (error != 0) return error;

    // Step 4: TAP_CFG2 = 0x03
    value = 0x03;
    error = lis2duxs12_write_reg(&lis2duxs12_ctx, LIS2DUXS12_TAP_CFG2, &value, 1);
    if (error != 0) return error;

    // Step 5: TAP_CFG3 = 0x84
    value = 0x84;
    error = lis2duxs12_write_reg(&lis2duxs12_ctx, LIS2DUXS12_TAP_CFG3, &value, 1);
    if (error != 0) return error;

    // Step 6: TAP_CFG4 = 0x88
    value = 0x88;
    error = lis2duxs12_write_reg(&lis2duxs12_ctx, LIS2DUXS12_TAP_CFG4, &value, 1);
    if (error != 0) return error;

    // Step 7: TAP_CFG5 = 0xE0
    // Enables single-, double-, and triple-tap events; no rebound recognition.
    value = 0xE0;
    error = lis2duxs12_write_reg(&lis2duxs12_ctx, LIS2DUXS12_TAP_CFG5, &value, 1);
    if (error != 0) return error;

    // Step 8: TAP_CFG6 = 0x0A
    value = 0x0A;
    error = lis2duxs12_write_reg(&lis2duxs12_ctx, LIS2DUXS12_TAP_CFG6, &value, 1);
    if (error != 0) return error;

    // Step 9: MD1_CFG = 0x08
    // This routes the TAP_IA signal (tap interrupt) to the RES (INT1) pin.
    value = 0x08;
    error = lis2duxs12_write_reg(&lis2duxs12_ctx, LIS2DUXS12_MD1_CFG, &value, 1);
    if (error != 0) return error;

    // Step 10: INTERRUPT_CFG = 0x01
    // Enables basic interrupts.
    value = 0x01;
    error = lis2duxs12_write_reg(&lis2duxs12_ctx, LIS2DUXS12_INTERRUPT_CFG, &value, 1);
    if (error != 0) return error;

    // Step 11: CTRL5 = 0xA2
    // Sets the sensor operating mode: ODR = 400 Hz, full scale = ±8 g.
    value = 0xA2;
    error = lis2duxs12_write_reg(&lis2duxs12_ctx, LIS2DUXS12_CTRL5, &value, 1);
    if (error != 0) return error;

    return 0;
}




int lis2duxs12_enable_double_tap_old(void)
{
    int16_t error;
    lis2duxs12_md_t md = {
        .odr = LIS2DUXS12_400Hz_LP,
        .fs  = LIS2DUXS12_2g,
        .bw  = LIS2DUXS12_ODR_div_4
    };

    // Enable embedded functions
    error = lis2duxs12_init_set(&lis2duxs12_ctx, LIS2DUXS12_SENSOR_EMB_FUNC_ON);
    if (error != 0) {
        return error;
    }

    // Set accelerometer operating mode (ODR, FS, BW)
    error = lis2duxs12_mode_set(&lis2duxs12_ctx, &md);
    if (error != 0) {
        return error;
    }

    // Enable tap detection in the MD1 configuration register (routes tap events on INT1)
    lis2duxs12_md1_cfg_t md1_cfg;
    error = lis2duxs12_read_reg(&lis2duxs12_ctx, LIS2DUXS12_MD1_CFG, (uint8_t *)&md1_cfg, 1);
    if (error != 0) {
        return error;
    }
    md1_cfg.int1_tap = PROPERTY_ENABLE;  // Enable tap detection
    error = lis2duxs12_write_reg(&lis2duxs12_ctx, LIS2DUXS12_MD1_CFG, (uint8_t *)&md1_cfg, 1);
    if (error != 0) {
        return error;
    }

    // Enable double tap detection in the TAP_CFG5 register.
    lis2duxs12_tap_cfg5_t tap_cfg5;
    error = lis2duxs12_read_reg(&lis2duxs12_ctx, LIS2DUXS12_TAP_CFG5, (uint8_t *)&tap_cfg5, 1);
    if (error != 0) {
        return error;
    }
    tap_cfg5.double_tap_en = PROPERTY_ENABLE;  // Enable double tap
    error = lis2duxs12_write_reg(&lis2duxs12_ctx, LIS2DUXS12_TAP_CFG5, (uint8_t *)&tap_cfg5, 1);
    if (error != 0) {
        return error;
    }

    /* Route DRDY interrupt to RES (INT1) pin */
    lis2duxs12_pin_int_route_t int_route = {0};
    int_route.int_on_res = 1;
    int_route.drdy       = 0;
    int_route.fifo_ovr   = 0;
    int_route.fifo_th    = 0;
    int_route.fifo_full  = 0;
    int_route.boot       = 0;
    int_route.free_fall  = 0;
    int_route.six_d      = 0;
    int_route.tap        = 1;
    int_route.wake_up    = 0;
    int_route.sleep_change = 0;
    int_route.emb_function = 1;
    int_route.timestamp  = 0;
    error = lis2duxs12_pin_int1_route_set(&lis2duxs12_ctx, &int_route);
    if (error != NO_ERROR) {
        LOG_ERR("Error routing interrupt to INT1: %d", error);
    }

    return 0;
}



K_THREAD_DEFINE(imu_tid, IMU_STACK_SIZE, imu_receive_thread, NULL, NULL, NULL, IMU_PRIORITY, 0, 0);


