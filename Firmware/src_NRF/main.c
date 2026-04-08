/*
 * ----------------------------------------------------------------------
 *
 * File: main.c
 *
 * Last edited: 30.10.2025
 *
 * Copyright (c) 2024 ETH Zurich and University of Bologna
 *
 * Authors:
 * - Philip Wiese (wiesep@iis.ee.ethz.ch), ETH Zurich
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

#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/gatt.h>
#include <zephyr/bluetooth/hci.h>
#include <zephyr/bluetooth/uuid.h>
#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/drivers/uart.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/logging/log_ctrl.h>
#include <zephyr/sys/ring_buffer.h>
#include <zephyr/usb/usb_device.h>
#include <zephyr/usb/usbd.h>

#include "bsp/pwr_bsp.h"
#include "pwr/pwr.h"
#include "pwr/pwr_common.h"
#include "max77654.h"

#include "afe/ads_spi.h"
#include "ble/ble_appl.h"
#include "core/common.h"
#include "sensors/imu/imu_appl.h"
#include "sensors/mic/mic_appl.h"
#include "sensors/eeg/eeg_appl.h"

// Inter-board hardware synchronization
#include "core/board_sync.h"

static const struct device *const uart_dev = DEVICE_DT_GET_ONE(zephyr_cdc_acm_uart);

LOG_MODULE_REGISTER(main, LOG_LEVEL_INF);

#define UART_BUF_SIZE 40
#define MINIMAL_STACK_SIZE 1024

struct uart_data_t {
  void *fifo_reserved;
  uint8_t data[UART_BUF_SIZE];
  uint16_t len;
};

void z_fatal_error(unsigned int reason, const z_arch_esf_t *esf) {
  LOG_INF("Fatal error occurred: %d", reason);
  while (1) {
    // Halt here for debugging
  }
}

int main(void) {
  int ret = 0;

  LOG_INIT();

  LOG_INF("LED Test on %s", CONFIG_BOARD);


  if (pwr_init()) {
    LOG_ERR("PWR Init failed!");
  }
  // pwr_start();
  if (pwr_bsp_start()) {
    LOG_ERR("PWR BSP Start failed!");
  }

  if (!device_is_ready(uart_dev)) {
    LOG_ERR("CDC ACM device not ready");
    return 0;
  }

  if (usb_enable(NULL)) {
    return 0;
  }
  LOG_INF("USB enabled");

  LOG_INF("Enabling charge...");
  pwr_charge_enable();
  LOG_INF("Initializing ADS...");
  ret = ads_dr_init();

  LOG_INF("Initializing SPI...");
  init_spi();

  LOG_INF("Powering GAP9...");
  gap9_pwr(true);
  LOG_INF("GAP9 powered up");

  struct uart_data_t *buf = k_malloc(sizeof(*buf));
  LOG_INF("Starting BLE adverts...");
  start_bluetooth_adverts();

  // Initialize microphone
  LOG_INF("Initializing microphone...");
  if (mic_init() != 0) {
    LOG_WRN("Microphone initialization failed - mic streaming disabled");
  } else {
    LOG_INF("Microphone initialized");
  }

  // Initialize IMU (LIS2DUXS12 accelerometer)
  LOG_INF("Initializing IMU...");
  if (imu_init() != 0) {
    LOG_WRN("IMU initialization failed - IMU streaming disabled");
  } else {
    LOG_INF("IMU initialized");
  }

#if defined(CONFIG_SENSOR_EEG) && !defined(CONFIG_SENSOR_EMG)
  // Initialize EEG subsystem
  LOG_INF("Initializing EEG subsystem...");
  if (eeg_init() != 0) {
    LOG_WRN("EEG initialization failed - EEG streaming disabled");
  } else {
    LOG_INF("EEG subsystem initialized");
  }
#endif

#if defined(CONFIG_SENSOR_EMG) && !defined(CONFIG_SENSOR_EEG)
  // Initialize EMG subsystem
  LOG_INF("Initializing EMG subsystem...");
  if (emg_init() != 0) {
    LOG_WRN("EMG initialization failed - EMG streaming disabled");
  } else {
    LOG_INF("EMG subsystem initialized");
  }
#endif

  // Initialize inter-board synchronization
  LOG_INF("Initializing board sync...");
  if (board_sync_init() != 0) {
    LOG_WRN("Board sync initialization failed - inter-board sync disabled");
  } else {
    LOG_INF("Board sync initialized");
  }

  while (1) {
    k_msleep(1000); // Main thread can sleep now, all the work is handeled by other threads
    if (flag_isr_soft_reset) {
      // Soft reset the device
      // Put PMIC into factory reset
      // do a nop in a busy for loop for 100ms to allow the PMIC to process the command
      volatile int i;
      for (i = 0; i < 10000000; i++) {
        __asm__ volatile("nop");
      }

      int ret = max77654_factory_ship_mode(&pmic_h);
      if (ret != 0) {
        LOG_ERR("Failed to soft reset PMIC (error %d)", ret);
      } else {
        LOG_INF("PMIC soft reset triggered");
      }
      flag_isr_soft_reset = 0;
      for (i = 0; i < 10000000; i++) {
        __asm__ volatile("nop");
      }
    }
  }
  return 0;
}
