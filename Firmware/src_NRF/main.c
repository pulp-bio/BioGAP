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

#include "afe/ads_appl.h"
#include "afe/ads_spi.h"
#include "ble/ble_appl.h"
#include "core/common.h"
#include "sensors/imu/imu_appl.h"
#include "sensors/mic/mic_appl.h"
#include "sensors/eeg/eeg_appl.h"
#include "sensors/emg/emg_appl.h"
#if defined(CONFIG_SENSOR_PPG_NEW)
#include "sensors/ppg_new/ppg_new_appl.h"
#endif
#if defined(CONFIG_SENSOR_WULPUS)
#include "sensors/wulpus/wulpus_appl.h"
#endif
#if defined(CONFIG_WI_FI)
#include "wifi_sd_shield/wifi_sd_shield_inits.h"
#include "wifi_sd_shield/wifi_sd_shield_appl.h"
#endif
#if defined(CONFIG_DUMMY_SENSOR)
#include "sensors/dummy_sensor/dummy_sensor_appl.h"
#endif

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

/* ExG acquisition is disturbed by I2C READ transactions from the PMIC
 * (measured with the CONFIG_PMIC_NOISE_TEST sweep: reads inject noise
 * bursts into EEG/EMG - the PMIC sinks the SDA pull-up current through its
 * die ground - while writes and AMUX/SAADC measurements are clean). While
 * the ADS streams, the power thread therefore runs read-free quiet cycles:
 * battery voltage/SoC/currents stay live, status flags and charger
 * operations are frozen until the stream stops. */
static bool pmic_measurements_allowed(void) { return ads_get_function() != ADS_READ; }

int main(void) {
  int ret = 0;

  LOG_INIT();

  LOG_INF("LED Test on %s", CONFIG_BOARD);


  if (pwr_init()) {
    LOG_ERR("PWR Init failed!");
  }

  if (!device_is_ready(uart_dev)) {
    LOG_ERR("CDC ACM device not ready");
    return 0;
  }

  if (usb_enable(NULL)) {
    return 0;
  }
  LOG_INF("USB enabled");

//  LOG_INF("Enabling charge...");
  pwr_charge_enable();

  /* Start the SDK power thread: refreshes the battery/charger telemetry
   * cache every THREAD_PWR_UPDATE_PERIOD_MS (and on PMIC nIRQ) and
   * re-applies the charger config periodically. Runs at the lowest
   * priority and never touches the VDx/VAx rails. The LED rail stays off
   * (CONFIG_PWR_START_LED_POWER=0) and the SDK long-press shutdown stays
   * disabled in favour of the app's soft-reset -> factory-ship path
   * (CONFIG_PWR_LONG_PRESS_KILL=0); both set in CMakeLists.txt. Started
   * after pwr_charge_enable() so the periodic charger re-config re-applies
   * the final charger settings. */
  pwr_set_measurement_gate(pmic_measurements_allowed);
  if (pwr_start()) {
    LOG_ERR("PWR Start failed!");
  }


//  LOG_INF("Initializing ADS...");
  ret = ads_dr_init();

//  LOG_INF("Initializing SPI...");
  init_spi();

  LOG_INF("Powering GAP9...");
 // gap9_pwr(true);
  LOG_INF("GAP9 powered up");

#if defined(CONFIG_WI_FI)
  LOG_INF("Initializing Wi-Fi/SD shield...");
  ret = wifi_sd_shield_cs_init();
  if (ret != 0) {
    LOG_ERR("Wi-Fi/SD shield initialization failed");
  }
  ret = initial_handshake_nrf_esp();
  if (ret != 0) {
    LOG_ERR("Initial handshake with ESP32 failed - Wi-Fi communication not established");
  } else {
    LOG_INF("Wi-Fi/SD shield initialized");
  }
#else
  struct uart_data_t *buf = k_malloc(sizeof(*buf));
  LOG_INF("Starting BLE adverts...");
  start_bluetooth_adverts();
#endif


#if defined(CONFIG_DUMMY_SENSOR)
  LOG_INF("Initializing Dummy Sensor...");
  if (dummy_sensor_init() != 0) {
    LOG_ERR("Dummy sensor initialization failed");
  } else {
    LOG_INF("Dummy sensor initialized");
  }
#endif

/*    COMMENTED OUT FOR NOW 
  // Initialize microphone
  LOG_INF("Initializing microphone...");
  //if (mic_init() != 0) {
  //  LOG_WRN("Microphone initialization failed - mic streaming disabled");
  //} else {
  //  LOG_INF("Microphone initialized");
 // }

  // Initialize IMU (LSM6DSV16BX accelerometer + gyroscope)
  LOG_INF("Initializing IMU...");
  if (imu_init() != 0) {
    LOG_WRN("IMU initialization failed - IMU streaming disabled");
  } else {
    LOG_INF("IMU initialized");
  }
*/

/* EEG and EMG can both be built into one image; the mode (unipolar vs
 * bipolar rails) is selected at runtime by the GUI start command, and
 * simultaneous streaming is rejected in eeg/emg_start_streaming(). */

/*
#if defined(CONFIG_SENSOR_EEG)
  // Initialize EEG subsystem
  LOG_INF("Initializing EEG subsystem...");
  if (eeg_init() != 0) {
    LOG_WRN("EEG initialization failed - EEG streaming disabled");
  } else {
    LOG_INF("EEG subsystem initialized");
  }
#endif

#if defined(CONFIG_SENSOR_EMG)
  // Initialize EMG subsystem
  LOG_INF("Initializing EMG subsystem...");
  if (emg_init() != 0) {
    LOG_WRN("EMG initialization failed - EMG streaming disabled");
  } else {
    LOG_INF("EMG subsystem initialized");
  }
#endif

#if defined(CONFIG_SENSOR_PPG_NEW)
  // Initialize multi-PPG subsystem (MAXM86161 × N via TCA9548A MUX)
  LOG_INF("Initializing PPG subsystem...");
  if (ppg_new_init() != 0) {
    LOG_WRN("PPG init failed - PPG streaming disabled");
  } else {
    LOG_INF("PPG subsystem initialized");
  }
#endif

#if defined(CONFIG_SENSOR_WULPUS)
  // Initialize WULPUS ultrasound sensor interface (MSP430 SPI bridge).
  // Only nRF-side GPIOs/SPI are set up here; the shield rails (VA0/VD0/VD2,
  // incl. the 5 V boost) are powered on demand when the first WULPUS config
  // arrives via BLE (see wulpus_set_msp_config), so they stay off during
  // EEG/EMG-only sessions.
  LOG_INF("Initializing WULPUS...");
  wulpus_init();
  LOG_INF("WULPUS initialized");
#endif

  // Initialize inter-board synchronization
  LOG_INF("Initializing board sync...");
  //  if (board_sync_init() != 0) {
  //   LOG_WRN("Board sync initialization failed - inter-board sync disabled");
  //} else {
  //   LOG_INF("Board sync initialized");
    //}


  */
  while (1) {
    k_msleep(1000); // Main thread can sleep now, all the work is handeled by other threads
    //gpio_pin_set_dt(&ppg_sync_gpio, 1);
    //k_msleep(1000);
    //gpio_pin_set_dt(&ppg_sync_gpio, 0);


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
