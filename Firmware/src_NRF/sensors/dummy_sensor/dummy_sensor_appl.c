/*
 * ----------------------------------------------------------------------
 *
 * File: dummy_sensor.c
 *
 * Last edited: 20.05.2026
 *
 * Copyright (C) 2026, ETH Zurich
 *
 * Authors:
 * - Giusy Spacone (gspacone@iis.ee.ethz.ch), ETH Zurich
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

 /**
 * @file dummy_sensor_appl.c
 * @brief Dummy Sensor Application Interface
 *
 * This file contains the application-level implementation for simulating a sensor.
 * It can be used for testing the data flow and integration of the system without requiring actual sensor hardware.
 * The dummy sensor will generate synthetic data at a configurable rate and send it through the same pathways as rwal sensors. 
*/

#include "dummy_sensor_appl.h"
#include <stdbool.h>
#include <zephyr/kernel.h>
LOG_MODULE_REGISTER(dummy_sensor_appl, LOG_LEVEL_INF);

/*==============================================================================*/
/* Static Function Declarations */
/*==============================================================================*/

static volatile dummy_sensor_state_t dummy_sensor_state = DUMMY_SENSOR_STATE_IDLE;
static volatile bool dummy_sensor_keep_running = false;
static K_SEM_DEFINE(dummy_sensor_start_sem, 0, 1);
static uint8_t dummy_sensor_tx_buf[DUMMY_SENSOR_PCKT_SIZE];
static uint8_t dummy_sensor_buf_idx = 0;
static uint8_t dummy_sensor_pkt_counter = 0;
static bool first_run = true;


/*==============================================================================
 * Static Function Implementations
 *============================================================================*/

static void dummy_sensor_streaming_thread(void *arg1, void *arg2, void *arg3) {
  ARG_UNUSED(arg1);
  ARG_UNUSED(arg2);
  ARG_UNUSED(arg3);

  int ret;

  LOG_INF("Dummy sensor streaming thread started");

  while (1) {
    /* Wait for start signal from dummy_sensor_start_streaming() */
    k_sem_take(&dummy_sensor_start_sem, K_FOREVER);

    LOG_INF("EMG streaming thread running");

    /*
     * SYNC BARRIER (waits for all local subsystems + inter-board sync)
     * The barrier in sync_streaming.c handles both:
     * - Intra-board sync (waits for MIC if combined streaming)
     * - Inter-board sync (GPIO coordination between PRIMARY/SECONDARY)
     */
    if (sync_is_active()) {
      LOG_INF("EMG ready, waiting at sync barrier...");
      ret = sync_wait(SYNC_SUBSYSTEM_EXG, 5000);
      if (ret != 0) {
        LOG_ERR("Sync wait failed: %d", ret);
        emg_state = EMG_STATE_ERROR;
        power_ads_off();
        continue;
      }
    }

    /* === SYNCHRONIZED START POINT === */
    LOG_INF("Starting ADS1298 data acquisition");
    ads_start();
    LOG_INF("ADS1298 started");

    emg_state = EMG_STATE_STREAMING;
    ads_set_function(ADS_READ);

    while (emg_keep_running) {
      process_ads_data();
    }

    LOG_INF("EMG streaming thread stopping");
    ads_set_function(ADS_STILL);
    counter = 0;
    LOG_INF("ADS set to STILL");
    ads_stop();
    LOG_INF("ADS stopped");

    k_msleep(100);
    emg_state = EMG_STATE_IDLE;
    LOG_INF("EMG state set to IDLE");
    power_ads_off();
    LOG_INF("Powering off ADS devices");
  }

  LOG_INF("EMG streaming thread exiting");
}