/*
 * ----------------------------------------------------------------------
 *
 * File: emg_appl.c
 *
 * Last edited: 09.12.2025
 *
 * Copyright (C) 2025, ETH Zurich and University of Bologna
 *
 * Authors:
 * - Philip Wiese (wiesep@iis.ee.ethz.ch), ETH Zurich
 * - Sebastian Frey (sefrey@iis.ee.ethz.ch), ETH Zurich
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
 * @file emg_appl.c
 * @brief EMG Application Layer Implementation
 *
 * This module implements the high-level application control for the ADS1298 EMG
 * sensor, managing data acquisition, streaming, and BLE packet formatting.
 */

#include "emg_appl.h"
#include <string.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/atomic.h>

// Include ADS1298 driver headers
#include "afe/ads_appl.h"
#include "afe/ads_defs.h"
#include "afe/ads_spi.h"

// Include BLE application header for packet transmission
#include "ble/ble_appl.h"
#include "bsp/pwr_bsp.h"
#include "bsp/power/power.h"

// Include sync streaming for synchronized start/stop
#include "core/sync_streaming.h"

// Include inter-board hardware synchronization
#include "core/board_sync.h"

LOG_MODULE_REGISTER(emg_appl, LOG_LEVEL_INF);

#define EMG_THREAD_STACK_SIZE 2048
#define EMG_THREAD_PRIORITY 5
/*==============================================================================
 * Static Variables
 *============================================================================*/

static volatile emg_state_t emg_state = EMG_STATE_IDLE;
static volatile bool emg_keep_running = false;
static K_SEM_DEFINE(emg_start_sem, 0, 1);
static uint8_t emg_tx_buf[EMG_PCKT_SIZE];
static uint8_t emg_buf_idx = 0;
static uint8_t emg_pkt_counter = 0;
static emg_config_t emg_config = {
    .sample_rate = 6,
    .ads_mode = 0,
    .channel_2_func = 2,
    .channel_4_func = 4,
    .gain = 0
};
static bool first_run = true;

/*==============================================================================
 * Static Function Declarations
 *============================================================================*/

/**
 * @brief EMG streaming thread function
 *
 * Main thread that handles EMG data acquisition and BLE transmission.
 * Runs continuously while emg_keep_running is true.
 *
 * @param arg1 Unused
 * @param arg2 Unused
 * @param arg3 Unused
 */
static void emg_streaming_thread(void *arg1, void *arg2, void *arg3);

/*==============================================================================
 * Thread Definition
 *============================================================================*/

K_THREAD_DEFINE(emg_thread_id, EMG_THREAD_STACK_SIZE, emg_streaming_thread, NULL, NULL, NULL, EMG_THREAD_PRIORITY, 0, 0);

/*==============================================================================
 * Public Function Implementations
 *============================================================================*/

int emg_init(void) {

  LOG_INF("Initializing EMG subsystem");

  // Set initial state
  emg_state = EMG_STATE_IDLE;
  emg_keep_running = false;
  emg_buf_idx = 0;
  emg_pkt_counter = 0;

  LOG_INF("EMG subsystem initialized successfully");
  return 0;
}

int emg_start_streaming(void) {
  #if defined(CONFIG_SENSOR_EEG)
    LOG_ERR("EEG sensor enabled - cannot start EMG streaming");
    return -EINVAL;
   #endif

   #if !defined(CONFIG_SENSOR_EEG) && !defined(CONFIG_SENSOR_EMG)
    LOG_ERR("No sensor enabled - enable either EEG or EMG in Kconfig");
    return -EINVAL;
   #endif

  if (emg_state != EMG_STATE_IDLE) {
    LOG_ERR("EMG not in idle state, current state: %d", emg_state);
    return -EBUSY;
  }

  LOG_INF("Starting EMG streaming");

  emg_state = EMG_STATE_STARTING;
  emg_keep_running = true;
  emg_buf_idx = 0;
  emg_pkt_counter = 0;

  if (power_exg_on() != 0) {
    LOG_ERR("Power on failed - cannot start EMG streaming");
    emg_state = EMG_STATE_ERROR;
    return -EINVAL;
  }
  k_msleep(300);

  if (first_run) {
    first_run = false;
    LOG_INF("Checking ADS1298 device IDs");
    ads_check_id(ADS1298_A);
    ads_check_id(ADS1298_B);
  }

  LOG_INF("Initializing ADS1298 devices with provided parameters");
  uint8_t ads_params[5] = {
      emg_config.sample_rate,
      emg_config.ads_mode,
      emg_config.channel_2_func,
      emg_config.channel_4_func,
      emg_config.gain
  };
  ads_init(ads_params, ADS1298_A);
  ads_init(ads_params, ADS1298_B);

  /* Signal thread to complete startup (sync barrier + ads_start) */
  k_sem_give(&emg_start_sem);
  LOG_INF("Signaled EMG streaming thread to start");

  return 0;
}

int emg_stop_streaming(void) {
  if (emg_state != EMG_STATE_STREAMING) {
    LOG_ERR("EMG not currently streaming, current state: %d", emg_state);
    return -EINVAL;
  }

  LOG_INF("Stopping EMG streaming");

  emg_state = EMG_STATE_STOPPING;
  emg_keep_running = false;

  LOG_INF("EMG streaming stopped");

  return 0;
}

emg_state_t emg_get_state(void) { return emg_state; }

bool emg_is_streaming(void) { return (emg_state == EMG_STATE_STREAMING); }

int emg_set_config(const emg_config_t *config) {
  if (!config) return -1;
  memcpy(&emg_config, config, sizeof(emg_config_t));
  return 0;
}

int emg_get_config(emg_config_t *config) {
  if (!config) return -1;
  memcpy(config, &emg_config, sizeof(emg_config_t));
  return 0;
}

/*==============================================================================
 * Static Function Implementations
 *============================================================================*/

static void emg_streaming_thread(void *arg1, void *arg2, void *arg3) {
  ARG_UNUSED(arg1);
  ARG_UNUSED(arg2);
  ARG_UNUSED(arg3);

  int ret;

  LOG_INF("EMG streaming thread started");

  while (1) {
    /* Wait for start signal from emg_start_streaming() */
    k_sem_take(&emg_start_sem, K_FOREVER);

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