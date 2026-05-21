/*
 * ----------------------------------------------------------------------
 *
 * File: command_dispatcher.c
 *
 * Last edited: 07.05.2026
 *
 * Copyright (C) 2026, ETH Zurich and University of Bologna.
 *
 * Authors:
 * - Giusy Spacone (gspacone@iis.ee.ethz.ch), ETH Zurich
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




#include "core/command_dispatcher.h"
#include <zephyr/logging/log.h>
#include "ble/ble_appl.h"

LOG_MODULE_REGISTER(command_dispatcher, LOG_LEVEL_DBG);

/**
 * @brief Process Connectivity Command
 *
 * Handles all commands received from the connected device. 
 * Connection can be either via Wi-Fi or BLE, depeding on the desired KConfig settings.
 * Commands include battery state requests, device settings, streaming
 * control (Nordic and microphone), board state management, and more.
 *
 * @param cmd The command byte received from connectivity (first byte of packet)
 */
void handle_connectivity_command(uint8_t cmd) {
  switch (cmd) {

  case REQUEST_BATTERY_STATE:
    LOG_DBG("Ping REQUEST_BATTERY_STATE");
    size_t out_len = 0;
    uint8_t bat_data[7];
    if (system_status_build_packet(bat_data, sizeof(bat_data), &out_len) == 0) {
  
  #ifndef CONFIG_WI_FI
      send_data_ble(bat_data, (uint16_t)out_len);
  #else
      /* TODO: send via WiFi transport */
  #endif
    } else {
      LOG_ERR("Failed to build battery status packet");
    }
    break;

  case GET_DEVICE_SETTINGS:
    LOG_DBG("Ping GET_DEVICE_SETTINGS");
    system_status_send_device_settings();
    break;

  case SET_DEVICE_SETTINGS:
    LOG_DBG("Ping SET_DEVICE_SETTINGS");
    break;

  case REQUEST_CONNECTING_STRING:
    LOG_DBG("Ping REQUEST_CONNECTING_STRING");
    system_status_send_ready();
    break;

  case REQUEST_HARDWARE_VERSION:
    LOG_DBG("Ping REQUEST_HARDWARE_VERSION");
    system_status_send_hardware_version();
    break;

  case REQUEST_FIRMWARE_VERSION:
    LOG_DBG("Ping REQUEST_FIRMWARE_VERSION");
    system_status_send_firmware_version();
    break;

  case REQUEST_AVAILABLE_SENSORS:
    LOG_DBG("Ping REQUEST_AVAILABLE_SENSORS");
    system_status_send_available_sensors();
    break;

  case GET_BOARD_STATE: {
    LOG_DBG("Ping GET_BOARD_STATE");
    int8_t current_state = system_status_get_board_state();
    LOG_DBG("Sending current state: %d", current_state);
  
    #ifndef CONFIG_WI_FI
    send_data_ble(&current_state, 1);
  #else
    /* TODO: send via WiFi transport */
  #endif
    break;
  }

  case SET_BOARD_STATE:
    LOG_DBG("Ping SET_BOARD_STATE");
#ifndef CONFIG_WI_FI
    LOG_DBG(".data[1], %d", ble_data_available.data[1]);
    LOG_DBG(".data[2], %d", ble_data_available.data[2]);

    if (ble_data_available.data[1] == 1) {
      system_status_set_board_state(STATE_STREAMING_NORDIC);
    } else {
      system_status_set_board_state(STATE_GAP9_MASTER);
    }
#else
    /* TODO: Handle SET_BOARD_STATE from WiFi transport */
#endif

    if (system_status_get_board_state() == STATE_STREAMING_NORDIC) {
      ads_set_function(ADS_STILL);
    } else {
      ads_set_function(ADS_INIT_GAP9_CTRL);
    }
    break;

  case RESET_GAP9:
    LOG_DBG("Ping RESET_GAP9");
    break;

  case RESET_BOARD:
    LOG_DBG("Ping RESET_BOARD");
    break;

  case SET_TRIGGER_STATE:
    LOG_DBG("Ping SET_TRIGGER_STATE (deprecated - trigger removed)");
    break;

  case ENTER_BOOTLOADERT_MODE:
    LOG_DBG("Ping ENTER_BOOTLOADER_MODE");
    break;

  case GO_TO_SLEEP:
    LOG_DBG("Ping GO_TO_SLEEP");
    break;

  case START_EEG_STREAMING:
    LOG_INF("Ping START_EEG_STREAMING");
    
  #ifndef CONFIG_WI_FI
    ble_reset_packet_counters(); /* Reset BLE packet counters for new session */
  #else
    /* TODO: reset WiFi transport packet counters */
  #endif
    eeg_start_streaming();
    break;

  case STOP_EEG_STREAMING:
    LOG_INF("Ping STOP_EEG_STREAMING");
    eeg_stop_streaming();
  #ifndef CONFIG_WI_FI
    ble_print_packet_stats(); /* Print BLE packet stats */
  #else
    /* TODO: print WiFi transport stats */
  #endif
    break;

  case START_EMG_STREAMING:
    LOG_INF("Ping START_EMG_STREAMING");
  #ifndef CONFIG_WI_FI
    ble_reset_packet_counters(); /* Reset packet counters for new session */
  #else
    /* TODO: reset WiFi transport packet counters */
  #endif
    LOG_INF("Starting EMG streaming");
    emg_start_streaming();
    break;

  case STOP_EMG_STREAMING:
    LOG_INF("Ping STOP_EMG_STREAMING");
    emg_stop_streaming();
  #ifndef CONFIG_WI_FI
    ble_print_packet_stats(); /* Print BLE packet stats */
  #else
    /* TODO: print WiFi transport stats */
  #endif
    break;
  
  case START_MIC_STREAMING:
    LOG_INF("Ping START_MIC_STREAMING");
    mic_start_streaming();
    break;

  case STOP_MIC_STREAMING:
    LOG_INF("Ping STOP_MIC_STREAMING");
    mic_stop_streaming();
    break;

  case START_EEG_MIC_STREAMING:
    LOG_DBG("Ping START_EEG_MIC_STREAMING");
    #ifndef CONFIG_WI_FI
      ble_reset_packet_counters(); /* Reset packet counters for new session */
    #else
    /* TODO: reset WiFi transport packet counters */
    #endif
    sync_begin(2);               /* Setup sync barrier for 2 subsystems (EEG + MIC) */
    mic_start_streaming();
    eeg_start_streaming();
    break;
  
  case STOP_EEG_MIC_STREAMING:
    LOG_DBG("Ping STOP_EEG_MIC_STREAMING");
    mic_stop_streaming();
    eeg_stop_streaming();
  
  #ifndef CONFIG_WI_FI
    ble_print_packet_stats(); /* Print BLE packet stats */
  #else
    /* TODO: print WiFi transport stats */
  #endif
    sync_reset();             /* Clean up sync state */
    break;

  
  case START_STREAMING_ALL:
    LOG_DBG("Ping START_STREAMING_ALL");
  #ifndef CONFIG_WI_FI
    ble_reset_packet_counters(); /* Reset BLE packet counters for new session */
  #else
    /* TODO: reset WiFi transport packet counters */
  #endif
    sync_begin(3);               /* Setup sync barrier for 2 subsystems (EEG + MIC + IMU) */
    mic_start_streaming();
    eeg_start_streaming();
    imu_start_streaming();
    break;

  case STOP_STREAMING_ALL:
    LOG_DBG("Ping STOP_STREAMING_ALL");
    mic_stop_streaming();
    eeg_stop_streaming();
    imu_stop_streaming();

  #ifndef CONFIG_WI_FI
    ble_print_packet_stats(); /* Print BLE packet stats */
  #else
    /* TODO: print WiFi transport stats */
  #endif
    sync_reset();             /* Clean up sync state */
    break;

    
  case START_IMU_STREAMING:
    LOG_DBG("Ping START_IMU_STREAMING");
    imu_start_streaming();
    break;
  case STOP_IMU_STREAMING:
    LOG_DBG("Ping STOP_IMU_STREAMING");
    imu_stop_streaming();
    break;
  }
}