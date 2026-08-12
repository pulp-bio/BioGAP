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
#include "ble/bluetooth.h"
#include "sensors/eeg/eeg_appl.h"
#include "sensors/emg/emg_appl.h"
#include "sensors/imu/imu_appl.h"
#include "sensors/mic/mic_appl.h"
#include "sensors/dummy_sensor/dummy_sensor_appl.h"
#if defined(CONFIG_WI_FI)
#include "wifi_sd_shield/wifi_sd_shield_appl.h"
#include "wifi_sd_shield/wifi_sd_shield_defs.h"

uint16_t esp_spi_packet_size = 0; // Global variable to hold the size of the current SPI packet from ESP
#endif
#if defined(CONFIG_SENSOR_PPG_NEW)
#include "sensors/ppg_new/ppg_new_appl.h"
#endif
#if defined(CONFIG_SENSOR_WULPUS)
#include "sensors/wulpus/wulpus_appl.h"
#endif
#if defined(CONFIG_SENSOR_MMWAVE)
#include "sensors/mmWave/mmWave_appl.h"
#endif

uint8_t ads_config[5] = {6, 5, 2, 4, 0x10};      // For Exg Data. Hard-coded config with square wave
bool wulpus_restart_rcv = false;

LOG_MODULE_REGISTER(command_dispatcher, LOG_LEVEL_DBG);

/**
 * @brief Process Connectivity Command
 *
 * Handles all commands received from the connected device.
 * Connection can be either via Wi-Fi or BLE, depeding on the desired KConfig settings.
 * Commands include battery state requests, device settings, streaming
 * control (Nordic and microphone), board state management, and more.
 *
 * @param data Full received payload; data[0] is the command byte (see the
 *             header doc for how commands needing more than the opcode are
 *             handled when the transport can't supply it).
 * @param size Number of valid bytes in data (>= 1).
 */
void handle_connectivity_command(const uint8_t *data, uint16_t size) {
  uint8_t cmd = data[0];
  switch (cmd) {

  case REQUEST_BATTERY_STATE:
    LOG_DBG("Ping REQUEST_BATTERY_STATE");
    size_t out_len = 0;
    uint8_t bat_data[7];
    if (system_status_build_packet(bat_data, sizeof(bat_data), &out_len) == 0) {

  #ifndef CONFIG_WI_FI
      send_data_ble(bat_data, (uint16_t)out_len);
  #else
      add_data_to_esp_send_buffer(bat_data, (uint16_t)out_len);
  #endif
    } else {
      LOG_ERR("Failed to build battery status packet");
    }
    break;

  case REQUEST_SYSTEM_STATUS: {
    LOG_DBG("Ping REQUEST_SYSTEM_STATUS");
    size_t status_len = 0;
    uint8_t status_data[SYSTEM_STATUS_PACKET_LEN];
    if (system_status_build_system_status_packet(status_data, sizeof(status_data), &status_len) == 0) {
  #ifndef CONFIG_WI_FI
      send_data_ble(status_data, (uint16_t)status_len);
  #else
      add_data_to_esp_send_buffer(status_data, (uint16_t)status_len);
  #endif
    } else {
      LOG_ERR("Failed to build system status packet");
    }
    break;
  }

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
    add_data_to_esp_send_buffer((uint8_t *)&current_state, 1);
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


  case START_DUMMY_STREAMING:
    LOG_INF("Ping START_DUMMY_STREAMING");
  #if defined CONFIG_WI_FI
    /* TODO: reset WiFi transport packet counters */
  #else
    /* Reset packet counters for new session */
    ble_reset_packet_counters();
  #endif
    LOG_INF("Starting DUMMY streaming");
  #if defined(CONFIG_WI_FI)
    // check the status of Wi-Fi state machine
    if(nrf_esp_comm_state != NRF_ESP_IDLE){
        LOG_INF("NRF-ESP communication state is not idle (current state: %d) - new streaming session may be delayed until current communication is complete.", nrf_esp_comm_state);
    }
    nrf_esp_comm_state = SEND_TO_ESP; // Set state to allow sending data to ESP
  #endif
  #if defined(CONFIG_DUMMY_SENSOR)
    //esp_spi_packet_size = DUMMY_SENSOR_PCKT_SIZE; // Set the expected SPI packet size for dummy sensor
    dummy_sensor_start_streaming();
  #else
    LOG_WRN("Dummy sensor not built (CONFIG_DUMMY_SENSOR=n) - ignoring START_DUMMY_STREAMING");
  #endif
    break;


  case STOP_DUMMY_STREAMING:
    LOG_INF("Ping STOP_DUMMY_STREAMING");
  #if defined(CONFIG_WI_FI)
    nrf_esp_comm_state = NRF_ESP_IDLE; // Reset state to idle to block sending data to ESP
  #endif
  #if defined(CONFIG_DUMMY_SENSOR)
    dummy_sensor_stop_streaming();
  #else
    LOG_WRN("Dummy sensor not built (CONFIG_DUMMY_SENSOR=n) - ignoring STOP_DUMMY_STREAMING");
  #endif
  #if defined CONFIG_WI_FI
    /* TODO: print WiFi transport stats */
  #else
    /* TODO: print WiFi transport stats */
    ble_print_packet_stats(); /* Print BLE packet stats */
  #endif
    break;

  case START_EEG_STREAMING:
    LOG_INF("Ping START_EEG_STREAMING");
    // the other bytes are the ads configuration
    memcpy(ads_config, data + 1, 5);
    LOG_INF("Received ADS configuration: 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X", ads_config[0], ads_config[1], ads_config[2], ads_config[3], ads_config[4]);

  #ifndef CONFIG_WI_FI
    ble_reset_packet_counters(); /* Reset BLE packet counters for new session */
  #else
    if (nrf_esp_comm_state != NRF_ESP_IDLE) {
        LOG_INF("NRF-ESP communication state is not idle (current state: %d) - new streaming session may be delayed until current communication is complete.", nrf_esp_comm_state);
    }
    nrf_esp_comm_state = SEND_TO_ESP; // Set state to allow sending data to ESP
  #endif
    eeg_start_streaming(&ads_config[0]);
    break;

  case STOP_EEG_STREAMING:
    LOG_INF("Ping STOP_EEG_STREAMING");
    eeg_stop_streaming();
  #ifndef CONFIG_WI_FI
    ble_print_packet_stats(); /* Print BLE packet stats */
  #else
    nrf_esp_comm_state = NRF_ESP_IDLE; // Reset state to idle to block sending data to ESP
  #endif
    break;

  case START_EMG_STREAMING:
    LOG_INF("Ping START_EMG_STREAMING");
    // the other bytes are the ads configuration
    memcpy(ads_config, data + 1, 5);
    LOG_INF("Received ADS configuration: 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X", ads_config[0], ads_config[1], ads_config[2], ads_config[3], ads_config[4]);

  #ifndef CONFIG_WI_FI
    ble_reset_packet_counters(); /* Reset packet counters for new session */
  #else
    if (nrf_esp_comm_state != NRF_ESP_IDLE) {
        LOG_INF("NRF-ESP communication state is not idle (current state: %d) - new streaming session may be delayed until current communication is complete.", nrf_esp_comm_state);
    }
    nrf_esp_comm_state = SEND_TO_ESP; // Set state to allow sending data to ESP
  #endif
    LOG_INF("Starting EMG streaming");
    emg_start_streaming(&ads_config[0]);
    break;

  case STOP_EMG_STREAMING:
    LOG_INF("Ping STOP_EMG_STREAMING");
    emg_stop_streaming();
  #ifndef CONFIG_WI_FI
    ble_print_packet_stats(); /* Print BLE packet stats */
  #else
    nrf_esp_comm_state = NRF_ESP_IDLE; // Reset state to idle to block sending data to ESP
  #endif
    break;

  case START_PPG_STREAMING:
    #if defined(CONFIG_SENSOR_PPG_NEW)
    LOG_INF("Ping START_PPG_STREAMING");
  #ifndef CONFIG_WI_FI
    ble_reset_packet_counters();
  #endif
    {
      /*
       * Payload layout (bytes after command code 39):
       *   [1]  sensor_mask
       *   [2]  sample_rate_hz
       *   [3]  led_green     (0x00 = disable)
       *   [4]  led_ir        (0x00 = disable)
       *   [5]  led_red       (0x00 = disable)
       *   [6]  led_range     (0-3 index)
       *   [7]  tint          (0-3 index)
       *   [8]  adc_range     (0-3 index)
       *   [9]  sample_avg    (0-7 index)
       *   [10] alc_enable    (0/1)
       *   [11] proximity_enable (0/1)
       * Total packet size: 12 bytes (cmd + 11 config bytes).
       * If the payload is shorter than expected (e.g. the WiFi shield's
       * fixed-size command transaction, which only ever supplies the
       * opcode), missing bytes fall back to these defaults.
       */
      const uint8_t *d = data;
      uint16_t       n = size;

      ppg_config_t pcfg = {
        .sensor_mask      = (n >  1) ? d[1]  : 0x01,
        .sample_rate_hz   = (n >  2) ? d[2]  : 125,
        .led_green        = (n >  3) ? d[3]  : 0x7F,
        .led_ir           = (n >  4) ? d[4]  : 0x7F,
        .led_red          = (n >  5) ? d[5]  : 0x7F,
        .led_range        = (n >  6) ? d[6]  : 3,   /* default 32k */
        .tint             = (n >  7) ? d[7]  : 3,   /* default 117.3 µs */
        .adc_range        = (n >  8) ? d[8]  : 3,   /* default 32k */
        .sample_avg       = (n >  9) ? d[9]  : 0,   /* default 1x */
        .alc_enable       = (n > 10) ? d[10] : 1,   /* default on */
        .proximity_enable = (n > 11) ? d[11] : 0,   /* default off */
      };

      ppg_new_start_streaming(&pcfg);
    }
    #endif
    break;

  case STOP_PPG_STREAMING:
    #if defined(CONFIG_SENSOR_PPG_NEW)
    LOG_INF("Ping STOP_PPG_STREAMING");
    ppg_new_stop_streaming();
    #endif
    break;

  case START_WULPUS_STREAMING:
    #if defined(CONFIG_SENSOR_WULPUS)
      LOG_INF("Ping START_WULPUS_STREAMING");

      #if defined(CONFIG_WI_FI)
        // ESP relay accumulates the whole config+start sequence and sends it
        // as one control frame, so both packages are already back-to-back
        // in data[] -- no need to wait for a second, separate dispatch.

        LOG_INF("WULPUS config package received, forwarding to wulpus_set_msp_config");

        if (nrf_esp_comm_state != NRF_ESP_IDLE) {
            LOG_INF("NRF-ESP communication state is not idle (current state: %d) - new streaming session may be delayed until current communication is complete.", nrf_esp_comm_state);
        }
        nrf_esp_comm_state = SEND_TO_ESP; // Set state to allow sending data to ESP
        wulpus_set_msp_config(&data[1], MSP_RESTART_PCK_LEN);
        wulpus_set_msp_config(&data[1 + MSP_RESTART_PCK_LEN], MSP_RESTART_PCK_LEN);
        LOG_INF("WULPUS config package forwarded to wulpus_set_msp_config");
        wulpus_cfg_sent = true; // Set the flag to indicate that the WULPUS config has been sent to the MSP430

      #else
        // BLE streaming -- packet can be fragmented; the conf package
        // arrives later as its own separate dispatch, via default: below.
        wulpus_set_msp_config(size > 1 ? data + 1 : NULL, size > 1 ? size - 1 : 0);
      #endif

      wulpus_restart_rcv = true; // Set the flag to indicate that a restart command has been received
    #endif
    break;

  case STOP_WULPUS_STREAMING:
    #if defined(CONFIG_SENSOR_WULPUS)
      LOG_INF("Ping STOP_WULPUS_STREAMING");
      wulpus_stop();
      wulpus_restart_rcv = false;
      #if defined(CONFIG_WI_FI)
        nrf_esp_comm_state = NRF_ESP_IDLE; // Reset state to idle to block sending data to ESP
      #endif
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
    eeg_start_streaming(&ads_config[0]);
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
    eeg_start_streaming(&ads_config[0]);
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

  /* The case labels themselves are conditional, not just their bodies: without
   * the radar, opcodes 44-51 must keep falling through to default:, which
   * forwards unrecognised payloads to the WULPUS MSP430 (a config blob may
   * legitimately start with one of these bytes). */
#if defined(CONFIG_SENSOR_MMWAVE)
  case TURN_ON_MMWAVE:
    LOG_INF("Ping TURN_ON_MMWAVE");
    mmWave_power_on();
    break;

  case TURN_OFF_MMWAVE:
    LOG_INF("Ping TURN_OFF_MMWAVE");
    mmWave_power_off();
    break;

  case CONFIGURE_MMWAVE:
    LOG_INF("Ping CONFIGURE_MMWAVE");
    mmWave_configure();
    break;

  case START_MMWAVE_STREAMING:
    LOG_INF("Ping START_MMWAVE_STREAMING");
    #ifndef CONFIG_WI_FI
    ble_reset_packet_counters(); /* Reset packet counters for new session */
    #endif
    mmWave_start_streaming();
    break;

  case STOP_MMWAVE_STREAMING:
    LOG_INF("Ping STOP_MMWAVE_STREAMING");
    mmWave_stop_streaming();
    #ifndef CONFIG_WI_FI
    ble_print_packet_stats(); /* Print BLE packet stats */
    #endif
    break;

  /* The three setters below take their value from data[1]; a caller that can
   * only supply the opcode leaves the current setting in place. */
  case CHANGE_IFGAIN_MMWAVE:
    LOG_INF("Ping CHANGE_IFGAIN_MMWAVE");
    if (size > 1) {
      mmWave_set_ifGain(data[1]);
    } else {
      LOG_WRN("CHANGE_IFGAIN_MMWAVE without a value byte - ignored");
    }
    break;

  case CHANGE_TXPOWER_MMWAVE:
    LOG_INF("Ping CHANGE_TXPOWER_MMWAVE");
    if (size > 1) {
      mmWave_set_txPower(data[1]);
    } else {
      LOG_WRN("CHANGE_TXPOWER_MMWAVE without a value byte - ignored");
    }
    break;

  case CHANGE_FPS_MMWAVE:
    LOG_INF("Ping CHANGE_FPS_MMWAVE");
    if (size > 1) {
      mmWave_set_fps(data[1]);
    } else {
      LOG_WRN("CHANGE_FPS_MMWAVE without a value byte - ignored");
    }
    break;
#endif /* CONFIG_SENSOR_MMWAVE */

  default:
    /*
     * Command code not recognised - treat the full payload as MSP430
     * configuration for the WULPUS dongle (which sends raw config bytes
     * without a preceding command code, exactly as the old nRF52 firmware).
     */
    #if defined(CONFIG_SENSOR_WULPUS)
        if (wulpus_restart_rcv == true) {
          LOG_INF("Received WULPUS configuration after restart command, forwarding to wulpus_set_msp_config");
          wulpus_set_msp_config(data, size);
        }
    #else
        LOG_WRN("Unrecognised command: %u", cmd);
    #endif
    break;
  }
}
