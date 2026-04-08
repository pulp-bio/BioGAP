/*
 * ----------------------------------------------------------------------
 *
 * File: ble_appl.c
 *
 * Last edited: 23.07.2025
 *
 * Copyright (C) 2025, ETH Zurich
 *
 * Authors:
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
#include "ble/ble_appl.h"
#include "afe/ads_appl.h"
#include "afe/ads_spi.h"
#include "bsp/battery/battery.h"
#include "bsp/system_status/system_status.h"
#include "core/common.h"
#include "core/sync_streaming.h"
#include "sensors/eeg/eeg_appl.h"
#include "sensors/imu/imu_appl.h"
#include "sensors/imu/lis2duxs12_sensor.h"
#include "sensors/mic/mic_appl.h"

#include <zephyr/logging/log.h>
#include <zephyr/logging/log_ctrl.h>

/* Initialize the logging module */
LOG_MODULE_REGISTER(ble_appl, LOG_LEVEL_INF);

// #define PRINT_RECEIVED_DATA

/* Define message queues */
K_MSGQ_DEFINE(send_msgq, sizeof(ble_packet_t), SEND_QUEUE_SIZE, 4);
K_MSGQ_DEFINE(receive_msgq, BLE_PCKT_RECEIVE_SIZE, RECEIVE_QUEUE_SIZE, 1);

/* Define stack sizes and priorities */
#define BLE_SEND_STACK_SIZE 2048
#define BLE_SEND_PRIORITY 5

#define BLE_RECEIVE_STACK_SIZE 2048
#define BLE_RECEIVE_PRIORITY 4

ble_nus_data_t ble_data_available;

uart_to_pulp_data_t pck_uart_wolf;


K_SEM_DEFINE(ble_data_received, 0, 1);

/**
 * @brief BLE Send Thread
 *
 * This thread continuously retrieves data from the send message queue
 * and transmits it over BLE. Supports variable packet sizes.
 *
 * @param arg1 Unused
 * @param arg2 Unused
 * @param arg3 Unused
 */
void ble_send_thread(void *arg1, void *arg2, void *arg3) {
  ble_packet_t packet;
  int ret;

  LOG_INF("BLE send thread started");

  while (1) {
    // Retrieve data to send from the send_msgq
    ret = k_msgq_get(&send_msgq, &packet, K_FOREVER);
    if (ret == 0) {
      // Send with actual packet size
      send_data_ble(packet.data, packet.size);
    } else {
      LOG_ERR("Failed to get data from send_msgq (err %d)", ret);
    }
  }
}



/**
 * @brief Forward BLE Data to GAP9 via UART
 *
 * When the board is in STATE_GAP9_MASTER mode, all received BLE data
 * is forwarded to the GAP9 processor through UART. The data is copied
 * to the UART packet buffer and marked as available for transmission.
 */
static void forward_to_gap9(void) {
  LOG_DBG("Forwarding to GAP9");
  pck_uart_wolf.data_len = ble_data_available.size;
  memcpy(pck_uart_wolf.p_data, ble_data_available.data, pck_uart_wolf.data_len);
  pck_uart_wolf.is_data_available = true;
}

/**
 * @brief Process BLE Command
 *
 * Handles all BLE commands received from the connected device.
 * Commands include battery state requests, device settings, streaming
 * control (Nordic and microphone), board state management, and more.
 *
 * @param cmd The command byte received from BLE (first byte of packet)
 */
static void handle_ble_command(uint8_t cmd) {
  switch (cmd) {
  case REQUEST_BATTERY_STATE:
    LOG_DBG("Ping REQUEST_BATTERY_STATE");
    size_t out_len = 0;
    uint8_t bat_data[7];
    if (system_status_build_ble_packet(bat_data, sizeof(bat_data), &out_len) == 0) {
      send_data_ble(bat_data, (uint16_t)out_len);
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
    send_data_ble(&current_state, 1);
    break;
  }

  case SET_BOARD_STATE:
    LOG_DBG("Ping SET_BOARD_STATE");
    LOG_DBG(".data[1], %d", ble_data_available.data[1]);
    LOG_DBG(".data[2], %d", ble_data_available.data[2]);

    if (ble_data_available.data[1] == 1) {
      system_status_set_board_state(STATE_STREAMING_NORDIC);
    } else {
      system_status_set_board_state(STATE_GAP9_MASTER);
    }

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
    ble_reset_packet_counters(); /* Reset packet counters for new session */
    eeg_start_streaming();
    break;

  case STOP_EEG_STREAMING:
    LOG_INF("Ping STOP_EEG_STREAMING");
    eeg_stop_streaming();
    ble_print_packet_stats(); /* Print BLE packet stats */
    break;

  case START_EMG_STREAMING:
    LOG_INF("Ping START_EMG_STREAMING");
    ble_reset_packet_counters(); /* Reset packet counters for new session */
    LOG_INF("Starting EMG streaming");
    emg_start_streaming();
    break;

  case STOP_EMG_STREAMING:
    LOG_INF("Ping STOP_EMG_STREAMING");
    emg_stop_streaming();
    ble_print_packet_stats(); /* Print BLE packet stats */
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
    ble_reset_packet_counters(); /* Reset packet counters for new session */
    sync_begin(2);               /* Setup sync barrier for 2 subsystems (EEG + MIC) */
    mic_start_streaming();
    eeg_start_streaming();
    break;
  case STOP_EEG_MIC_STREAMING:
    LOG_DBG("Ping STOP_EEG_MIC_STREAMING");
    mic_stop_streaming();
    eeg_stop_streaming();
    ble_print_packet_stats(); /* Print BLE packet stats */
    sync_reset();             /* Clean up sync state */
    break;
  case START_STREAMING_ALL:
    LOG_DBG("Ping START_STREAMING_ALL");
    ble_reset_packet_counters(); /* Reset packet counters for new session */
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
    ble_print_packet_stats(); /* Print BLE packet stats */
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

/**
 * @brief BLE Process Received Data Thread
 *
 * Main thread for processing data received over BLE. This thread waits
 * on a semaphore for incoming data and handles it based on the current
 * board state:
 * - STATE_PROGRAM_WOLF: Data is ignored (reserved for future DFU support)
 * - STATE_GAP9_MASTER: Data is forwarded to GAP9 via UART
 * - Otherwise: Data is processed as a BLE command
 *
 * @param arg1 Unused
 * @param arg2 Unused
 * @param arg3 Unused
 */
void process_received_data_thread(void *arg1, void *arg2, void *arg3) {
  LOG_INF("BLE receive thread started");

  while (1) {
    k_sem_take(&ble_data_received, K_FOREVER);

    if (!ble_data_available.available)
      continue;

    ble_data_available.available = false;
    LOG_INF("Received data from BLE");

    // Skip processing in programming mode
    if (system_status_get_board_state() == STATE_PROGRAM_WOLF)
      continue;

    // Forward to GAP9 if in master mode
    if (system_status_get_board_state() == STATE_GAP9_MASTER) {
      forward_to_gap9();
      continue;
    }

    // Process BLE command
    uint8_t cmd = ble_data_available.data[0];
    for (int k = 0; k < ble_data_available.size; k++) {
      LOG_DBG("Data[%d]: %d", k, ble_data_available.data[k]);
    }
    handle_ble_command(cmd);
  }
}


// Function to put data into receive buffer
void add_data_to_receive_buffer(uint8_t *data) {
  int ret;

  ret = k_msgq_put(&receive_msgq, data, K_NO_WAIT);
  if (ret != 0) {
    LOG_ERR("Receive message queue overflow! Data not enqueued: %d (err %d)", data, ret);
  } else {
    LOG_DBG("Data enqueued for receiving: %d", data);
  }
}

/**
 * @brief Add Data to Send Buffer
 *
 * Enqueues data into the send message queue for transmission.
 * Supports variable packet sizes.
 *
 * @param data The pointer to the byte array to be sent over BLE.
 * @param size The size of the data to send in bytes.
 */
void add_data_to_send_buffer(uint8_t *data, uint16_t size) {
  int ret;
  ble_packet_t packet;

  // Validate size
  if (size > BLE_PCKT_MAX_SIZE) {
    LOG_ERR("Packet size %d exceeds max %d", size, BLE_PCKT_MAX_SIZE);
    return;
  }

  packet.size = size;
  memcpy(packet.data, data, size);

  ret = k_msgq_put(&send_msgq, &packet, K_NO_WAIT);
  if (ret != 0) {
    LOG_ERR("Send message queue overflow! Data not sent: %d (err %d)", data, ret);
  } else {
    LOG_DBG("Data enqueued for sending: %d", data);
  }
}


/* BLE Send Thread Definition */
K_THREAD_DEFINE(ble_send_tid, BLE_SEND_STACK_SIZE, ble_send_thread, NULL, NULL, NULL, BLE_SEND_PRIORITY, 0, 0);

/* BLE Receive Thread Definition */
K_THREAD_DEFINE(ble_receive_tid, BLE_RECEIVE_STACK_SIZE, process_received_data_thread, NULL, NULL, NULL,
                BLE_RECEIVE_PRIORITY, 0, 0);
