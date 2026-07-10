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
#include "bsp/system_status/system_status.h"
#include "core/common.h"
#include "core/command_dispatcher.h"

#include <zephyr/logging/log.h>
#include <zephyr/logging/log_ctrl.h>

/* Initialize the logging module */
LOG_MODULE_REGISTER(ble_appl, LOG_LEVEL_INF);

// #define PRINT_RECEIVED_DATA

/* Define message queues */
K_MSGQ_DEFINE(send_msgq, sizeof(ble_packet_t), SEND_QUEUE_SIZE, 4);
K_MSGQ_DEFINE(receive_msgq, BLE_PCKT_RECEIVE_SIZE, RECEIVE_QUEUE_SIZE, 1);

/* Incoming NUS command queue. Depth > 1 is essential: command handling can
 * block the receive thread for a long time (EEG rail settle ~300 ms, first
 * WULPUS rail power-up ~900 ms) while the GUI keeps sending - notably the
 * multi-fragment WULPUS MSP430 config. With the previous one-slot buffer,
 * every fragment arriving during such a window overwrote the previous one
 * and the MSP430 config assembly stayed incomplete (first start failed,
 * second start worked because the rails were already on). */
K_MSGQ_DEFINE(nus_rx_msgq, sizeof(ble_nus_data_t), NUS_RX_QUEUE_SIZE, 4);

/* Define stack sizes and priorities */
#define BLE_SEND_STACK_SIZE 2048
#define BLE_SEND_PRIORITY 5

#define BLE_RECEIVE_STACK_SIZE 2048
#define BLE_RECEIVE_PRIORITY 4

ble_nus_data_t ble_data_available;

uart_to_pulp_data_t pck_uart_wolf;

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
    /* Blocks until the next NUS message; the queue preserves every message
     * even while a previous command keeps this thread busy. */
    k_msgq_get(&nus_rx_msgq, &ble_data_available, K_FOREVER);
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
    for (int k = 0; k < ble_data_available.size; k++) {
      LOG_DBG("Data[%d]: %d", k, ble_data_available.data[k]);
    }
    handle_connectivity_command(ble_data_available.data, (uint16_t)ble_data_available.size);
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


#ifndef CONFIG_WI_FI
/* BLE Send Thread Definition */
K_THREAD_DEFINE(ble_send_tid, BLE_SEND_STACK_SIZE, ble_send_thread, NULL, NULL, NULL, BLE_SEND_PRIORITY, 0, 0);

/* BLE Receive Thread Definition */
K_THREAD_DEFINE(ble_receive_tid, BLE_RECEIVE_STACK_SIZE, process_received_data_thread, NULL, NULL, NULL,
                BLE_RECEIVE_PRIORITY, 0, 0);
#endif
