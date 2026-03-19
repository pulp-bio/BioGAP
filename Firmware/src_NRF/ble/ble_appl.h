/*
 * ----------------------------------------------------------------------
 *
 * File: ble_appl.h
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
#ifndef BLE_APPL_H
#define BLE_APPL_H

#include <stdbool.h>
#include <stdint.h>

// BLE application definitions
/* Define message queue sizes (number of messages) */
#define SEND_QUEUE_SIZE 64    // Number of BLE packets that can be queued for sending
#define RECEIVE_QUEUE_SIZE 16 // Number of BLE packets that can be queued for receiving

#define BLE_PCKT_MAX_SIZE 244     // Maximum BLE packet size (must fit within MTU)
#define BLE_PCKT_RECEIVE_SIZE 234 // Size of each received packet in bytes

/**
 * @brief BLE packet structure for variable-size packet support
 * 
 * Allows sending packets of different sizes through the same queue.
 * Used for both EEG (234 bytes) and MIC (131 bytes) packets.
 */
typedef struct {
  uint16_t size;                   // Actual size of data in this packet
  uint8_t data[BLE_PCKT_MAX_SIZE]; // Packet data buffer
} ble_packet_t;

#include "ble/ble_commands.h"

/**
 * @brief BLE NUS received data structure
 * 
 * Holds data received from the BLE Nordic UART Service.
 */
typedef struct {
  bool available;
  int size;
  uint8_t data[BLE_PCKT_RECEIVE_SIZE];
} ble_nus_data_t;

/**
 * @brief UART to GAP9/PULP data structure
 * 
 * Used for forwarding BLE data to the GAP9 processor via UART.
 */
typedef struct {
  bool is_data_available;
  uint8_t data_len;
  uint8_t p_data[250];
} uart_to_pulp_data_t;

extern ble_nus_data_t ble_data_available;
extern struct k_sem ble_data_received;

// Function prototypes
void init_ble_comm(void);
void add_data_to_send_buffer(uint8_t *data, uint16_t size);
void add_data_to_receive_buffer(uint8_t *data);

#endif // BLE_APPL_H