/*
 * ----------------------------------------------------------------------
 *
 * File: system_status.h
 *
 * Last edited: 05.01.2026
 *
 * Copyright (C) 2026, ETH Zurich and University of Bologna.
 *
 * ----------------------------------------------------------------------
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef SYSTEM_STATUS_H_
#define SYSTEM_STATUS_H_

#include <stdint.h>
#include <stddef.h>

/**
 * @brief Initialize the system status aggregator.
 * 
 * @return 0 on success, negative on error.
 */
int system_status_init(void);

/**
 * @brief Build a BLE response packet for the system status.
 * 
 * Aggregates battery and sensor information into a format suitable for BLE transmission.
 * 
 * @param buffer Pointer to the buffer where the packet will be built.
 * @param buf_size Size of the buffer.
 * @param out_len Pointer to store the resulting packet length.
 * @return 0 on success, negative on error (e.g., buffer too small).
 */
int system_status_build_ble_packet(uint8_t *buffer, size_t buf_size, size_t *out_len);

/*==============================================================================
 * Device Information Functions
 *============================================================================*/

/**
 * @brief Send hardware version over BLE.
 * 
 * Packet format: [REQUEST_HARDWARE_VERSION, HARDWARE_VERSION, HARDWARE_REVISION, BLE_PCK_TAILER]
 */
void system_status_send_hardware_version(void);

/**
 * @brief Send firmware version over BLE.
 * 
 * Packet format: [REQUEST_FIRMWARE_VERSION, FIRMWARE_VERSION, FIRMWARE_REVISION, BLE_PCK_TAILER]
 */
void system_status_send_firmware_version(void);

/**
 * @brief Send available sensors over BLE.
 * 
 * Packet format: [REQUEST_AVAILABLE_SENSORS, available_flag, reserved, BLE_PCK_TAILER]
 */
void system_status_send_available_sensors(void);

/**
 * @brief Send device settings over BLE.
 * 
 * Currently not implemented.
 */
void system_status_send_device_settings(void);

/**
 * @brief Send ready/connection confirmation string over BLE.
 * 
 * Sends "BWF16" to confirm successful connection to the client.
 */
void system_status_send_ready(void);

/*==============================================================================
 * Board State Management
 *============================================================================*/

/**
 * @brief Set the board operating state.
 * 
 * @param state The new state (e.g., STATE_STREAMING_NORDIC, STATE_GAP9_MASTER)
 */
void system_status_set_board_state(int8_t state);

/**
 * @brief Get the current board operating state.
 * 
 * @return Current board state
 */
int8_t system_status_get_board_state(void);

#endif /* SYSTEM_STATUS_H_ */

