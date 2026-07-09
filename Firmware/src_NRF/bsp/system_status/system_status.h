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

/** Total length of the extended system-status response (command 43). */
#define SYSTEM_STATUS_PACKET_LEN 21

/** Bits of the flags byte (byte 1) of the extended system-status response. */
#define SYSTEM_STATUS_FLAG_CHGIN_PRESENT (1U << 0) /**< External power (USB) present */
#define SYSTEM_STATUS_FLAG_CHARGING (1U << 1)      /**< Charger actively charging */
#define SYSTEM_STATUS_FLAG_CHG_FAULT (1U << 2)     /**< Charger timer/temperature fault */
#define SYSTEM_STATUS_FLAG_THERMAL_ALARM (1U << 3) /**< PMIC junction thermal alarm */
#define SYSTEM_STATUS_FLAG_TEMP_VALID (1U << 4)    /**< Temperature field is valid */
#define SYSTEM_STATUS_FLAG_BATT_PRESENT (1U << 5)  /**< A battery is attached */

/**
 * @brief Initialize the system status aggregator.
 *
 * @return 0 on success, negative on error.
 */
int system_status_init(void);

/**
 * @brief Build the legacy 7-byte battery/status BLE response (command 17).
 *
 * Byte layout is kept byte-for-byte compatible with the original BioGAP GUI.
 * All values come from the SDK power-thread cache - no PMIC/I2C access.
 *
 * @param buffer Pointer to the buffer where the packet will be built.
 * @param buf_size Size of the buffer.
 * @param out_len Pointer to store the resulting packet length.
 * @return 0 on success, negative on error (e.g., buffer too small).
 */
int system_status_build_ble_packet(uint8_t *buffer, size_t buf_size, size_t *out_len);

/**
 * @brief Build the extended system-status BLE response (command 43).
 *
 * Full battery/charger telemetry (voltages, currents, powers, charger state
 * machine) plus the IMU die temperature. See BLE_PACKET_STRUCTURE.md for the
 * byte layout. All values come from the SDK power-thread cache.
 *
 * @param buffer Pointer to the buffer where the packet will be built.
 * @param buf_size Size of the buffer (>= SYSTEM_STATUS_PACKET_LEN).
 * @param out_len Pointer to store the resulting packet length.
 * @return 0 on success, negative on error (e.g., buffer too small).
 */
int system_status_build_system_status_packet(uint8_t *buffer, size_t buf_size, size_t *out_len);

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

