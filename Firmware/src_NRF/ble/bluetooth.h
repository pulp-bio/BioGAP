/*
 * ----------------------------------------------------------------------
 *
 * File: bluetooth.h
 *
 * Last edited: 05.12.2025
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

/**
 * @file bluetooth.h
 * @brief Bluetooth Low Energy (BLE) Communication Interface
 *
 * This module provides high-level BLE functionality for the SENSEI platform:
 * - BLE advertising and connection management
 * - Data transmission via Nordic UART Service (NUS)
 * - Packet statistics for debugging and performance analysis
 * - Connection diagnostics and throughput measurement
 *
 * @section ble_usage Usage Example
 * @code
 * // During system initialization
 * start_bluetooth_adverts();
 *
 * // Wait for connection, then start streaming
 * ble_reset_packet_counters();
 *
 * // During streaming - send sensor data
 * send_data_ble(packet_data, packet_length);
 *
 * // After streaming - check statistics
 * ble_print_packet_stats();
 *
 * // Or get stats programmatically
 * ble_packet_stats_t stats;
 * ble_get_packet_stats(&stats);
 * @endcode
 *
 * @section ble_packet_format Packet Format Detection
 * The module automatically detects packet types based on header bytes:
 * - 0x55: EEG/ExG data packets (192 bytes, without IMU)
 * - 0x56: IMU accelerometer packets (127 bytes, independent 400 Hz stream)
 * - 0xAA: Microphone audio packets (131 bytes)
 *
 * @section ble_debugging Debugging Packet Loss
 * To debug packet loss between firmware and Python receiver:
 * 1. Call ble_reset_packet_counters() before starting streaming
 * 2. Stream data for a known duration
 * 3. Call ble_print_packet_stats() after stopping
 * 4. Compare firmware counts with Python receiver counts
 */

#ifndef BLUETOOTH_H
#define BLUETOOTH_H

#include <stdbool.h>
#include <stdint.h>
#include <zephyr/drivers/sensor.h>

/*==============================================================================
 * Constants
 *============================================================================*/

/**
 * @brief EEG/ExG packet header byte
 *
 * Matches BLE_PCK_HEADER defined in ads_defs.h.
 * Used to identify EEG packets for statistics tracking.
 */
#define BLE_EEG_PACKET_HEADER 0x55

/**
 * @brief Microphone packet header byte
 *
 * Matches MIC_DATA_HEADER defined in mic_appl.h.
 * Used to identify MIC packets for statistics tracking.
 */
#define BLE_MIC_PACKET_HEADER 0xAA

/**
 * @brief IMU packet header byte
 *
 * Matches BLE_IMU_HEADER defined in ads_defs.h.
 * Used to identify IMU packets for statistics tracking.
 * IMU packets are sent independently at native 400 Hz rate.
 */
#define BLE_IMU_PACKET_HEADER 0x56

/*==============================================================================
 * Type Definitions
 *============================================================================*/

/**
 * @brief BLE packet statistics structure
 *
 * Holds counters for tracking packet transmission by sensor type.
 * Useful for debugging packet loss issues between firmware and receiver.
 *
 * @note All counters are reset to zero by ble_reset_packet_counters().
 */
typedef struct {
  uint32_t eeg_sent;   /**< Number of EEG packets successfully sent (header 0x55) */
  uint32_t mic_sent;   /**< Number of MIC packets successfully sent (header 0xAA) */
  uint32_t imu_sent;   /**< Number of IMU packets successfully sent (header 0x56) */
  uint32_t other_sent; /**< Number of other/unknown packets successfully sent */
  uint32_t failed;     /**< Number of packets that failed to send */
} ble_packet_stats_t;

/*==============================================================================
 * Function Declarations - Initialization & Connection
 *============================================================================*/

/**
 * @brief Initialize and start BLE advertising
 *
 * Initializes the Bluetooth stack, configures the Nordic UART Service (NUS),
 * and starts advertising. The device will be discoverable with the name
 * configured in CONFIG_BT_DEVICE_NAME.
 *
 * Connection parameters after successful connection:
 * - Connection interval: 7.5 ms (optimized for throughput)
 * - PHY: 2M (if supported by central device)
 * - Data length: Maximum supported
 * - MTU: Negotiated to maximum
 *
 * @note This function is non-blocking. Connection callbacks will handle
 *       subsequent connection events.
 * @note Call this once during system initialization.
 */
void start_bluetooth_adverts(void);

/**
 * @brief Check if a BLE connection is currently active
 *
 * @return true if a device is connected, false otherwise
 *
 * @note Useful for checking connection status before attempting to send data
 *       or for conditional behavior based on connection state.
 */
bool ble_is_connected(void);

/*==============================================================================
 * Function Declarations - Data Transmission
 *============================================================================*/

/**
 * @brief Send data over BLE connection
 *
 * Transmits a data packet over the active BLE connection using the Nordic
 * UART Service (NUS). Automatically tracks packet statistics based on the
 * packet header byte.
 *
 * Packet type detection (based on first byte):
 * - 0x55 (BLE_EEG_PACKET_HEADER): EEG packet -> increments eeg_sent
 * - 0x56 (BLE_IMU_PACKET_HEADER): IMU packet -> increments imu_sent
 * - 0xAA (BLE_MIC_PACKET_HEADER): MIC packet -> increments mic_sent
 * - Other values: Unknown packet -> increments other_sent
 *
 * @param data_array Pointer to the data buffer to send
 * @param length     Length of data in bytes
 *
 * @warning Maximum payload size depends on negotiated MTU (typically ~244 bytes).
 *          Sending larger packets will fail.
 *
 * @note If transmission fails, the failed counter is incremented and a
 *       warning is logged. Common failure reasons:
 *       - No active connection
 *       - BLE stack buffer full (sending too fast)
 *       - Packet size exceeds MTU
 */
void send_data_ble(char *data_array, int16_t length);

/*==============================================================================
 * Function Declarations - Packet Statistics
 *============================================================================*/

/**
 * @brief Reset all packet counters to zero
 *
 * Clears EEG, MIC, IMU, other, and failed packet counters. Should be called
 * at the start of each streaming session for accurate per-session statistics.
 *
 * @note This is automatically called by the streaming command handlers
 *       (START_EEG_STREAMING, START_COMBINED_STREAMING).
 */
void ble_reset_packet_counters(void);

/**
 * @brief Print packet statistics to log
 *
 * Outputs current packet counts to the logging system at INFO level.
 *
 * Output format:
 * @code
 * [INF] main_bluetooth: BLE packets: EEG=1234, IMU=5000, MIC=5678, other=0, failed=2
 * @endcode
 *
 * @note This is automatically called by the streaming stop command handlers
 *       (STOP_EEG_STREAMING, STOP_COMBINED_STREAMING).
 *
 * @see ble_get_packet_stats() for programmatic access to statistics.
 */
void ble_print_packet_stats(void);

/**
 * @brief Get all packet statistics in a structure
 *
 * Retrieves a snapshot of all packet counters. This is useful for
 * programmatic analysis or when you need multiple counter values
 * atomically.
 *
 * @param[out] stats Pointer to structure to fill with current statistics.
 *                   Must not be NULL.
 *
 * Example usage:
 * @code
 * ble_packet_stats_t stats;
 * ble_get_packet_stats(&stats);
 * LOG_INF("Total sent: %u", stats.eeg_sent + stats.imu_sent + stats.mic_sent);
 * LOG_INF("Loss rate: %.2f%%", 100.0 * stats.failed / (stats.eeg_sent + stats.failed));
 * @endcode
 */
void ble_get_packet_stats(ble_packet_stats_t *stats);

/**
 * @brief Get total number of packets successfully sent
 *
 * @return Sum of EEG, IMU, MIC, and other packets sent
 */
uint32_t ble_get_packets_sent(void);

/**
 * @brief Get number of failed packet transmissions
 *
 * @return Number of packets that failed to send via bt_nus_send()
 *
 * @note A high failure count may indicate:
 *       - Sending data faster than BLE can transmit
 *       - Connection issues or interference
 *       - Buffer overflow in the BLE stack
 */
uint32_t ble_get_packets_failed(void);

/**
 * @brief Get number of EEG packets sent
 *
 * EEG packets are identified by header byte 0x55 (BLE_EEG_PACKET_HEADER).
 *
 * @return Number of EEG packets successfully transmitted
 *
 * @note At 500 SPS with 7 samples per packet, expect ~71 packets/second.
 */
uint32_t ble_get_eeg_packets_sent(void);

/**
 * @brief Get number of MIC packets sent
 *
 * MIC packets are identified by header byte 0xAA (BLE_MIC_PACKET_HEADER).
 *
 * @return Number of MIC packets successfully transmitted
 *
 * @note At 16 kHz with 64 samples per packet, expect 250 packets/second.
 */
uint32_t ble_get_mic_packets_sent(void);

/**
 * @brief Get number of IMU packets sent
 *
 * IMU packets are identified by header byte 0x56 (BLE_IMU_PACKET_HEADER).
 *
 * @return Number of IMU packets successfully transmitted
 *
 * @note At 400 Hz with 20 samples per packet, expect 20 packets/second.
 */
uint32_t ble_get_imu_packets_sent(void);

/*==============================================================================
 * Function Declarations - Diagnostics
 *============================================================================*/

/**
 * @brief Print BLE connection information
 *
 * Logs detailed connection parameters to help debug throughput issues:
 * - TX/RX maximum data length (bytes)
 * - TX/RX maximum time (microseconds)
 * - Connection interval (in 1.25ms units)
 * - Connection latency
 * - Supervision timeout (in 10ms units)
 *
 * Example output:
 * @code
 * [INF] main_bluetooth: TX max length: 251 bytes
 * [INF] main_bluetooth: TX max time: 2120 us
 * [INF] main_bluetooth: Conn. interval is 6 units
 * @endcode
 *
 * @note Only valid when a connection is active.
 */
void print_ble_conn_info(void);

/**
 * @brief Measure and report BLE throughput
 *
 * Performs a throughput test by sending 100 test packets (240 bytes each)
 * as fast as possible and measuring the time taken. Reports:
 * - Time taken for all transmissions
 * - Total bytes sent
 * - Throughput in Mbit/s
 *
 * @warning This is a blocking test function that sends data continuously.
 *          Do not call during normal sensor streaming operation.
 *
 * @note Useful for verifying that BLE connection parameters are optimal
 *       and for baseline throughput measurements.
 */
void measure_throughput(void);

#endif /* BLUETOOTH_H */
