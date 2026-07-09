/*
 * ----------------------------------------------------------------------
 *
 * File: imu_appl.h
 *
 * Last edited: 08.12.2025
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
 * @file imu_appl.h
 * @brief IMU Application Layer Interface
 *
 * This module provides high-level application control for the LSM6DSV16BX
 * 6-axis IMU (3-axis accelerometer + 3-axis gyroscope). It manages data
 * acquisition and streaming over BLE.
 *
 * The IMU streams at 480 Hz (native sensor ODR) and packages 19 samples
 * per BLE packet, resulting in ~25 packets/second.
 */

#ifndef IMU_APPL_H
#define IMU_APPL_H

#include <stdbool.h>
#include <stdint.h>

/*==============================================================================
 * Configuration
 *============================================================================*/

/** @brief IMU sample rate in Hz (LSM6DSV16BX ODR setting, accel + gyro) */
#define IMU_SAMPLE_RATE 480

/**
 * @brief Number of IMU samples per BLE packet
 *
 * At 480 Hz ODR, 19 samples = ~39.6 ms of data, resulting in ~25 packets/second.
 * 19 samples is the maximum that fits the 244-byte BLE payload limit.
 */
#define IMU_SAMPLES_PER_PACKET 19

/** @brief Bytes per IMU sample (accel X/Y/Z + gyro X/Y/Z - each int16_t) */
#define IMU_BYTES_PER_SAMPLE 12

/** @brief BLE packet header byte for IMU data */
#define IMU_DATA_HEADER 0x56

/** @brief BLE packet trailer byte for IMU data */
#define IMU_DATA_TRAILER 0x57

/**
 * @brief Total IMU BLE packet size in bytes
 *
 * Packet structure (236 bytes total). The header/counter/timestamp prefix is
 * harmonized with the ExG/MIC/PPG packets:
 * - 1 byte: Header (0x56)
 * - 2 bytes: Packet counter (uint16, little-endian)
 * - 4 bytes: Timestamp (microseconds, for cross-packet synchronization)
 * - 228 bytes: 19 samples x 12 bytes per sample
 *   - 2 bytes: Acceleration X (int16_t, big-endian)
 *   - 2 bytes: Acceleration Y (int16_t, big-endian)
 *   - 2 bytes: Acceleration Z (int16_t, big-endian)
 *   - 2 bytes: Angular rate X (int16_t, big-endian)
 *   - 2 bytes: Angular rate Y (int16_t, big-endian)
 *   - 2 bytes: Angular rate Z (int16_t, big-endian)
 * - 1 byte: Trailer (0x57)
 */
#define IMU_PCKT_SIZE (1 + 2 + 4 + (IMU_SAMPLES_PER_PACKET * IMU_BYTES_PER_SAMPLE) + 1)

/** @brief Timeout in ms for reading IMU data */
#define IMU_READ_TIMEOUT 100

/*==============================================================================
 * Type Definitions
 *============================================================================*/

/**
 * @enum imu_state_t
 * @brief IMU streaming states
 */
typedef enum {
    IMU_STATE_IDLE,      /**< IMU idle, not streaming */
    IMU_STATE_STARTING,  /**< IMU initializing and configuring */
    IMU_STATE_STREAMING, /**< IMU actively streaming data */
    IMU_STATE_STOPPING,  /**< IMU stopping */
    IMU_STATE_ERROR      /**< Error state */
} imu_state_t;

/*==============================================================================
 * Function Declarations
 *============================================================================*/

/**
 * @brief Initialize the IMU subsystem
 *
 * Initializes the LSM6DSV16BX sensor and prepares for streaming.
 * Must be called before imu_start_streaming().
 *
 * @return 0 on success, negative error code on failure
 */
int imu_init(void);

/**
 * @brief Start IMU streaming
 *
 * Begins accelerometer + gyroscope data acquisition and streaming over BLE.
 * The IMU will stream at 480 Hz, packaged into 19-sample BLE packets.
 *
 * If synchronized streaming is active (sync_is_active() returns true),
 * the IMU will wait at the sync barrier before starting data capture.
 *
 * @return 0 on success, negative error code on failure
 * @return -EALREADY if already streaming
 * @return -EBUSY if not in idle state
 */
int imu_start_streaming(void);

/**
 * @brief Stop IMU streaming
 *
 * Stops accelerometer + gyroscope data acquisition and streaming.
 *
 * @return 0 on success, negative error code on failure
 * @return -EINVAL if not currently streaming
 * @return -ETIMEDOUT if stop operation timed out
 */
int imu_stop_streaming(void);

/**
 * @brief Get current IMU state
 *
 * @return Current imu_state_t value
 */
imu_state_t imu_get_state(void);

/**
 * @brief Check if IMU is currently streaming
 *
 * @return true if streaming, false otherwise
 */
bool imu_is_streaming(void);

/**
 * @brief Read temperature from IMU sensor
 *
 * Reads the on-chip temperature sensor of the LSM6DSV16BX.
 * Can be called while streaming or when idle (the value is only
 * refreshed while the sensor is sampling).
 *
 * @param[out] temp_celsius Pointer to store temperature in degrees Celsius
 * @return 0 on success, negative error code on failure
 */
int imu_read_temperature(float *temp_celsius);

#endif // IMU_APPL_H
