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
 * This module provides high-level application control for the LIS2DUXS12 IMU
 * sensor. It manages accelerometer data acquisition and streaming over BLE.
 *
 * The IMU streams at 400 Hz (native sensor ODR) and packages 20 samples
 * per BLE packet, resulting in ~20 packets/second.
 */

#ifndef IMU_APPL_H
#define IMU_APPL_H

#include <stdbool.h>
#include <stdint.h>

/*==============================================================================
 * Configuration
 *============================================================================*/

/** @brief IMU sample rate in Hz (LIS2DUXS12 ODR setting) */
#define IMU_SAMPLE_RATE 400

/** 
 * @brief Number of IMU samples per BLE packet
 * 
 * At 400 Hz ODR, 20 samples = 50ms of data, resulting in ~20 packets/second.
 */
#define IMU_SAMPLES_PER_PACKET 20

/** @brief Bytes per IMU sample (X, Y, Z - each int16_t) */
#define IMU_BYTES_PER_SAMPLE 6

/** @brief BLE packet header byte for IMU data */
#define IMU_DATA_HEADER 0x56

/** @brief BLE packet trailer byte for IMU data */
#define IMU_DATA_TRAILER 0x57

/**
 * @brief Total IMU BLE packet size in bytes
 *
 * Packet structure (127 bytes total):
 * - 1 byte: Header (0x56)
 * - 1 byte: Packet counter
 * - 4 bytes: Timestamp (microseconds, for cross-packet synchronization)
 * - 120 bytes: 20 samples x 6 bytes per sample
 *   - 2 bytes: Acceleration X (int16_t, big-endian)
 *   - 2 bytes: Acceleration Y (int16_t, big-endian)
 *   - 2 bytes: Acceleration Z (int16_t, big-endian)
 * - 1 byte: Trailer (0x57)
 */
#define IMU_PCKT_SIZE (1 + 1 + 4 + (IMU_SAMPLES_PER_PACKET * IMU_BYTES_PER_SAMPLE) + 1)

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
 * Initializes the LIS2DUXS12 sensor and prepares for streaming.
 * Must be called before imu_start_streaming().
 *
 * @return 0 on success, negative error code on failure
 */
int imu_init(void);

/**
 * @brief Start IMU streaming
 *
 * Begins accelerometer data acquisition and streaming over BLE.
 * The IMU will stream at 400 Hz, packaged into 20-sample BLE packets.
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
 * Stops accelerometer data acquisition and streaming.
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
 * Reads the on-chip temperature sensor of the LIS2DUXS12.
 * Can be called while streaming or when idle.
 *
 * @param[out] temp_celsius Pointer to store temperature in degrees Celsius
 * @return 0 on success, negative error code on failure
 */
int imu_read_temperature(float *temp_celsius);

#endif // IMU_APPL_H
