/*
 * ----------------------------------------------------------------------
 *
 * File: lis2duxs12_sensor.h
 *
 * Last edited: 08.12.2025
 *
 * Copyright (C) 2025, ETH Zurich and University of Bologna.
 *
 * Authors:
 * - Philip Wiese (wiesep@iis.ee.ethz.ch), ETH Zurich
 * - Sebastian Frey (sefrey@iis.ee.ethz.ch), ETH Zurich
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
 * @file lis2duxs12_sensor.h
 * @brief LIS2DUXS12 Low-Level Driver Interface
 *
 * This module provides low-level driver functions for the LIS2DUXS12
 * 3-axis accelerometer. It handles:
 * - Sensor initialization and configuration
 * - Data acquisition via interrupt-driven reading
 * - Temperature sensor reading
 * - Tap detection (optional feature)
 *
 * For high-level streaming control, use the imu_appl.h API instead.
 */

#ifndef LIS2DUXS12_SENSOR_H
#define LIS2DUXS12_SENSOR_H

#include <stdint.h>

#include "sensors/imu/driver/lis2duxs12_reg.h"

/*==============================================================================
 * Initialization & Configuration
 *============================================================================*/

/**
 * @brief Initialize the LIS2DUXS12 sensor
 *
 * Performs full sensor initialization:
 * - Configures GPIO interrupt pin
 * - Initializes I2C communication
 * - Wakes sensor from deep power down
 * - Reads and verifies device ID
 * - Resets sensor to default configuration
 *
 * @return 0 on success, negative error code on failure
 */
int lis2duxs12_init(void);

/**
 * @brief Start accelerometer sampling at 400 Hz
 *
 * Configures the sensor for continuous sampling:
 * - Sets ODR to 400 Hz low-power mode
 * - Configures full-scale to +/-8g
 * - Enables DRDY (data ready) interrupt on INT1
 *
 * After calling this function, use lis2duxs12_wait_data_ready() to
 * wait for new samples.
 *
 * @return 0 on success, negative error code on failure
 */
int lis2duxs12_start_sampling(void);

/**
 * @brief Stop accelerometer sampling
 *
 * Puts the sensor into power-down mode to conserve energy.
 *
 * @return 0 on success, negative error code on failure
 */
int lis2duxs12_stop_sampling(void);

/*==============================================================================
 * Data Reading
 *============================================================================*/

/**
 * @brief Wait for data ready interrupt
 *
 * Blocks until the sensor signals that new accelerometer data is available,
 * or until the timeout expires.
 *
 * @param timeout_ms Maximum time to wait in milliseconds
 *                   - 0: Return immediately if no data ready
 *                   - >0: Wait up to timeout_ms for data
 *
 * @return 0 on success (data ready)
 * @return -EAGAIN on timeout
 * @return negative error code on other failures
 */
int lis2duxs12_wait_data_ready(uint32_t timeout_ms);

/**
 * @brief Read raw accelerometer data
 *
 * Reads the latest accelerometer sample from the sensor.
 * Should be called after lis2duxs12_wait_data_ready() returns 0.
 *
 * @param[out] x Pointer to store X-axis acceleration (raw int16_t)
 * @param[out] y Pointer to store Y-axis acceleration (raw int16_t)
 * @param[out] z Pointer to store Z-axis acceleration (raw int16_t)
 *
 * @return 0 on success, negative error code on failure
 */
int lis2duxs12_read_accel(int16_t *x, int16_t *y, int16_t *z);

/**
 * @brief Read temperature from on-chip sensor
 *
 * Reads the temperature from the LIS2DUXS12's internal temperature sensor.
 *
 * @param[out] temp_celsius Pointer to store temperature in degrees Celsius
 *
 * @return 0 on success, negative error code on failure
 */
int lis2duxs12_read_temperature(float *temp_celsius);

/*==============================================================================
 * Tap Detection (Optional Feature)
 *============================================================================*/

/**
 * @brief Enable double-tap detection
 *
 * Configures the sensor to detect double-tap events and generate
 * an interrupt on the INT1 pin.
 *
 * @note This changes the interrupt configuration. If you need both
 *       continuous accelerometer data and tap detection, additional
 *       configuration may be required.
 *
 * @return 0 on success, negative error code on failure
 */
int lis2duxs12_enable_double_tap(void);

/**
 * @brief Disable tap detection
 *
 * Disables tap detection and restores normal accelerometer operation.
 *
 * @return 0 on success, negative error code on failure
 */
int lis2duxs12_disable_tap(void);

#endif // LIS2DUXS12_SENSOR_H
