/*
 * ----------------------------------------------------------------------
 *
 * File: lsm6dsv16bx_sensor.h
 *
 * Last edited: 08.07.2026
 *
 * Copyright (C) 2026, ETH Zurich and University of Bologna.
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
 * @file lsm6dsv16bx_sensor.h
 * @brief LSM6DSV16BX Low-Level Driver Interface
 *
 * This module provides low-level driver functions for the LSM6DSV16BX
 * 6-axis IMU (3-axis accelerometer + 3-axis gyroscope). It handles:
 * - Sensor initialization and configuration
 * - Data acquisition via interrupt-driven reading (DRDY on INT1)
 * - Temperature sensor reading
 *
 * Hardware (BioGAP-Ultra mainboard):
 * - Bus: I2C_A (i2ca alias), 7-bit address 0x6A (SA0 tied to GND)
 * - INT1: nRF P0.23 (devicetree node gpio_imu_int1)
 * - INT2, QVAR and the TDM audio channel are not wired to the nRF5340
 *
 * For high-level streaming control, use the imu_appl.h API instead.
 */

#ifndef LSM6DSV16BX_SENSOR_H
#define LSM6DSV16BX_SENSOR_H

#include <stdint.h>

#include "sensors/imu/driver/lsm6dsv16bx_reg.h"

/*==============================================================================
 * Configuration
 *============================================================================*/

/** @brief 7-bit I2C address (SDO/SA0 pin tied to GND on the mainboard) */
#define LSM6DSV16BX_I2C_ADDR 0x6A

/*==============================================================================
 * Initialization & Configuration
 *============================================================================*/

/**
 * @brief Initialize the LSM6DSV16BX sensor
 *
 * Performs full sensor initialization:
 * - Configures the INT1 GPIO interrupt pin
 * - Initializes I2C communication
 * - Reads and verifies the device ID (WHO_AM_I = 0x71)
 * - Resets the sensor to its default configuration
 *
 * @return 0 on success, negative error code on failure
 */
int lsm6dsv16bx_sensor_init(void);

/**
 * @brief Start accelerometer + gyroscope sampling at 480 Hz
 *
 * Configures the sensor for continuous 6-axis sampling:
 * - Enables block data update (BDU)
 * - Sets accelerometer full scale to +/-8 g
 * - Sets gyroscope full scale to +/-2000 dps
 * - Enables the pulsed accelerometer DRDY interrupt on INT1
 * - Sets both ODRs to 480 Hz (high-performance mode)
 *
 * After calling this function, use lsm6dsv16bx_wait_data_ready() to
 * wait for new samples.
 *
 * @return 0 on success, negative error code on failure
 */
int lsm6dsv16bx_start_sampling(void);

/**
 * @brief Stop accelerometer + gyroscope sampling
 *
 * Puts both sensors into power-down mode to conserve energy.
 *
 * @return 0 on success, negative error code on failure
 */
int lsm6dsv16bx_stop_sampling(void);

/*==============================================================================
 * Data Reading
 *============================================================================*/

/**
 * @brief Wait for data ready interrupt
 *
 * Blocks until the sensor signals new accelerometer data on INT1
 * (the gyroscope runs at the same ODR, so its sample is read together
 * with the accelerometer one), or until the timeout expires.
 *
 * @param timeout_ms Maximum time to wait in milliseconds
 *                   - 0: Return immediately if no data ready
 *                   - >0: Wait up to timeout_ms for data
 *
 * @return 0 on success (data ready)
 * @return -EAGAIN on timeout
 * @return negative error code on other failures
 */
int lsm6dsv16bx_wait_data_ready(uint32_t timeout_ms);

/**
 * @brief Read one raw 6-axis sample
 *
 * Reads the latest accelerometer and gyroscope sample from the sensor.
 * Should be called after lsm6dsv16bx_wait_data_ready() returns 0.
 *
 * @param[out] accel Array of 3 int16_t for acceleration X, Y, Z
 *                   (+/-8 g full scale, 0.244 mg/LSB)
 * @param[out] gyro  Array of 3 int16_t for angular rate X, Y, Z
 *                   (+/-2000 dps full scale, 70 mdps/LSB)
 *
 * @return 0 on success, negative error code on failure
 */
int lsm6dsv16bx_read_accel_gyro(int16_t accel[3], int16_t gyro[3]);

/**
 * @brief Read temperature from on-chip sensor
 *
 * Reads the temperature from the LSM6DSV16BX's internal temperature
 * sensor. The value is only refreshed while the accelerometer is
 * running; in power-down the last converted value is returned.
 *
 * @param[out] temp_celsius Pointer to store temperature in degrees Celsius
 *
 * @return 0 on success, negative error code on failure
 */
int lsm6dsv16bx_read_temperature(float *temp_celsius);

#endif /* LSM6DSV16BX_SENSOR_H */
