/*
 * ----------------------------------------------------------------------
 *
 * File: ads_appl.h
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

/**
 * @file ads_appl.h
 * @brief ADS1298 Application Layer Interface
 *
 * This module provides high-level application control for the ADS1298 bio-potential
 * analog front-end (AFE) devices. It manages the application state machine
 * for EEG/EMG/ExG data acquisition.
 */

#ifndef ADS_APPL_H
#define ADS_APPL_H

#include <stdbool.h>
#include <stdint.h>

/*==============================================================================
 * Type Definitions
 *============================================================================*/

/**
 * @brief Application state machine functions for ADS1298 control
 *
 * Defines the various operational states and commands that control the
 * behavior of the ADS1298 devices and the overall system.
 */
typedef enum {
  ADS_READ,           /**< Continuous data reading mode */
  ADS_START,          /**< Start data acquisition */
  ADS_STOP,           /**< Stop data acquisition */
  ADS_STILL,          /**< Idle state, no active operation */
  ADS_INIT_GAP9_CTRL, /**< Initialize GAP9 processor control */
  ADS_WOLF_CTRL,      /**< Biowolf system control */
  ADS_CONNECT,        /**< Establish connection */
  ADS_RESTART_WOLF,   /**< Restart Biowolf system */
  ADS_TOGGLE_DRDY,    /**< Toggle data ready pin monitoring */
  ADS_READ_BATTERY,   /**< Read battery status */
  ADS_PROGRAM_WOLF,   /**< Program Biowolf firmware */
  ADS_ES_QUALITY      /**< Electrode-skin quality measurement */
} ads_function_t;

/**
 * @brief ADS1298 device identifier
 *
 * Used to distinguish between the two ADS1298 chips in the dual-AFE configuration.
 */
typedef enum { ADS1298_A, ADS1298_B } ads_device_id_t;

/*==============================================================================
 * Global Variables
 *============================================================================*/

/**
 * @brief Current application state for ADS1298 control
 *
 * This variable tracks the current operational state of the system.
 * It is used by various modules to coordinate data acquisition and processing.
 */
extern ads_function_t ads_function;

/*==============================================================================
 * Function Declarations
 *============================================================================*/

/**
 * @brief Get the current ADS application function state
 *
 * @return Current ads_function_t state
 */
ads_function_t ads_get_function(void);

/**
 * @brief Set the ADS application function state
 *
 * Updates the global state machine to control system behavior.
 * This function is typically called from BLE command handlers or
 * error handling routines.
 *
 * @param f New function state to set
 */
void ads_set_function(ads_function_t f);

#endif // ADS_APPL_H