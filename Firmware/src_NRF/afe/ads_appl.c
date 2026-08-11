/*
 * ----------------------------------------------------------------------
 *
 * File: ads_appl.c
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
 * @file ads_appl.c
 * @brief ADS1298 Application Layer Implementation
 *
 * Implements the application-level state machine and control logic for the
 * ADS1298 bio-potential AFE. This module acts as a bridge between the BLE
 * command interface and the low-level SPI driver.
 */

#include "afe/ads_appl.h"

#include <zephyr/logging/log.h>
#include <zephyr/logging/log_ctrl.h>

/* Initialize the logging module */
LOG_MODULE_REGISTER(ads_appl, LOG_LEVEL_INF);

/*==============================================================================
 * Module Variables
 *============================================================================*/

/**
 * @brief Current application state
 *
 * Initialized to ADS_STILL (idle) state. Modified by BLE commands and error handlers.
 */
ads_function_t ads_function = ADS_STILL;

/*==============================================================================
 * Public Functions
 *============================================================================*/

/**
 * @brief Get the current ADS application function state
 *
 * Thread-safe getter for the global application state.
 *
 * @return Current ads_function_t state
 */
ads_function_t ads_get_function(void) { return ads_function; }

/**
 * @brief Set the ADS application function state
 *
 * Updates the global state machine. This function is called from:
 * - BLE command handlers to change operating mode
 * - Error handlers to stop acquisition
 * - Main application logic for state transitions
 *
 * @param f New function state to set
 *
 * @note This function does not perform the state transition itself,
 *       it only updates the state variable. The actual transition is
 *       handled by the main loop or interrupt handlers.
 */
void ads_set_function(ads_function_t f) { 
    LOG_DBG("Setting ADS function to %d", f);
    ads_function = f; 
}

