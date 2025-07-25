/*
 * ----------------------------------------------------------------------
 *
 * File: pwr_bsp.h
 *
 * Last edited: 19.06.2024
 *
 * Copyright (C) 2024, ETH Zurich and University of Bologna.
 *
 * Authors:
 * - Philipp Schilk (schilkp@ethz.ch), ETH Zurich
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
#ifndef PWR_BSP_H_
#define PWR_BSP_H_

#include <stdint.h>   // Defines uint32_t, uint8_t, etc.
#include <stdbool.h>  // Defines bool

/**
 * @brief Initialise all the pwr hardware interface.
 * @return negative on error, 0 otherwise
 */
int pwr_bsp_init();

/**
 * @brief Configure all the pwr switches.
 * @return negative on error, 0 otherwise
 */
int pwr_bsp_start();
int pwr_charge_enable();

int pwr_ads_on_unipolar();
int pwr_ads_off();

int pwr_ads_on_bipolar();

int pwr_update_battery_status();

int pwr_get_full_status(uint32_t *soc, uint32_t *bat_mv, uint32_t *power_mw, bool *charging, const char **power_source);

// Returns the battery state-of-charge in percent (0–100).
uint32_t bsp_get_battery_soc(void);

// Returns the battery voltage in millivolts.
uint32_t bsp_get_battery_voltage(void);

// Returns the total system power consumption in milliwatts.
uint32_t bsp_get_total_power_mw(void);

// Returns 1 if the battery is being charged (i.e., external power is active), 0 otherwise.
uint8_t bsp_is_charging(void);

// Returns a string representing the current power source ("USB/External" or "Battery").
const char* bsp_get_power_source(void);

#endif /* PWR_BSP_H_ */
