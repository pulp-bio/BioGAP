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

#include <stdbool.h> // Defines bool
#include <stdint.h>  // Defines uint32_t, uint8_t, etc.

#include "max77654.h"

/**
 * @brief Initialise all the pwr hardware interface.
 * @return negative on error, 0 otherwise
 */
int pwr_bsp_init();

/**
 * @brief Apply the current pmic_h.conf settings of one SBB rail to the PMIC.
 *
 * Serialized against the other PMIC users via pwr_mutex, verifies the
 * driver return code and retries once. Use these instead of a bare
 * max77654_config() for rail switching: a silently failed I2C transaction
 * otherwise leaves the rail in the wrong state.
 *
 * @param sbb  SBB rail (MAX77654_SBB0/1/2)
 * @param name Rail name for log messages (e.g. "VD0 5V")
 * @return 0 on success, -EIO if the config failed after retry
 */
int pwr_rail_config_sbb(max77654_sbb_t sbb, const char *name);

/** @brief LDO variant of pwr_rail_config_sbb(). */
int pwr_rail_config_ldo(max77654_ldo_t ldo, const char *name);

/**
 * @brief Configure all the pwr switches.
 * @return negative on error, 0 otherwise
 */
int pwr_bsp_start();
int pwr_charge_enable();


/**
 * @brief Power on WULPUS shield.
 * 
 * @return 0 on success, negative on error.
 */
int wulpus_power_on(void);


extern bool flag_isr_soft_reset;

#endif /* PWR_BSP_H_ */
