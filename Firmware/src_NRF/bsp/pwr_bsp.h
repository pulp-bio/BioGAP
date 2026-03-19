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

extern bool flag_isr_soft_reset;

#endif /* PWR_BSP_H_ */
