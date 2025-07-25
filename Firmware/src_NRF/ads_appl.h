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
#ifndef ADS_APPL_H
#define ADS_APPL_H

#include <stdint.h>
#include <stdbool.h>



// BLE application definitions

enum ADS_function_t {READ, START, STOP, STILL, INIT_GAP9_CTRL, WOLF_CTRL, CONNECT, RESTART_WOLF, TOGGLE_DRDY, READ_BATTERY, PROGRAM_WOLF, ES_QUALITY};
enum ADS_id_t {ADS1298_A, ADS1298_B};
extern enum ADS_function_t ADS_function;



enum ADS_function_t Get_ADS_Function();
void Set_ADS_Function(enum ADS_function_t f);
void set_trigger(uint8_t value);
uint8_t get_trigger();

#endif // ADS_APPL_H