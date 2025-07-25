/*
 * ----------------------------------------------------------------------
 *
 * File: lis2duxs12_sensor.h
 *
 * Last edited: 24.03.2025
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

#ifndef LIS2DUXS12_SENSOR_H
#define LIS2DUXS12_SENSOR_H

#include "lis2duxs12_reg.h"

void init_lis2duxs12(void);
void enable_acc_sampling(void);
void test_lis2duxs12();

int lis2duxs12_enable_double_tap(void);


extern lis2duxs12_xl_data_t data_xl;
extern lis2duxs12_outt_data_t data_temp;

#endif // LIS2DUXS12_SENSOR_H