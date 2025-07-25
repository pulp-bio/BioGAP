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
#include "ads_appl.h"
#include "ads_spi.h"
#include "ble_appl.h"
#include "common.h"

#include "pwr/pwr.h"
#include "bsp/pwr_bsp.h"
#include "pwr/pwr_common.h"

#include <zephyr/logging/log.h>
#include <zephyr/logging/log_ctrl.h>

/* Initialize the logging module */
LOG_MODULE_REGISTER(ads_appl, LOG_LEVEL_DBG);


enum ADS_function_t ADS_function = STILL;

static int trigger_value = 0x00;
//uint8_t InitParams[5]={2,1,0,0,0};  //SAMPLE RATE 1KSPS, CHANNEL FUNCTION SHORT, NC, NC, GAIN = 6


enum ADS_function_t Get_ADS_Function()
{
  return(ADS_function);
}

void Set_ADS_Function(enum ADS_function_t f)
{
  ADS_function=f;
}



void set_trigger(uint8_t value)
{
  trigger_value = value;

}

uint8_t get_trigger()
{
  return trigger_value;

}
