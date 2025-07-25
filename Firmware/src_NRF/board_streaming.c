/*
 * ----------------------------------------------------------------------
 *
 * File: board_streaming.c.h
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
#include <stdio.h>
#include <string.h>

#include "pwr/pwr.h"
#include "bsp/pwr_bsp.h"
#include "pwr/pwr_common.h"
#include "pwr/thread_pwr.h"

#include "common.h"
#include "ble_appl.h"
#include "ads_appl.h"
#include "ads_spi.h"

#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>

bool first_run=true;

uint8_t InitParams[5]={2,1,0,0,0};  //SAMPLE RATE 1KSPS, CHANNEL FUNCTION SHORT, NC, NC, GAIN = 6

void loop_streaming()
{
  switch(Get_ADS_Function())
  {
    case START:
      if(first_run)
      {
        //set_trigger(0);               // Reset the trigger.
        GetConfigParam(InitParams);   // Send "Ready" to host and gets configuration parameters
        //ADS_SPI_Config();             //Configure SPI as master  
        //ADS_drdy_init();
        //Analog_OFF();
        k_msleep(200);
        //Analog_ON();                  // Turn on analog section
        ADS_check_ID(ADS1298_A);
        ADS_check_ID(ADS1298_B);
        ADS_Init(InitParams, ADS1298_A);         // Initialize ADS
        ADS_Init(InitParams, ADS1298_B);         // Initialize ADS
        ADS_Start();                  // Start ADS
        Set_ADS_Function(READ);
        first_run = false;

      }
        else
        {
          //set_trigger(0);               // Reset the trigger.
          GetConfigParam(InitParams);   // Send "Ready" to host and gets configuration parameters 
          //Analog_OFF();
          k_msleep(200);
          //Analog_ON();                  // Turn on analog section
          ADS_Init(InitParams, ADS1298_A);         // Initialize ADS
          ADS_Init(InitParams, ADS1298_B);         // Initialize ADS
          ADS_Start();                  // Start ADS
          Set_ADS_Function(READ);
          
        }

    break;

   case STOP:
    if(!first_run)
    {
      Set_ADS_Function(STILL);
      ADS_Stop();                         //Stop ADS
      k_msleep(100);
      //Analog_OFF();                       //Power Down Analog
      //if(get_es_state()==ESQ_RUNNING)
      //  set_es_state(ESQ_DISABLE_HARDWARE);
    }
    break;

  case READ:
    process_ads_data();
    break;
      
  case STILL:
    set_trigger(0);
    break;

   default:
      break;
  }
}

