/*
 * ----------------------------------------------------------------------
 *
 * File: common.h
 *
 * Last edited: 5.07.2024
 *
 * Copyright (C) 2024, ETH Zurich and University of Bologna.
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

 #include <stdint.h>

 
/*Board States*/
#define STATE_STREAMING_NORDIC          50
#define STATE_GAP9_MASTER               60
#define STATE_SWITCH                    70
#define STATE_PROGRAM_WOLF              80
#define STATE_ES_QUALITY                90

#define FIRMWARE_VERSION  '2'
#define FIRMWARE_REVISION 'c'

#define HARDWARE_VERSION  '2'
#define HARDWARE_REVISION 'b'

#define SPACES "                                                               "

#define NO_ERROR 0


// List of states
typedef enum {
    S_SHUTDOWN,
    S_DEEPSLEEP,
    S_LOW_POWER_CONNECTED,
    S_NORDIC_STREAM,
    S_GAP_CTRL,
    S_MAX_STATES
  } State_t;

void set_SM_state(State_t new_state);
State_t get_SM_state(void);

void start_bluetooth_adverts(void);
void send_data_ble(char *data_array, int16_t length);

void update_status(struct sensor_value *temp, struct sensor_value *press, struct sensor_value *humidity,
                   struct sensor_value *gas_res);


