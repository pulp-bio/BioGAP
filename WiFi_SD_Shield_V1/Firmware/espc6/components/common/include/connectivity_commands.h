/*
 * ----------------------------------------------------------------------
 *
 * File: connectivity_commands.h
 *
 * Last edited: 22.05.2026
 *
 * Copyright (C) 2026, ETH Zurich
 *
 * Authors:
 * - Giusy Spacone (gspacone@iis.ee.ethz.ch), ETH Zurich
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

#ifndef CONNECTIVITY_COMMANDS_H
#define CONNECTIVITY_COMMANDS_H

/**
 * @file connectivity_commands.h
 * @brief Connectivity Protocol Command Definitions
 *
 * This header defines the command codes used in the connectivity communication
 * protocol between the SENSEI board and the host application. 
 * It finds an equivalent in the BIOGAP firmware to ensure consistent command handling across the system 
 */

/*==============================================================================
 * Connectivity Command Codes
 *============================================================================*/

#define GET_DEVICE_SETTINGS 13
#define REQUEST_HARDWARE_VERSION 14
#define GET_BOARD_STATE 15
#define REQUEST_BATTERY_STATE 17
#define START_EEG_STREAMING 18
#define STOP_EEG_STREAMING 19
#define SET_BOARD_STATE 20
#define RESET_BOARD 21
#define ENTER_BOOTLOADERT_MODE 22
#define SET_TRIGGER_STATE 23
#define GO_TO_SLEEP 24
#define RESET_GAP9 25
#define START_MIC_STREAMING 26
#define STOP_MIC_STREAMING 27
#define REQUEST_AVAILABLE_SENSORS 28
#define REQUEST_FIRMWARE_VERSION 29
#define REQUEST_CONNECTING_STRING 30
#define START_STREAMING_ALL 31
#define STOP_STREAMING_ALL 32
#define START_IMU_STREAMING 33
#define STOP_IMU_STREAMING 34
#define START_EEG_MIC_STREAMING 35
#define STOP_EEG_MIC_STREAMING 36
#define SET_DEVICE_SETTINGS 12
#define START_EMG_STREAMING 37
#define STOP_EMG_STREAMING 38
/* Deliberately NOT 250/251 (0xFA/0xFB): the nRF firmware's WULPUS driver
 * treats those as protocol-internal "new config"/"restart" markers embedded
 * in raw MSP430 config bytes forwarded through its command dispatcher --
 * must stay in sync with Firmware/src_NRF/core/connectivity_commands.h. */
#define START_DUMMY_STREAMING 243
#define STOP_DUMMY_STREAMING 244
#define ESP_STOP_COMMAND 245

#endif // CONNECTIVITY_COMMANDS_H

/** @brief Validates a received command */
esp_err_t validate_command(uint8_t command); 