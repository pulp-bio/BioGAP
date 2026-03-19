/*
 * ----------------------------------------------------------------------
 *
 * File: ble_commands.h
 *
 * Last edited: 06.01.2026
 *
 * Copyright (C) 2026, ETH Zurich
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

#ifndef BLE_COMMANDS_H
#define BLE_COMMANDS_H

/**
 * @file ble_commands.h
 * @brief BLE Protocol Command Definitions
 *
 * This header defines the command codes used in the BLE communication
 * protocol between the SENSEI board and the host application.
 */

/*==============================================================================
 * BLE Command Codes
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

#endif // BLE_COMMANDS_H
