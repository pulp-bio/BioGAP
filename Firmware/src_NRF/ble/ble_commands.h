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
#define START_PPG_STREAMING 39
#define STOP_PPG_STREAMING 40
#define START_WULPUS_STREAMING 41
#define STOP_WULPUS_STREAMING 42
/* Extended battery/system status (v2). Response header 43 = 0x2B is chosen
 * to not collide with any streaming packet header (0x55 ExG, 0x56 IMU,
 * 0xAA MIC, 0x10-0x13 WULPUS, 0x70 PPG) - unlike the legacy
 * REQUEST_BATTERY_STATE echo (17 = 0x11), which is ambiguous with WULPUS
 * chunk 2 during ultrasound streaming. */
#define REQUEST_SYSTEM_STATUS 43

/* mmWave radar (BGT60TR13C, SENSEI mmWave shield).
 *
 * The radar has to be powered and configured before it can stream, so the
 * host sequence is TURN_ON -> [CHANGE_*] -> CONFIGURE -> START, and
 * STOP -> TURN_OFF on the way down (see sensors/mmWave/mmWave_appl.h).
 *
 * CHANGE_IFGAIN_MMWAVE, CHANGE_TXPOWER_MMWAVE and CHANGE_FPS_MMWAVE expect
 * one parameter byte after the opcode; without it they are ignored. */
#define START_MMWAVE_STREAMING 44
#define STOP_MMWAVE_STREAMING 45
#define CONFIGURE_MMWAVE 46
#define TURN_OFF_MMWAVE 47
#define TURN_ON_MMWAVE 48
#define CHANGE_IFGAIN_MMWAVE 49
#define CHANGE_TXPOWER_MMWAVE 50
#define CHANGE_FPS_MMWAVE 51

#endif // BLE_COMMANDS_H
