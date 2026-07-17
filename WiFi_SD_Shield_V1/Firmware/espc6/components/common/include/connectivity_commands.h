/*
 * ----------------------------------------------------------------------
 *
 * File: connectivity_commands.h
 *
 * Last edited: 17.07.2026
 *
 * Copyright (c) 2026 ETH Zurich and University of Bologna
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
 * This header defines extra commands needed for the connectivity between ESP and the BIOGUI.
 * The ESP acts as a bridge between the BIOGAP and the GUI, forwarding commands and data.
 * Full list of commands can be found in the BIOGAP firmware's connectivity_commands.h file. 
 */

#define START_DUMMY_STREAMING 243
#define STOP_DUMMY_STREAMING 244
#define ESP_STOP_COMMAND 245

#define NRF_EXG_HEADER            0x55
#define NRF_EXG_TAILER            0xAA
#define ESP_EXG_HEADER            0x66
#define ESP_EXG_TAILER            0xBB
#define NRF_EXG_PACKET_SIZE       211

#endif // CONNECTIVITY_COMMANDS_H
