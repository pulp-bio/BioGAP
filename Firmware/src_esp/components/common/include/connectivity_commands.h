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

/** @brief NRF_EXG packet markers */
#define NRF_EXG_HEADER            0x55
#define NRF_EXG_TAILER            0xAA
#define ESP_EXG_HEADER            0x66
#define ESP_EXG_TAILER            0xBB

/** @brief WULPUS packet markers */
#define WULPUS_HDR_XFER_0      0x10
#define WULPUS_HDR_XFER_1      0x11
#define WULPUS_HDR_XFER_2      0x12
#define WULPUS_HDR_XFER_3      0x13

/** @brief WULPUS packet markers when entire frame is received */
#define WULPUS_FULL_HEADER          0xCC
#define WULPUS_FULL_TAILER          0xDD

#define BIOGUI_CHECK_HEADER        0x55
#define BIOGUI_CHECK_TAILER        0xAA

#endif // CONNECTIVITY_COMMANDS_H
