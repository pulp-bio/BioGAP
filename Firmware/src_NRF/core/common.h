/*
 * ----------------------------------------------------------------------
 *
 * File: common.h
 *
 * Last edited: 05.12.2025
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

/**
 * @file common.h
 * @brief Common Definitions and Includes for ExG Stream Application
 *
 * This header provides shared definitions used across the ExG streaming
 * application including board states, version information, and commonly
 * needed module includes.
 */

#ifndef COMMON_H
#define COMMON_H

#include <stdint.h>

#include "ble/bluetooth.h"

/*==============================================================================
 * Board State Definitions
 *============================================================================*/

/** @brief Nordic streaming mode - data streamed via nRF BLE */
#define STATE_STREAMING_NORDIC 50

/** @brief GAP9 master mode - GAP9 controls data acquisition */
#define STATE_GAP9_MASTER 60

/** @brief State switch in progress */
#define STATE_SWITCH 70

/** @brief Biowolf programming mode */
#define STATE_PROGRAM_WOLF 80

/** @brief Electrode-skin quality measurement mode */
#define STATE_ES_QUALITY 90

/*==============================================================================
 * Version Information
 *============================================================================*/

/** @brief Firmware major version */
#define FIRMWARE_VERSION '2'

/** @brief Firmware minor revision */
#define FIRMWARE_REVISION 'c'

/** @brief Hardware major version */
#define HARDWARE_VERSION '2'

/** @brief Hardware minor revision */
#define HARDWARE_REVISION 'b'

/*==============================================================================
 * Utility Definitions
 *============================================================================*/

/** @brief Success return code */
#define NO_ERROR 0

#endif /* COMMON_H */
