/*
 * ----------------------------------------------------------------------
 *
 * File: command_dispatcher.h
 *
 * Last edited: 07.05.2026
 *
 * Copyright (C) 2026, ETH Zurich and University of Bologna.
 *
 * Authors:
 * - Giusy Spacone (gspacone@iis.ee.ethz.ch), ETH Zurich
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



#ifndef COMMAND_DISPATCHER_H
#define COMMAND_DISPATCHER_H

#include "connectivity_commands.h"
#include "core/common.h"
#include "afe/ads_appl.h"
#include "bsp/system_status/system_status.h"
#include "core/sync_streaming.h"
#include <stdbool.h>
#include <stdint.h>


void handle_connectivity_command(uint8_t cmd);

#endif // COMMAND_DISPATCHER_H