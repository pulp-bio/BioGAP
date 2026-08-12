/*
 * ----------------------------------------------------------------------
 *
 * File: mmWave_config.h
 *
 * Copyright (C) 2026, ETH Zurich
 *
 * Authors:
 * - Benjamin Löliger, ETH Zurich
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

/**
 * @file mmWave_config.h
 * @brief Selects the BGT60TR13C register profile and derives frame geometry
 *
 * The radar's chirp timing, frame rate and frame geometry are fixed by a
 * generated register list. One profile is selected through Kconfig
 * (CONFIG_MMWAVE_CONF_*), and this header pulls in the matching generated
 * header so every translation unit agrees on the frame geometry.
 *
 * @note The generated headers only define `register_list[]` when
 * XENSIV_BGT60TRXX_CONF_IMPL is defined before the first inclusion. Exactly
 * one translation unit (mmWave_appl.c) does that; everyone else gets the
 * XENSIV_BGT60TRXX_CONF_* geometry macros only.
 */

#ifndef MMWAVE_CONFIG_H
#define MMWAVE_CONFIG_H

#include <stdint.h>

#if defined(CONFIG_MMWAVE_CONF_25FPS)
#include "sensors/mmWave/driver/25fps.h"
#elif defined(CONFIG_MMWAVE_CONF_50FPS)
#include "sensors/mmWave/driver/50fps.h"
#elif defined(CONFIG_MMWAVE_CONF_100FPS)
#include "sensors/mmWave/driver/100fps.h"
#elif defined(CONFIG_MMWAVE_CONF_150FPS)
#include "sensors/mmWave/driver/150fps.h"
#elif defined(CONFIG_MMWAVE_CONF_200FPS)
#include "sensors/mmWave/driver/200fps.h"
#elif defined(CONFIG_MMWAVE_CONF_STATIC_DISTANCE)
#include "sensors/mmWave/driver/static_distance.h"
#else /* CONFIG_MMWAVE_CONF_100FPS_32C_8S, the default */
#include "sensors/mmWave/driver/100fps_32chirps_8samples_2000kHz.h"
#endif

/*==============================================================================
 * Derived Frame Geometry
 *============================================================================*/

/** @brief Number of ADC samples contained in one radar frame */
#define MMWAVE_NUM_SAMPLES_PER_FRAME                                           \
  (XENSIV_BGT60TRXX_CONF_NUM_RX_ANTENNAS *                                     \
   XENSIV_BGT60TRXX_CONF_NUM_CHIRPS_PER_FRAME *                                \
   XENSIV_BGT60TRXX_CONF_NUM_SAMPLES_PER_CHIRP)

/** @brief Frame size in bytes when samples are sent as uint16_t */
#define MMWAVE_FRAME_SIZE_BYTES_U16                                            \
  (MMWAVE_NUM_SAMPLES_PER_FRAME * sizeof(uint16_t))

/** @brief Frame size in bytes with the radar's native 12-bit sample packing
 *  (two samples per three bytes, as they arrive from the FIFO) */
#define MMWAVE_FRAME_SIZE_BYTES_PACKED                                         \
  ((MMWAVE_NUM_SAMPLES_PER_FRAME * 3U) / 2U)

#endif /* MMWAVE_CONFIG_H */
