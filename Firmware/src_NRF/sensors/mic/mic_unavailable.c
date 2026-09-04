/*
 * ----------------------------------------------------------------------
 *
 * File: mic_unavailable.c
 *
 * Copyright (C) 2026, ETH Zurich
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
 * @file mic_unavailable.c
 * @brief Microphone API for builds that have no PDM microphone
 *
 * Built instead of mic_appl.c when the PDM peripheral is not part of the
 * image (CONFIG_AUDIO_DMIC=n). The real implementation resolves the DMIC
 * device with DEVICE_DT_GET(), which does not link when the devicetree node is
 * disabled -- so a build that gives the microphone's pins to another peripheral
 * needs these definitions instead.
 *
 * Currently that applies to the mmWave radar build (-DMMWAVE_SHIELD=ON): the
 * radar's RST and IRQ lines are the microphone's CLK and DATA pins
 * (P0.04 / P0.12), so the two are mutually exclusive on this hardware.
 *
 * Keeping the API present means the transport-agnostic command dispatcher and
 * the combined-streaming commands need no conditional compilation; requests
 * for microphone data are simply rejected at runtime.
 */

#include "sensors/mic/mic_appl.h"

#include <errno.h>
#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(mic_unavailable, LOG_LEVEL_INF);

/** @brief Log the rejection once, so a mistaken request is visible but a
 *  polling host cannot flood the log. */
static void mic_warn_unavailable(void) {
  static bool warned;

  if (!warned) {
    warned = true;
    LOG_WRN("No microphone in this build: its CLK/DATA pins (P0.04/P0.12) are "
            "used by the mmWave radar");
  }
}

int mic_init(void) { return -ENODEV; }

int mic_start_streaming(void) {
  mic_warn_unavailable();
  return -ENODEV;
}

int mic_stop_streaming(void) { return -ENODEV; }

mic_state_t mic_get_state(void) { return MIC_STATE_IDLE; }

bool mic_is_streaming(void) { return false; }
