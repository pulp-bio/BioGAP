/*
 * ----------------------------------------------------------------------
 *
 * File: mic_appl.h
 *
 * Last edited: 27.11.2025
 *
 * Copyright (C) 2025, ETH Zurich and University of Bologna
 *
 * Authors:
 * - Based on mic_test example by Philip Wiese (wiesep@iis.ee.ethz.ch), ETH Zurich
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
 * @file mic_appl.h
 * @brief PDM Microphone Application Layer Interface
 *
 * This module provides high-level application control for the PDM microphone.
 * It manages audio capture and streaming over BLE.
 */

#ifndef MIC_APPL_H
#define MIC_APPL_H

#include <stdbool.h>
#include <stdint.h>

/*==============================================================================
 * Configuration
 *============================================================================*/

/** Maximum sample rate for PDM microphone */
#define MIC_MAX_SAMPLE_RATE 16000

/** Downsampling rate (if needed) */
#define MIC_DOWNSAMPLING_RATE 8

/** Sample bit width */
#define MIC_SAMPLE_BIT_WIDTH 16

/** Bytes per sample */
#define MIC_BYTES_PER_SAMPLE sizeof(int16_t)

/** Timeout in ms for reading a block */
#define MIC_READ_TIMEOUT 1000

/** Number of memory blocks for audio buffering */
#define MIC_BLOCK_COUNT 8

/** BLE packet header for mic data (matches ExG format) */
#define MIC_DATA_HEADER 0xAA

/** BLE packet trailer for mic data (matches ExG format) */
#define MIC_DATA_TRAILER 0x55

/** Number of 16-bit samples per BLE packet */
#define MIC_SAMPLES_PER_PACKET 64

/** 
 * Audio payload size per packet
 * - Audio data: 128 bytes (64 samples × 2 bytes)
 */
#define MIC_AUDIO_PAYLOAD_SIZE (MIC_SAMPLES_PER_PACKET * MIC_BYTES_PER_SAMPLE)

/** 
 * Total MIC BLE packet size: 132 bytes
 * - Header: 1 byte
 * - Counter: 2 bytes
 * - Audio data: 128 bytes (64 samples × 2 bytes)
 * - Trailer: 1 byte
 */
#define MIC_PCKT_SIZE (1 + 2 + MIC_AUDIO_PAYLOAD_SIZE + 1)

/*==============================================================================
 * Type Definitions
 *============================================================================*/

/**
 * @enum mic_state_t
 * @brief Microphone streaming states
 */
typedef enum {
    MIC_STATE_IDLE,      /**< Microphone idle, not streaming */
    MIC_STATE_STARTING,  /**< Microphone starting up */
    MIC_STATE_STREAMING, /**< Microphone actively streaming */
    MIC_STATE_STOPPING,  /**< Microphone stopping */
    MIC_STATE_ERROR      /**< Error state */
} mic_state_t;

/*==============================================================================
 * Function Declarations
 *============================================================================*/

/**
 * @brief Initialize the microphone subsystem
 *
 * Configures the DMIC device and prepares for audio capture.
 *
 * @return 0 on success, negative error code on failure
 */
int mic_init(void);

/**
 * @brief Start microphone streaming
 *
 * Begins audio capture and streaming over BLE.
 *
 * @return 0 on success, negative error code on failure
 */
int mic_start_streaming(void);

/**
 * @brief Stop microphone streaming
 *
 * Stops audio capture and streaming.
 *
 * @return 0 on success, negative error code on failure
 */
int mic_stop_streaming(void);

/**
 * @brief Get current microphone state
 *
 * @return Current mic_state_t
 */
mic_state_t mic_get_state(void);

/**
 * @brief Check if microphone is currently streaming
 *
 * @return true if streaming, false otherwise
 */
bool mic_is_streaming(void);

#endif // MIC_APPL_H
