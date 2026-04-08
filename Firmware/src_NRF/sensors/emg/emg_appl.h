/*
 * ----------------------------------------------------------------------
 *
 * File: emg_appl.h
 *
 * Last edited: 09.12.2025
 *
 * Copyright (C) 2025, ETH Zurich and University of Bologna
 *
 * Authors:
 * - Philip Wiese (wiesep@iis.ee.ethz.ch), ETH Zurich
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

/**
 * @file emg_appl.h
 * @brief EMG Application Layer Interface
 *
 * This module provides high-level application control for the ADS1298 EMG
 * sensor. It manages EMG data acquisition and streaming over BLE.
 *
 * The EMG streams at 250 Hz (native sensor ODR) and packages 4 samples
 * per BLE packet, resulting in ~62.5 packets/second.
 */

#ifndef EMG_APPL_H
#define EMG_APPL_H

#include <stdbool.h>
#include <stdint.h>

/*==============================================================================
 * Configuration
 *============================================================================*/

/** @brief EMG sample rate in Hz (ADS1298 ODR setting) */
#define EMG_SAMPLE_RATE 250

/**
 * @brief Number of EMG samples per BLE packet
 *
 * At 250 Hz ODR, 4 samples = 16ms of data, resulting in ~62.5 packets/second.
 */
#define EMG_SAMPLES_PER_PACKET 4

/** @brief Number of EMG channels (ADS1298 has 8 channels per chip, 2 chips = 16) */
#define EMG_NUM_CHANNELS 16

/** @brief Bytes per EMG channel (24-bit data) */
#define EMG_BYTES_PER_CHANNEL 3

/** @brief Bytes per EMG sample (16 channels × 3 bytes + metadata) */
#define EMG_BYTES_PER_SAMPLE 50

/** @brief BLE packet header byte for EMG data */
#define EMG_DATA_HEADER 0x55

/** @brief BLE packet trailer byte for EMG data */
#define EMG_DATA_TRAILER 0xAA

/**
 * @brief Total EMG BLE packet size in bytes
 *
 * Packet structure (211 bytes total):
 * - 1 byte: Header (0x55)
 * - 2 bytes: Packet counter
 * - 4 bytes: Timestamp (microseconds, for cross-packet synchronization)
 * - 200 bytes: 4 samples × 50 bytes per sample
 *   - 24 bytes: ADS1298_A data (8 channels × 3 bytes)
 *   - 24 bytes: ADS1298_B data (8 channels × 3 bytes)
 *   - 1 byte: Counter_extra
 *   - 1 byte: Trigger
 * - 3 bytes: Metadata (reserved for future use)
 * - 1 byte: Trailer (0xAA)
 */
#define EMG_PCKT_SIZE 211

/** @brief Timeout in ms for reading EMG data */
#define EMG_READ_TIMEOUT 100

/*==============================================================================
 * Type Definitions
 *============================================================================*/

/**
 * @enum emg_state_t
 * @brief EMG streaming states
 */
typedef enum {
  EMG_STATE_IDLE,      /**< EMG idle, not streaming */
  EMG_STATE_STARTING,  /**< EMG initializing and configuring */
  EMG_STATE_STREAMING, /**< EMG actively streaming data */
  EMG_STATE_STOPPING,  /**< EMG stopping */
  EMG_STATE_ERROR      /**< Error state */
} emg_state_t;

/**
 * @struct emg_config_t
 * @brief EMG configuration parameters
 * 
 * Configuration array format: [sample_rate, ads_mode, channel_2_func, channel_4_func, gain]
 */
typedef struct {
  uint8_t sample_rate;     /**< ADS1298 sample rate index */
  uint8_t ads_mode;        /**< ADS1298 power mode */
  uint8_t channel_2_func;  /**< ADS1298 channel 2 function */
  uint8_t channel_4_func;  /**< ADS1298 channel 4 function */
  uint8_t gain;            /**< ADS1298 gain setting */
} emg_config_t;

/*==============================================================================
 * Function Declarations
 *============================================================================*/

/**
 * @brief Initialize the EMG subsystem
 *
 * Initializes the ADS1298 sensors and prepares for streaming.
 * Must be called before emg_start_streaming().
 *
 * @return 0 on success, negative error code on failure
 */
int emg_init(void);

/**
 * @brief Start EMG streaming
 *
 * Begins EMG data acquisition and streaming over BLE.
 * The EMG will stream at 250 Hz, packaged into 4-sample BLE packets.
 *
 * If synchronized streaming is active (sync_is_active() returns true),
 * the EMG will wait at the sync barrier before starting data capture.
 *
 * @return 0 on success, negative error code on failure
 * @return -EALREADY if already streaming
 * @return -EBUSY if not in idle state
 */
int emg_start_streaming(void);

/**
 * @brief Stop EMG streaming
 *
 * Stops EMG data acquisition and streaming.
 *
 * @return 0 on success, negative error code on failure
 * @return -EINVAL if not currently streaming
 * @return -ETIMEDOUT if stop operation timed out
 */
int emg_stop_streaming(void);

/**
 * @brief Get current EMG state
 *
 * @return Current emg_state_t value
 */
emg_state_t emg_get_state(void);

/**
 * @brief Check if EMG is currently streaming
 *
 * @return true if streaming, false otherwise
 */
bool emg_is_streaming(void);

/**
 * @brief Set the EMG configuration
 *
 * Updates the EMG configuration parameters. Configuration will be applied
 * on the next call to emg_start_streaming().
 *
 * @param config Pointer to the configuration structure
 * @return 0 on success, negative error code on failure
 */
int emg_set_config(const emg_config_t *config);

/**
 * @brief Get the current EMG configuration
 *
 * Retrieves the current EMG configuration parameters.
 *
 * @param config Pointer to the structure to fill with current config
 * @return 0 on success, negative error code on failure
 */
int emg_get_config(emg_config_t *config);

#endif // EMG_APPL_H