/*
 * ----------------------------------------------------------------------
 *
 * File: sync_streaming.h
 *
 * Copyright (C) 2025, ETH Zurich
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
 * @file sync_streaming.h
 * @brief Synchronized Streaming Control for Multi-Sensor Data Acquisition
 *
 * This module provides a barrier synchronization mechanism to ensure multiple
 * streaming subsystems (EXG, MIC) start capturing data at exactly the same time.
 *
 * ## Problem
 *
 * When starting combined EXG+MIC streaming, each subsystem has different
 * initialization times:
 * - MIC: Fast (~1ms) - just configures DMIC
 * - EXG: Slow (~500ms+) - waits for BLE config, sleeps 200ms, initializes ADS chips
 *
 * Without synchronization, MIC would start streaming hundreds of milliseconds
 * before EXG, making time-aligned multi-modal analysis impossible.
 *
 * ## Solution
 *
 * A barrier synchronization pattern where:
 * 1. `sync_begin(N)` sets up a barrier for N subsystems
 * 2. Each subsystem completes its initialization
 * 3. Each subsystem calls `sync_wait()` - this blocks until all N are ready
 * 4. Once all N subsystems have called `sync_wait()`, they all unblock together
 * 5. Each subsystem immediately starts its hardware (DMIC/ADS)
 *
 * ## Usage Example
 *
 * ```c
 * // In BLE command handler (START_COMBINED_STREAMING):
 * sync_begin(2);              // Expect 2 subsystems
 * mic_start_streaming();      // Wake MIC thread
 * ads_set_function(ADS_START); // Trigger EXG state machine
 *
 * // In MIC thread:
 * dmic_configure(...);        // Initialize hardware
 * sync_wait(SYNC_SUBSYSTEM_MIC, 5000);  // Wait at barrier
 * dmic_trigger(DMIC_START);   // Start after barrier releases
 *
 * // In EXG state machine:
 * GetConfigParam(...);        // Wait for BLE config
 * ads_init(...);              // Initialize hardware
 * sync_wait(SYNC_SUBSYSTEM_EXG, 5000);  // Wait at barrier
 * ads_start();                // Start after barrier releases
 * ```
 *
 * ## Thread Safety
 *
 * All functions are thread-safe and use atomic operations and Zephyr
 * synchronization primitives.
 */

#ifndef SYNC_STREAMING_H
#define SYNC_STREAMING_H

#include <stdbool.h>
#include <stdint.h>

/*==============================================================================
 * Type Definitions
 *============================================================================*/

/**
 * @enum sync_subsystem_t
 * @brief Identifiers for streaming subsystems that can be synchronized
 *
 * Each subsystem that participates in synchronized streaming must have
 * a unique identifier. The identifier is used to:
 * - Prevent duplicate registration (same subsystem calling sync_wait twice)
 * - Track which subsystems have reached the barrier
 */
typedef enum {
    SYNC_SUBSYSTEM_EXG = 0,  /**< EXG/ADS1298 bio-potential acquisition subsystem */
    SYNC_SUBSYSTEM_MIC = 1,  /**< PDM microphone audio capture subsystem */
    SYNC_SUBSYSTEM_IMU = 2,  /**< IMU/LIS2DUXS12 accelerometer subsystem */
    SYNC_SUBSYSTEM_COUNT     /**< Total number of syncable subsystems (max barrier size) */
} sync_subsystem_t;

/*==============================================================================
 * Function Declarations
 *============================================================================*/

/**
 * @brief Initialize the sync module
 *
 * Resets all internal state. Called once at system startup.
 * Safe to call multiple times.
 *
 * @note This is optional if sync_begin() is always called before use,
 *       as sync_begin() internally calls sync_reset().
 */
void sync_init(void);

/**
 * @brief Begin a synchronized streaming session
 *
 * Sets up a barrier that will block until the specified number of subsystems
 * have called sync_wait(). Must be called before starting any subsystems.
 *
 * @param num_subsystems Number of subsystems that will participate in sync.
 *                       - 1: No synchronization (sync_wait returns immediately)
 *                       - 2: Both EXG and MIC will sync
 *                       - Values > SYNC_SUBSYSTEM_COUNT are clamped
 *
 * @note Calling sync_begin() resets any previous sync state.
 *
 * Example:
 * @code
 * sync_begin(2);  // Prepare for EXG + MIC sync
 * @endcode
 */
void sync_begin(uint8_t num_subsystems);

/**
 * @brief Signal readiness and wait for other subsystems at the barrier
 *
 * This is the core synchronization function. Each subsystem calls this after
 * completing its initialization but BEFORE starting actual data capture.
 *
 * Behavior:
 * - Atomically increments the ready count
 * - If not all subsystems ready: blocks on semaphore until released
 * - If this is the last subsystem: releases all waiting subsystems and returns
 *
 * @param subsystem The subsystem identifier (must be unique per sync session)
 * @param timeout_ms Maximum time to wait for other subsystems:
 *                   - 0: Wait forever (K_FOREVER)
 *                   - >0: Timeout in milliseconds
 *
 * @return 0 on success (all subsystems synchronized)
 * @return -EINVAL if subsystem ID is invalid
 * @return -EALREADY if this subsystem already called sync_wait in this session
 * @return -ETIMEDOUT if timeout expired before all subsystems were ready
 *
 * @note After this function returns 0, all subsystems should immediately
 *       start their hardware to minimize timing skew.
 *
 * Example:
 * @code
 * // After initialization, before starting capture:
 * int ret = sync_wait(SYNC_SUBSYSTEM_MIC, 5000);
 * if (ret == 0) {
 *     dmic_trigger(dmic_dev, DMIC_TRIGGER_START);
 * } else {
 *     LOG_ERR("Sync failed: %d", ret);
 * }
 * @endcode
 */
int sync_wait(sync_subsystem_t subsystem, uint32_t timeout_ms);

/**
 * @brief Check if synchronized streaming mode is active
 *
 * Use this to conditionally execute sync logic. When sync is not active,
 * subsystems can start immediately without waiting.
 *
 * @return true if sync_begin() was called with num_subsystems > 1
 * @return false if no sync session is active or only 1 subsystem registered
 *
 * Example:
 * @code
 * ads_init(...);
 * if (sync_is_active()) {
 *     sync_wait(SYNC_SUBSYSTEM_EXG, 5000);
 * }
 * ads_start();
 * @endcode
 */
bool sync_is_active(void);

/**
 * @brief Reset the sync state for a new session
 *
 * Clears all internal state:
 * - Resets target count to 0
 * - Resets ready count to 0
 * - Clears the ready mask (allows subsystems to register again)
 * - Resets the barrier semaphore
 *
 * Call this when:
 * - Streaming stops (STOP_COMBINED_STREAMING)
 * - An error occurs and sync needs to be aborted
 * - Before starting a new sync session (sync_begin does this automatically)
 *
 * @note Safe to call even if no sync session is active.
 */
void sync_reset(void);

#endif /* SYNC_STREAMING_H */
