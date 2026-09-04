/*
 * ----------------------------------------------------------------------
 *
 * File: board_sync.h
 *
 * Copyright (C) 2025, ETH Zurich and University of Bologna
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
 * @file board_sync.h
 * @brief Inter-Board Hardware Synchronization
 *
 * This module provides hardware-based synchronization between multiple boards
 * via a GPIO trigger line. One board acts as PRIMARY (generates sync pulses),
 * others act as SECONDARY (wait for sync pulses).
 *
 * ## Usage
 *
 * PRIMARY board:
 * - Calls board_sync_assert() when starting acquisition
 * - Calls board_sync_pulse() periodically for drift correction markers
 * - Calls board_sync_deassert() when stopping acquisition
 *
 * SECONDARY board:
 * - Calls board_sync_wait() before starting acquisition
 * - Receives sync pulses via GPIO interrupt
 *
 * ## Hardware Connection
 *
 * Connect GPIO sync pin between boards (PRIMARY output → SECONDARY input)
 * and ensure common ground.
 */

#ifndef BOARD_SYNC_H
#define BOARD_SYNC_H

#include <stdbool.h>
#include <stdint.h>

/*==============================================================================
 * Function Declarations
 *============================================================================*/

/**
 * @brief Initialize board synchronization GPIO
 *
 * Configures the sync GPIO as output (PRIMARY) or input with interrupt (SECONDARY).
 *
 * @return 0 on success, negative error code on failure
 */
int board_sync_init(void);

/**
 * @brief Assert sync line high (PRIMARY only)
 *
 * Call this when starting acquisition to signal SECONDARY boards.
 * Increments the sync pulse counter.
 */
void board_sync_assert(void);

/**
 * @brief Deassert sync line low (PRIMARY only)
 *
 * Call this when stopping acquisition.
 */
void board_sync_deassert(void);

/**
 * @brief Send a short sync pulse (PRIMARY only)
 *
 * Generates a brief pulse (~10µs) for periodic drift correction markers.
 * Increments the sync pulse counter.
 */
void board_sync_pulse(void);

/**
 * @brief Wait for sync signal from PRIMARY (SECONDARY only)
 *
 * Blocks until a sync pulse is received or timeout expires.
 *
 * @param timeout_ms Timeout in milliseconds (0 = wait forever)
 * @return 0 on success, -ETIMEDOUT on timeout
 */
int board_sync_wait(uint32_t timeout_ms);

/**
 * @brief Check if this board is configured as PRIMARY
 *
 * @return true if PRIMARY role, false otherwise
 */
bool board_sync_is_primary(void);

/**
 * @brief Get the sync pulse counter
 *
 * Returns the number of sync pulses sent (PRIMARY) or received (SECONDARY).
 * Embedded in packet metadata for post-hoc alignment.
 *
 * @return Current pulse count (0-255, wraps)
 */
uint8_t board_sync_get_pulse_count(void);

/**
 * @brief Get the configured board ID
 *
 * Returns CONFIG_BOARD_ID, unique identifier for each board.
 *
 * @return Board ID (1-255)
 */
uint8_t board_sync_get_board_id(void);

#endif /* BOARD_SYNC_H */
