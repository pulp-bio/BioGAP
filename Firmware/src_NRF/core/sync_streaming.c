/*
 * ----------------------------------------------------------------------
 *
 * File: sync_streaming.c
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
 * @file sync_streaming.c
 * @brief Synchronized Streaming Control Implementation
 *
 * Implements a barrier synchronization mechanism using Zephyr's atomic
 * operations and semaphores to coordinate multi-subsystem streaming start.
 *
 * ## Implementation Details
 *
 * The barrier uses a counting approach:
 * 1. `sync_target_count` holds the number of subsystems that must sync
 * 2. `sync_ready_count` is atomically incremented as each subsystem arrives
 * 3. `sync_ready_mask` prevents duplicate registrations using bit flags
 * 4. `sync_barrier_sem` is a counting semaphore used to block/release threads
 *
 * When the last subsystem arrives (ready_count == target_count), it releases
 * all waiting threads by giving the semaphore (target_count - 1) times.
 *
 * ## Thread Safety
 *
 * - `sync_ready_count`: Protected by atomic_inc()
 * - `sync_ready_mask`: Protected by atomic_or()
 * - `sync_barrier_sem`: Zephyr semaphore (inherently thread-safe)
 * - `sync_target_count`: Only written by sync_begin/sync_reset (single caller)
 */

#include "core/sync_streaming.h"

#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/atomic.h>

/* Inter-board synchronization */
#include "core/board_sync.h"

LOG_MODULE_REGISTER(sync_streaming, LOG_LEVEL_DBG);

/*==============================================================================
 * Private Variables
 *============================================================================*/

/**
 * @brief Number of subsystems that must call sync_wait() before barrier opens
 *
 * Set by sync_begin(), reset by sync_reset().
 * Value of 0 or 1 means sync is effectively disabled.
 */
static uint8_t sync_target_count = 0;

/**
 * @brief Atomic counter tracking how many subsystems have reached the barrier
 *
 * Incremented atomically by each sync_wait() call.
 * When this equals sync_target_count, the barrier opens.
 */
static atomic_t sync_ready_count = ATOMIC_INIT(0);

/**
 * @brief Semaphore used to block threads at the barrier
 *
 * Initialized with count 0. Threads block on k_sem_take().
 * The last arriving thread gives (target_count - 1) times to release others.
 * Max count set to SYNC_SUBSYSTEM_COUNT to handle all possible waiters.
 */
static K_SEM_DEFINE(sync_barrier_sem, 0, SYNC_SUBSYSTEM_COUNT);

/**
 * @brief Bitmask tracking which subsystems have registered
 *
 * Bit N is set when subsystem N calls sync_wait().
 * Used to detect and reject duplicate registrations.
 * Prevents bugs where a subsystem accidentally calls sync_wait() twice.
 */
static atomic_t sync_ready_mask = ATOMIC_INIT(0);

/*==============================================================================
 * Public Functions
 *============================================================================*/

void sync_init(void) {
  sync_reset();
  LOG_DBG("Sync module initialized");
}

void sync_begin(uint8_t num_subsystems) {
  /* Clamp to maximum supported subsystems */
  if (num_subsystems > SYNC_SUBSYSTEM_COUNT) {
    LOG_ERR("sync_begin: num_subsystems %d exceeds max %d, clamping", num_subsystems, SYNC_SUBSYSTEM_COUNT);
    num_subsystems = SYNC_SUBSYSTEM_COUNT;
  }

  /* Reset state from any previous session */
  sync_reset();

  /* Set the barrier target */
  sync_target_count = num_subsystems;

  LOG_INF("Sync begin: waiting for %d subsystems", num_subsystems);
}

int sync_wait(sync_subsystem_t subsystem, uint32_t timeout_ms) {
  /* Validate subsystem ID */
  if (subsystem >= SYNC_SUBSYSTEM_COUNT) {
    return -EINVAL;
  }

  /*
   * Check if this subsystem already registered.
   * Use atomic OR to set our bit and get the previous mask.
   * If our bit was already set, we've registered before.
   */
  atomic_val_t mask = BIT(subsystem);
  atomic_val_t old_mask = atomic_or(&sync_ready_mask, mask);
  if (old_mask & mask) {
    LOG_WRN("Subsystem %d already registered", subsystem);
    return -EALREADY;
  }

  /*
   * Atomically increment ready count and get the new value.
   * atomic_inc returns the OLD value, so add 1 to get current count.
   */
  atomic_val_t count = atomic_inc(&sync_ready_count) + 1;
  LOG_DBG("Subsystem %d ready (%d/%d)", subsystem, (int)count, sync_target_count);

  /*
   * Check if we're the last subsystem to arrive.
   * If so, we don't wait - instead we release everyone else.
   */
  if (count >= sync_target_count) {
    LOG_INF("All %d subsystems ready - preparing cross-board sync", sync_target_count);

    /*
     * INTER-BOARD SYNCHRONIZATION
     * Now that all LOCAL subsystems are ready, coordinate with the other board.
     * - SECONDARY: Wait for sync signal from PRIMARY
     * - PRIMARY: Assert sync signal to release SECONDARY
     */
#if defined(CONFIG_BOARD_SYNC_ROLE_SECONDARY)
    LOG_INF("SECONDARY: Waiting for inter-board sync from PRIMARY...");
    int sync_ret = board_sync_wait(10000);
    if (sync_ret != 0) {
      LOG_ERR("Inter-board sync timeout - releasing anyway");
    }
#endif

#if defined(CONFIG_BOARD_SYNC_ROLE_PRIMARY)
    board_sync_assert();
    LOG_INF("PRIMARY: Sync signal asserted");
#endif

    LOG_INF("GO! Releasing all subsystems");

    /*
     * Release all waiting subsystems.
     * We give (target - 1) times because we (the last one) don't wait.
     * Each waiting thread is blocked on k_sem_take(), so each give
     * wakes up exactly one thread.
     */
    for (int i = 0; i < sync_target_count - 1; i++) {
      k_sem_give(&sync_barrier_sem);
    }
    return 0;
  }

  /*
   * We're not the last one - block and wait for others.
   * Convert timeout_ms to Zephyr timeout type.
   */
  k_timeout_t timeout;
  if (timeout_ms == 0) {
    timeout = K_FOREVER;
  } else {
    timeout = K_MSEC(timeout_ms);
  }

  /* Wait on the semaphore until released or timeout occurs */
  int ret = k_sem_take(&sync_barrier_sem, timeout);

  if (ret == -EAGAIN) {
    /* Timeout expired before we were released */
    LOG_ERR("Subsystem %d timeout waiting for sync", subsystem);
    return -ETIMEDOUT;
  }

  LOG_DBG("Subsystem %d released from barrier", subsystem);
  return 0;
}

bool sync_is_active(void) {
  /*
   * Sync is only meaningful with 2+ subsystems.
   * With 0 or 1, there's nothing to synchronize.
   */
  return (sync_target_count > 1);
}

void sync_reset(void) {
  /*
   * Reset all state for a fresh sync session.
   * Order matters: reset semaphore last to avoid race conditions
   * where a thread might try to take it during reset.
   */

  /*
   * INTER-BOARD SYNC CLEANUP
   * Deassert the sync line when session ends (PRIMARY only).
   */
#if defined(CONFIG_BOARD_SYNC_ROLE_PRIMARY)
  board_sync_deassert();
#endif

  sync_target_count = 0;
  atomic_set(&sync_ready_count, 0);
  atomic_set(&sync_ready_mask, 0);
  k_sem_reset(&sync_barrier_sem);

  LOG_DBG("Sync state reset");
}
