/*
 * ----------------------------------------------------------------------
 *
 * File: imu_appl.c
 *
 * Last edited: 08.12.2025
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
 * @file imu_appl.c
 * @brief IMU Application Layer Implementation
 *
 * Implements the application-level control for the LIS2DUXS12 IMU including
 * accelerometer data acquisition and BLE streaming.
 */

#include "sensors/imu/imu_appl.h"
#include "ble/ble_appl.h"
#include "sensors/imu/lis2duxs12_sensor.h"
#include "core/sync_streaming.h"

#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>

/* Initialize the logging module */
LOG_MODULE_REGISTER(imu_appl, LOG_LEVEL_INF);

/*==============================================================================
 * Private Definitions
 *============================================================================*/

/** @brief Thread stack size */
#define IMU_THREAD_STACK_SIZE 2048

/** @brief Thread priority */
#define IMU_THREAD_PRIORITY 6

/*==============================================================================
 * Private Variables
 *============================================================================*/

/** @brief Current IMU state */
static volatile imu_state_t imu_state = IMU_STATE_IDLE;

/** @brief Flag to signal streaming thread to stop */
static volatile bool imu_keep_running = false;

/** @brief Semaphore to signal start of streaming */
static K_SEM_DEFINE(imu_start_sem, 0, 1);

/** @brief IMU BLE packet buffer */
static uint8_t imu_tx_buf[IMU_PCKT_SIZE];

/** @brief Current index in IMU packet buffer */
static uint8_t imu_buf_idx = 0;

/** @brief IMU packet counter (wraps at 256) */
static uint8_t imu_pkt_counter = 0;

/** @brief Timestamp of first sample in current packet */
static uint32_t imu_packet_timestamp = 0;

/** @brief Number of samples accumulated in current packet */
static uint8_t imu_sample_count = 0;

/** @brief Flag indicating if IMU has been initialized */
static bool imu_initialized = false;

/*==============================================================================
 * Private Functions
 *============================================================================*/

/**
 * @brief Initialize IMU packet buffer for new packet
 *
 * Resets buffer index and prepares header bytes. The timestamp
 * will be captured when the first sample is added.
 */
static void imu_packet_init(void) {
  imu_buf_idx = 0;
  imu_tx_buf[imu_buf_idx++] = IMU_DATA_HEADER;
  imu_tx_buf[imu_buf_idx++] = imu_pkt_counter++;
  /* Reserve 4 bytes for timestamp (filled when first sample arrives) */
  imu_buf_idx += 4;
  imu_sample_count = 0;
}

/**
 * @brief Add a sample to the IMU packet buffer and send if full
 *
 * Adds accelerometer X, Y, Z data to the packet. When the packet
 * contains IMU_SAMPLES_PER_PACKET samples, it is sent via BLE.
 *
 * @param x Acceleration X (raw int16_t)
 * @param y Acceleration Y (raw int16_t)
 * @param z Acceleration Z (raw int16_t)
 */
static void imu_packet_add_sample(int16_t x, int16_t y, int16_t z) {
  /* Capture timestamp on first sample */
  if (imu_sample_count == 0) {
    imu_packet_timestamp = k_cyc_to_us_floor32(k_cycle_get_32());
    /* Fill in timestamp bytes (little-endian) */
    imu_tx_buf[2] = (uint8_t)(imu_packet_timestamp & 0xFF);
    imu_tx_buf[3] = (uint8_t)((imu_packet_timestamp >> 8) & 0xFF);
    imu_tx_buf[4] = (uint8_t)((imu_packet_timestamp >> 16) & 0xFF);
    imu_tx_buf[5] = (uint8_t)((imu_packet_timestamp >> 24) & 0xFF);
  }

  /* Add X, Y, Z accelerometer data (big-endian) */
  imu_tx_buf[imu_buf_idx++] = (uint8_t)(x >> 8);    /* X high byte */
  imu_tx_buf[imu_buf_idx++] = (uint8_t)(x & 0xFF);  /* X low byte */
  imu_tx_buf[imu_buf_idx++] = (uint8_t)(y >> 8);    /* Y high byte */
  imu_tx_buf[imu_buf_idx++] = (uint8_t)(y & 0xFF);  /* Y low byte */
  imu_tx_buf[imu_buf_idx++] = (uint8_t)(z >> 8);    /* Z high byte */
  imu_tx_buf[imu_buf_idx++] = (uint8_t)(z & 0xFF);  /* Z low byte */

  imu_sample_count++;

  /* Check if packet is full */
  if (imu_sample_count >= IMU_SAMPLES_PER_PACKET) {
    /* Add trailer */
    imu_tx_buf[imu_buf_idx++] = IMU_DATA_TRAILER;

    /* Send packet via BLE queue */
    add_data_to_send_buffer(imu_tx_buf, IMU_PCKT_SIZE);

    /* Reset for next packet */
    imu_packet_init();
  }
}

/**
 * @brief IMU streaming thread
 *
 * Main thread for IMU data acquisition. Waits for start signal, initializes
 * sensor, then continuously reads accelerometer data and sends it over BLE.
 */
static void imu_streaming_thread(void *arg1, void *arg2, void *arg3) {
  ARG_UNUSED(arg1);
  ARG_UNUSED(arg2);
  ARG_UNUSED(arg3);

  int ret;
  int16_t x, y, z;

  LOG_INF("IMU streaming thread started");

  while (1) {
    /* Wait for start signal */
    k_sem_take(&imu_start_sem, K_FOREVER);

    LOG_INF("Starting IMU capture");
    imu_state = IMU_STATE_STARTING;

    /* Initialize the sensor if not already done */
    if (!imu_initialized) {
      ret = lis2duxs12_init();
      if (ret != 0) {
        LOG_ERR("Failed to initialize LIS2DUXS12: %d", ret);
        imu_state = IMU_STATE_ERROR;
        continue;
      }
      imu_initialized = true;
    }

    /* Start accelerometer sampling at 400Hz */
    ret = lis2duxs12_start_sampling();
    if (ret != 0) {
      LOG_ERR("Failed to start LIS2DUXS12 sampling: %d", ret);
      imu_state = IMU_STATE_ERROR;
      continue;
    }

    /* Wait at sync barrier if synchronized streaming is active */
    if (sync_is_active()) {
      LOG_INF("IMU ready, waiting at sync barrier...");
      ret = sync_wait(SYNC_SUBSYSTEM_IMU, 5000);
      if (ret != 0) {
        LOG_ERR("Sync wait failed: %d", ret);
        lis2duxs12_stop_sampling();
        imu_state = IMU_STATE_ERROR;
        continue;
      }
    }

    imu_state = IMU_STATE_STREAMING;
    imu_pkt_counter = 0;
    imu_packet_init();

    LOG_INF("IMU streaming active at %u Hz", IMU_SAMPLE_RATE);

    /* Streaming loop - wait for DRDY interrupt and read data */
    while (imu_keep_running) {
      ret = lis2duxs12_wait_data_ready(IMU_READ_TIMEOUT);
      if (ret != 0) {
        if (ret == -EAGAIN) {
          /* Timeout - check if we should keep running */
          continue;
        }
        LOG_ERR("Wait for data ready failed: %d", ret);
        break;
      }

      ret = lis2duxs12_read_accel(&x, &y, &z);
      if (ret != 0) {
        LOG_ERR("Failed to read accelerometer: %d", ret);
        continue;
      }

      /* Add sample to packet (sends automatically when full) */
      imu_packet_add_sample(x, y, z);
    }

    /* Stop accelerometer sampling */
    imu_state = IMU_STATE_STOPPING;
    ret = lis2duxs12_stop_sampling();
    if (ret != 0) {
      LOG_ERR("Failed to stop LIS2DUXS12: %d", ret);
    }

    imu_state = IMU_STATE_IDLE;
    LOG_INF("IMU streaming stopped");
  }
}

/* Define the IMU thread */
K_THREAD_DEFINE(imu_thread_id, IMU_THREAD_STACK_SIZE, imu_streaming_thread, 
                NULL, NULL, NULL, IMU_THREAD_PRIORITY, 0, 0);

/*==============================================================================
 * Public Functions
 *============================================================================*/

int imu_init(void) {
  int ret;

  if (imu_initialized) {
    LOG_DBG("IMU already initialized");
    return 0;
  }

  /* Initialize the LIS2DUXS12 sensor */
  ret = lis2duxs12_init();
  if (ret != 0) {
    LOG_ERR("Failed to initialize LIS2DUXS12: %d", ret);
    return ret;
  }

  imu_initialized = true;
  LOG_INF("IMU initialized (sample rate: %u Hz)", IMU_SAMPLE_RATE);
  return 0;
}

int imu_start_streaming(void) {
  if (imu_state == IMU_STATE_STREAMING) {
    LOG_WRN("IMU already streaming");
    return -EALREADY;
  }

  if (imu_state != IMU_STATE_IDLE) {
    LOG_ERR("IMU not in idle state (current: %d)", imu_state);
    return -EBUSY;
  }

  imu_keep_running = true;
  k_sem_give(&imu_start_sem);

  return 0;
}

int imu_stop_streaming(void) {
  if (imu_state != IMU_STATE_STREAMING) {
    LOG_WRN("IMU not streaming");
    return -EINVAL;
  }

  imu_keep_running = false;

  /* Wait for the streaming thread to stop */
  int timeout = 100; /* 1 second timeout (100 * 10ms) */
  while (imu_state != IMU_STATE_IDLE && timeout > 0) {
    k_msleep(10);
    timeout--;
  }

  if (timeout == 0) {
    LOG_ERR("Timeout waiting for IMU to stop");
    return -ETIMEDOUT;
  }

  return 0;
}

imu_state_t imu_get_state(void) { 
  return imu_state; 
}

bool imu_is_streaming(void) { 
  return (imu_state == IMU_STATE_STREAMING); 
}

int imu_read_temperature(float *temp_celsius) {
  if (temp_celsius == NULL) {
    return -EINVAL;
  }

  if (!imu_initialized) {
    LOG_ERR("IMU not initialized");
    return -ENODEV;
  }

  return lis2duxs12_read_temperature(temp_celsius);
}
