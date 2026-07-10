/*
 * ----------------------------------------------------------------------
 *
 * File: dummy_sensor.c
 *
 * Last edited: 20.05.2026
 *
 * Copyright (C) 2026, ETH Zurich
 *
 * Authors:
 * - Giusy Spacone (gspacone@iis.ee.ethz.ch), ETH Zurich
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
 * @file dummy_sensor_appl.c
 * @brief Dummy Sensor Application Interface
 *
 * This file contains the application-level implementation for simulating a sensor.
 * It can be used for testing the data flow and integration of the system without requiring actual sensor hardware.
 * The dummy sensor will generate synthetic data at a configurable rate and send it through the same pathways as rwal sensors. 
*/

#include "dummy_sensor_appl.h"
#include "ble/ble_appl.h"
#include "wifi_sd_shield/wifi_sd_shield_appl.h"
#include <stdbool.h>
#include <zephyr/kernel.h>
#include <string.h>
#include <stdint.h>
#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(dummy_sensor_appl, LOG_LEVEL_INF);

/*==============================================================================*/
/* Static Function Declarations */
/*==============================================================================*/

/**
 * @brief Dummy sensor streaming thread function
 *
 * Main thread that handles dummy sensor data acquisition. 
 * Runs continuously while dummy_sensor_keep_running is true.
 *
 * @param arg1 Unused
 * @param arg2 Unused
 * @param arg3 Unused
 */
static void dummy_sensor_streaming_thread(void *arg1, void *arg2, void *arg3);


static volatile dummy_sensor_state_t dummy_sensor_state = DUMMY_SENSOR_STATE_IDLE;
static volatile bool dummy_sensor_keep_running = false;
static K_SEM_DEFINE(dummy_sensor_start_sem, 0, 1);
uint8_t dummy_sensor_buf[DUMMY_SENSOR_PCKT_SIZE];
static uint16_t dummy_sensor_buf_idx = DUMMY_SAMPLE_HEADER_SIZE; /* Start after header space */
static uint8_t dummy_sensor_pkt_counter = 0;
static bool first_run = true;

static void dummy_periodic_timer_expiry(struct k_timer *timer);
K_TIMER_DEFINE(dummy_periodic_timer, dummy_periodic_timer_expiry, NULL);
K_SEM_DEFINE(dummy_data_ready_sem, 0, 1);
uint32_t chunk_counter = 0;



void fill_dummy_exg_sample(uint8_t *sample, uint32_t sample_index)
{
  /* Generate DUMMY_BYTES_PER_SAMPLE bytes whose values continue across samples
   * within a packet so the full packet payload becomes 0..(N-1).
   * sample_index is the sample number within the current packet (0..)
   */
  uint32_t base = sample_index * DUMMY_BYTES_PER_SAMPLE;
  for (int i = 0; i < DUMMY_BYTES_PER_SAMPLE; i++) {
    sample[i] = (uint8_t)(base + i);
  }
}

static void build_full_dummy_packet(uint8_t *packet)
{
  /* 
    * Build complete Dummy packet from aggregation buffer:
    * Bytes 0-6: header, counter, timestamp (7 bytes)
    * Bytes 7-206: payload (200 bytes from aggregation_buffer)
    * Bytes 207-209: metadata (3 bytes)
    * Byte 210: tailer
    */
    uint32_t ts_us = k_cyc_to_us_floor32(k_cycle_get_32());
    
    /* Header */
    packet[0] = DUMMY_SENSOR_HEADER;
    
    /* Counter (little-endian) */
    packet[1] = (uint8_t)(dummy_sensor_pkt_counter & 0xFF);
    packet[2] = (uint8_t)((dummy_sensor_pkt_counter >> 8) & 0xFF);
    
    /* Timestamp (microseconds, little-endian) */
    packet[3] = (uint8_t)(ts_us & 0xFF);
    packet[4] = (uint8_t)((ts_us >> 8) & 0xFF);
    packet[5] = (uint8_t)((ts_us >> 16) & 0xFF);
    packet[6] = (uint8_t)((ts_us >> 24) & 0xFF);
    
    /* Metadata bytes */
    packet[DUMMY_SENSOR_PCKT_SIZE - 4] = 0x01;      /* board_id placeholder */
    packet[DUMMY_SENSOR_PCKT_SIZE - 3] = 0x00;  /* sync_pulse_count placeholder */
    packet[DUMMY_SENSOR_PCKT_SIZE - 2] = 0x00;  /* reserved */
    
    /* Tailer */
    packet[DUMMY_SENSOR_PCKT_SIZE - 1] = DUMMY_SENSOR_TAILER;
    //LOG_INF("Built full DUMMY packet,header is 0x%02X, tailer is: 0x%02X", packet[0], packet[DUMMY_SENSOR_PCKT_SIZE - 1]);
}

static void dummy_periodic_timer_expiry(struct k_timer *timer)
{
  ARG_UNUSED(timer);
  k_sem_give(&dummy_data_ready_sem);
}

int dummy_sensor_init(void) {

  LOG_INF("Initializing Dummy Sensor subsystem");

  // Set initial state
  dummy_sensor_state = DUMMY_SENSOR_STATE_IDLE;
  dummy_sensor_keep_running = false;
  dummy_sensor_buf_idx = DUMMY_SAMPLE_HEADER_SIZE; /* Start after header space */
  dummy_sensor_pkt_counter = 0;

  LOG_INF("Dummy Sensor subsystem initialized successfully");
  return 0;
}

int dummy_sensor_start_streaming(void) {
  if (dummy_sensor_state != DUMMY_SENSOR_STATE_IDLE) {
    LOG_ERR("Dummy sensor not in idle state, current state: %d", dummy_sensor_state);
    return -EBUSY;
  }

  LOG_INF("Starting Dummy Sensor streaming");

  dummy_sensor_state = DUMMY_SENSOR_STATE_STARTING;
  dummy_sensor_keep_running = true;
  dummy_sensor_buf_idx = DUMMY_SAMPLE_HEADER_SIZE; /* Start after header space */
  dummy_sensor_pkt_counter = 0;
  chunk_counter = 0;

  // Start dummy sensor hardware. Timer to generate synthetic data ready
  k_timer_start(&dummy_periodic_timer, K_MSEC(SENSOR_SAMPLING_PERIOD_MS), K_MSEC(SENSOR_SAMPLING_PERIOD_MS));
  /* Signal thread to complete startup (sync barrier + ads_start) */
  k_sem_give(&dummy_sensor_start_sem);
  LOG_INF("Signaled Dummy Sensor streaming thread to start");

  return 0;
}

int dummy_sensor_stop_streaming(void){
  if (dummy_sensor_state != DUMMY_SENSOR_STATE_STREAMING) {
    LOG_WRN("Dummy sensor not streaming");
    return -EINVAL;
  }

  dummy_sensor_keep_running = false;
  k_timer_stop(&dummy_periodic_timer); /* Stop timer to avoid running between sessions */
  k_sem_give(&dummy_data_ready_sem); /* In case thread is waiting on timer expiry, unblock it to allow clean shutdown */
  LOG_INF("Stopping Dummy Sensor streaming");

  /* Wait for the streaming thread to stop */
  int timeout = 100; /* 1 second timeout */
  while (dummy_sensor_state != DUMMY_SENSOR_STATE_IDLE && timeout > 0) {
    k_msleep(10);
    timeout--;
  }

  if (timeout == 0) {
    LOG_ERR("Timeout waiting for dummy sensor to stop");
    return -ETIMEDOUT;
  }

  return 0;
}



static void dummy_sensor_streaming_thread(void *arg1, void *arg2, void *arg3) {
  ARG_UNUSED(arg1);
  ARG_UNUSED(arg2);
  ARG_UNUSED(arg3);

  int ret;

  LOG_INF("Dummy sensor streaming thread started, waiting for initialization signal...");

  while (1) {
    /* Wait for start signal from dummy_sensor_start_streaming() */
    k_sem_take(&dummy_sensor_start_sem, K_FOREVER);
    dummy_sensor_state = DUMMY_SENSOR_STATE_STREAMING;

    while (dummy_sensor_keep_running) {
      /* wait for timer expiry (data-ready) */
      k_sem_take(&dummy_data_ready_sem, K_FOREVER);
      // fill dummy sample
      uint8_t dummy_sample[DUMMY_BYTES_PER_SAMPLE];
      fill_dummy_exg_sample(dummy_sample, chunk_counter++);
      // copy to the big buffer
      memcpy(&dummy_sensor_buf[dummy_sensor_buf_idx], dummy_sample, DUMMY_BYTES_PER_SAMPLE);
      dummy_sensor_buf_idx += DUMMY_BYTES_PER_SAMPLE;
      if(dummy_sensor_buf_idx >= (DUMMY_SAMPLE_HEADER_SIZE + DUMMY_BYTES_PER_SAMPLE * DUMMY_SAMPLES_PER_PACKET)){
        build_full_dummy_packet(&dummy_sensor_buf);

        #if defined(CONFIG_WI_FI)
          add_data_to_esp_send_buffer(dummy_sensor_buf, DUMMY_SENSOR_PCKT_SIZE);
        #else
        /* Send via BLE queue */
          add_data_to_send_buffer(dummy_sensor_buf, DUMMY_SENSOR_PCKT_SIZE);
        #endif
        /* Prepare for next packet*/
        dummy_sensor_buf_idx = DUMMY_SAMPLE_HEADER_SIZE; /* Start after header space */
        dummy_sensor_pkt_counter++;
        /* restart sample numbering for the next packet */
        chunk_counter = 0;
      }
    }


    /* stop periodic timer to avoid running between sessions */
    k_timer_stop(&dummy_periodic_timer);

    dummy_sensor_state = DUMMY_SENSOR_STATE_IDLE;
    LOG_INF("Dummy sensor streaming stopped");
  }

  LOG_INF("Dummy sensor streaming thread exiting");
}


#if defined(CONFIG_DUMMY_SENSOR)
  K_THREAD_DEFINE(dummy_sensor_thread_id, DUMMY_SENSOR_THREAD_STACK_SIZE, dummy_sensor_streaming_thread,
  NULL, NULL, NULL, DUMMY_SENSOR_THREAD_PRIORITY, 0, 0);
#endif
