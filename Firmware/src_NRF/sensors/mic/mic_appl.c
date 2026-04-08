/*
 * ----------------------------------------------------------------------
 *
 * File: mic_appl.c
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
 * @file mic_appl.c
 * @brief PDM Microphone Application Layer Implementation
 *
 * Implements the application-level control for the PDM microphone including
 * audio capture and BLE streaming.
 */

#include "sensors/mic/mic_appl.h"
#include "ble/ble_appl.h"
#include "core/sync_streaming.h"

#include <string.h>
#include <zephyr/audio/dmic.h>
#include <zephyr/device.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>

/* Initialize the logging module */
LOG_MODULE_REGISTER(mic_appl, LOG_LEVEL_INF);

/*==============================================================================
 * Private Definitions
 *============================================================================*/

/* Size of a block for 10 ms of audio data (reduced from 100ms for better interleaving) */
#define BLOCK_SIZE(_sample_rate, _number_of_channels)                                                                  \
  (MIC_BYTES_PER_SAMPLE * ((_sample_rate) / 10) * (_number_of_channels))

/* Maximum block size based on max sample rate and stereo */
#define MAX_BLOCK_SIZE BLOCK_SIZE(MIC_MAX_SAMPLE_RATE, 2)

/* Thread configuration */
#define MIC_THREAD_STACK_SIZE 2048
#define MIC_THREAD_PRIORITY 6

/*==============================================================================
 * Private Variables
 *============================================================================*/

/* Memory slab for audio buffers */
K_MEM_SLAB_DEFINE_STATIC(mic_mem_slab, MAX_BLOCK_SIZE, MIC_BLOCK_COUNT, 4);

/* DMIC device */
static const struct device *dmic_dev;

/* Current microphone state */
static volatile mic_state_t mic_state = MIC_STATE_IDLE;

/* Flag to signal streaming thread to stop */
static volatile bool mic_keep_running = false;

/* Semaphore to signal start of streaming */
static K_SEM_DEFINE(mic_start_sem, 0, 1);

/* DMIC configuration */
static struct pcm_stream_cfg mic_stream = {
    .pcm_width = MIC_SAMPLE_BIT_WIDTH,
    .mem_slab = &mic_mem_slab,
};

static struct dmic_cfg mic_cfg = {
    .io =
        {
            /* PDM clock configuration limits supported by the microphone */
            .min_pdm_clk_freq = 1000000,
            .max_pdm_clk_freq = 3500000,
            .min_pdm_clk_dc = 40,
            .max_pdm_clk_dc = 60,
        },
    .streams = &mic_stream,
    .channel =
        {
            .req_num_streams = 1,
        },
};

/*==============================================================================
 * Private Functions
 *============================================================================*/

/**
 * @brief Microphone streaming thread
 *
 * Continuously reads audio data from the DMIC and sends it over BLE.
 * Packet format:
 *   [header(1)][counter(2)][audio_data(128)][trailer(1)] = 132 bytes
 *   64 samples × 2 bytes = 128 bytes audio, matching 4 ms EEG packet timing
 */
static void mic_streaming_thread(void *arg1, void *arg2, void *arg3) {
  int ret;
  uint8_t ble_packet[MIC_PCKT_SIZE];
  uint16_t packet_counter = 0;

  /* Position in current packet where next samples will be written */
  int packet_sample_offset = 0;

  LOG_INF("Microphone streaming thread started");

  while (1) {
    /* Wait for start signal */
    k_sem_take(&mic_start_sem, K_FOREVER);

    LOG_INF("Starting microphone capture");
    mic_state = MIC_STATE_STARTING;

    /* Configure the DMIC */
    ret = dmic_configure(dmic_dev, &mic_cfg);
    if (ret < 0) {
      LOG_ERR("Failed to configure DMIC: %d", ret);
      mic_state = MIC_STATE_ERROR;
      continue;
    }

    if (sync_is_active()) {
      LOG_INF("Mic ready, waiting at sync barrier...");
      ret = sync_wait(SYNC_SUBSYSTEM_MIC, 5000);
      if (ret != 0) {
        LOG_ERR("Sync wait failed: %d", ret);
        mic_state = MIC_STATE_ERROR;
        continue;
      }
    }
    /* Start the DMIC */
    ret = dmic_trigger(dmic_dev, DMIC_TRIGGER_START);
    if (ret < 0) {
      LOG_ERR("Failed to start DMIC: %d", ret);
      mic_state = MIC_STATE_ERROR;
      continue;
    }

    mic_state = MIC_STATE_STREAMING;
    packet_counter = 0;
    packet_sample_offset = 0;

    /* Initialize packet header */
    ble_packet[0] = MIC_DATA_HEADER;

    LOG_INF("Microphone streaming active at %u Hz", mic_cfg.streams[0].pcm_rate);

    /* Streaming loop */
    while (mic_keep_running) {
      void *buffer;
      uint32_t size;

      ret = dmic_read(dmic_dev, 0, &buffer, &size, MIC_READ_TIMEOUT);
      if (ret < 0) {
        LOG_ERR("DMIC read failed: %d", ret);
        break;
      }

      /* Process audio data and send over BLE */
      int16_t *samples = (int16_t *)buffer;
      int num_samples = size / MIC_BYTES_PER_SAMPLE;
      int sample_idx = 0;

      while (sample_idx < num_samples) {
        /* Calculate how many samples we can fit in current packet */
        int samples_remaining_in_packet = MIC_SAMPLES_PER_PACKET - packet_sample_offset;
        int samples_to_copy = MIN(samples_remaining_in_packet, num_samples - sample_idx);

        /* Copy samples to packet (offset by 3 for header + counter) */
        memcpy(&ble_packet[3 + (packet_sample_offset * MIC_BYTES_PER_SAMPLE)], &samples[sample_idx],
               samples_to_copy * MIC_BYTES_PER_SAMPLE);

        packet_sample_offset += samples_to_copy;
        sample_idx += samples_to_copy;

        /* Check if packet is full */
        if (packet_sample_offset >= MIC_SAMPLES_PER_PACKET) {
          /* Complete the packet */
          ble_packet[1] = (uint8_t)(packet_counter & 0xFF);
          ble_packet[2] = (uint8_t)((packet_counter >> 8) & 0xFF);

          ble_packet[MIC_PCKT_SIZE - 1] = MIC_DATA_TRAILER;

          /* Send via BLE queue */
          add_data_to_send_buffer(ble_packet, MIC_PCKT_SIZE);

          /* Prepare next packet */
          packet_counter++;
          packet_sample_offset = 0;
          ble_packet[0] = MIC_DATA_HEADER;
        }
      }

      /* Free the buffer */
      k_mem_slab_free(&mic_mem_slab, buffer);
    }

    /* Stop the DMIC */
    mic_state = MIC_STATE_STOPPING;
    ret = dmic_trigger(dmic_dev, DMIC_TRIGGER_STOP);
    if (ret < 0) {
      LOG_ERR("Failed to stop DMIC: %d", ret);
    }

    mic_state = MIC_STATE_IDLE;
    LOG_INF("Microphone streaming stopped");
  }
}

/* Define the microphone thread */
K_THREAD_DEFINE(mic_thread_id, MIC_THREAD_STACK_SIZE, mic_streaming_thread, NULL, NULL, NULL, MIC_THREAD_PRIORITY, 0,
                0);

/*==============================================================================
 * Public Functions
 *============================================================================*/

int mic_init(void) {
  /* Get the DMIC device */
  dmic_dev = DEVICE_DT_GET(DT_NODELABEL(dmic_dev));
  if (!device_is_ready(dmic_dev)) {
    LOG_ERR("DMIC device not ready");
    return -ENODEV;
  }

  /* Configure for mono capture */
  mic_cfg.channel.req_num_chan = 1;
  mic_cfg.channel.req_chan_map_lo = dmic_build_channel_map(0, 0, PDM_CHAN_LEFT);

  /* Set sample rate and block size */
  mic_cfg.streams[0].pcm_rate = MIC_MAX_SAMPLE_RATE;
  mic_cfg.streams[0].block_size = BLOCK_SIZE(mic_cfg.streams[0].pcm_rate, mic_cfg.channel.req_num_chan);

  LOG_INF("Microphone initialized (sample rate: %u Hz)", MIC_MAX_SAMPLE_RATE);
  return 0;
}

int mic_start_streaming(void) {
  if (mic_state == MIC_STATE_STREAMING) {
    LOG_WRN("Microphone already streaming");
    return -EALREADY;
  }

  if (mic_state != MIC_STATE_IDLE) {
    LOG_ERR("Microphone not in idle state");
    return -EBUSY;
  }

  mic_keep_running = true;
  k_sem_give(&mic_start_sem);

  return 0;
}

int mic_stop_streaming(void) {
  if (mic_state != MIC_STATE_STREAMING) {
    LOG_WRN("Microphone not streaming");
    return -EINVAL;
  }

  mic_keep_running = false;

  /* Wait for the streaming thread to stop */
  int timeout = 100; /* 1 second timeout */
  while (mic_state != MIC_STATE_IDLE && timeout > 0) {
    k_msleep(10);
    timeout--;
  }

  if (timeout == 0) {
    LOG_ERR("Timeout waiting for microphone to stop");
    return -ETIMEDOUT;
  }

  return 0;
}

mic_state_t mic_get_state(void) { return mic_state; }

bool mic_is_streaming(void) { return (mic_state == MIC_STATE_STREAMING); }
