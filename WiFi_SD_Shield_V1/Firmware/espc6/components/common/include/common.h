/*
 * ----------------------------------------------------------------------
 *
 * File: common.h
 *
 * Last edited: 17.07.2026
 *
 * Copyright (c) 2026 ETH Zurich and University of Bologna
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

#pragma once

#include <stdint.h>
#include <stdbool.h>

// FreeRTOS / ESP-IDF
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "freertos/ringbuf.h"
#include "freertos/semphr.h"
#include "esp_timer.h"
// GPIO
#include "driver/gpio.h"
// SPI
#include "driver/spi_master.h"
#include "driver/spi_slave.h"
#include "pin_definitions.h"
#include "shield_config.h"


// ---------------------- AP State Machine ----------------------------

typedef enum {
    STATE_DISCONNECTED,  // Initial State, waiting to verify TCP connection and BIOGAP communication
    STATE_IDLE  ,        // TCP and BIOGAP comms verified, waiting to start streaming
    STATE_STREAMING,     // Streaming active/expected
} node_state_t;

extern volatile node_state_t node_state;
extern int64_t start_time;

// -------------------- Event bits --------------------
extern EventGroupHandle_t g_evt;
extern SemaphoreHandle_t spi_bus_mutex;

#define SPI_BUS_LOCK(timeout_ticks) (xSemaphoreTake(spi_bus_mutex, (timeout_ticks)) == pdTRUE)
#define SPI_BUS_UNLOCK() xSemaphoreGive(spi_bus_mutex)

// These are for Wi-Fi Streaming
#define B_BIOGAP_CONECTED               (1U << 0)  // Connection with BIOGAP master verified by successful SPI transaction
#define B_WIFI_CONNECTED                (1U << 1)  // TCP connected (socket usable)
#define B_GUI_SOCKET_BIND               (1U << 2)  // GUI socket successfully bound and accepted (gui_sock valid)
#define B_START_CMD_RCV                 (1U << 3)  // Start command received from GUI (via TCP)
#define B_START_CMD_FWD_TO_BIOGAP       (1U << 4)  // Start command forwarded to BIOGAP master successfully
#define B_STOP_CMD_RCV_GUI              (1U << 5)  // Stop command received from GUI (via TCP)
#define B_STOP_CMD_FWD_TO_BIOGAP        (1U << 6)  // Stop command forwarded to BIOGAP master successfully
#define B_RINGBUFFER_FULL               (1U << 7)  // Ringbuffer is full, cannot add more data until some is consumed
// SPI quiesce handshake: reader sets this when outstanding pre-queued descriptors
// have been drained and the SPI bus is safe for a STOP-frame transmit by sender.
#define B_SPI_QUIESCED                  (1U << 8)
#define B_STOP_CMD_RCV_FORCED           (1U << 9)  // Stop command received from GUI (via TCP)
#define B_STOP_CMD_RPT_PENDING          (1U << 10)  // GUI task should report the simulated STOP event


/* Handshake ping-pong markers to validate NRF-ESP communication */
#define HANDSHAKE_MARKER 0x5A
#define HANDSHAKE_RESPONSE_MARKER 0xA5

// -------------------- Task stack sizes --------------------
#define READ_FROM_BIOGAP_STACK_SIZE  4096

// -------------------- Packet format --------------------
#define PAYLOAD_SIZE                 1440    // application payload target for WiFi transmission, can be adjusted
#define HEADER_BYTES                 3
#define COUNTER_BYTES                4
#define TSF_BYTES                    8
#define NUM_SAMPLES_BYTES            2

#define PACKET_DEF_BYTES             (HEADER_BYTES + COUNTER_BYTES + TSF_BYTES + NUM_SAMPLES_BYTES)
#define SPACE_RESERVED_FOR_MASTER    3
#define CRC_BYTES                    0

#define EFFECTIVE_MAX_PAYLOAD        (PAYLOAD_SIZE - PACKET_DEF_BYTES - SPACE_RESERVED_FOR_MASTER - CRC_BYTES)

// NOTE: this is currently enforced by prepare_buffer(). Keep consistent with builder math.
#define EXPECTED_DATA_LEN            1436

// -------------------- Streaming / timing --------------------
#define NODE_FRAME_TAG               0x0A    // high nibble for node frame

// -------------------- Ringbuffer --------------------
#define RINGBUFF_SIZE                200000  // bytes
extern RingbufHandle_t ringbuff;

// -------------------- Constants for NRF-ESP SPI transfer and packet building --------------------
#define SPI_FROM_BIOGAP_MAX_SIZE    1000   // max number of bytes in a single SPI transaction from BIOGAP (e.g. 410 if streaming from WULPUS)

//--------------------- SPI Transactions & Mode Switching --------
// Single SPI host constant used for both NRF and SD card modes (runtime switching)
#define SPI_HOST_DEVICE SPI2_HOST

// SPI bus ownership modes
typedef enum {
    SPI_MODE_IDLE = 0,
    SPI_MODE_NRF = 1,
    SPI_MODE_SD = 2
} spi_mode_t;

extern spi_device_handle_t nrf_spi_device;
extern spi_mode_t current_spi_mode;

esp_err_t config_spi_nrf_master_esp_slave_pins(void);
esp_err_t init_nrf_spi_master_esp_slave_bus(void); 
extern bool handshake_pq_done;
extern bool send_start_command_to_biogap_master;

//---------------------- Logging tags ------------------------
// Set to 1 only when actively profiling SPI mode-switch overhead.
#ifndef ENABLE_SPI_PROFILE_LOGS
#define ENABLE_SPI_PROFILE_LOGS 0
#endif

// 0: suppress INFO logs (default, faster runtime)
// 1: enable INFO logs for debugging/profiling
#ifndef ESP_ENABLE_INFO_LOGS
#define ESP_ENABLE_INFO_LOGS 1
#endif
