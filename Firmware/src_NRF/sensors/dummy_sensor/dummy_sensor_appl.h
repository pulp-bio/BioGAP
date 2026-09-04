/*
 * ----------------------------------------------------------------------
 *
 * File: dummy_sensor.h
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
 * @file dummy_sensor_appl.h
 * @brief Dummy Sensor Application Interface
 *
 * This header contains the application-level interface to simulate a sensor.
 * It can be used for testing the data flow and integration of the system without requiring actual sensor hardware.
 * The dummy sensor will generate synthetic data at a configurable rate and send it through the same pathways as rwal sensors.
*/
#ifndef DUMMY_SENSOR_APPL_H
#define DUMMY_SENSOR_APPL_H

#include <zephyr/kernel.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/spi.h>
#include <zephyr/logging/log.h>
#include <string.h>
#include <errno.h>


#define EMULATE_EXG_DATA 1   /* Set to 1 to enable dummy producer thread that generates EXG-like packets for testing without real sensor input */
#define EMULATE_US_DATA 0       /* Set to 1 to enable dummy producer thread that generates US-like packets for testing without real sensor input */

#if (EMULATE_EXG_DATA == 1)

        /* Packet format constants (aligned with biogap firmware ads_defs.h) */
        #define DUMMY_SENSOR_PCKT_SIZE 211                                                       /* Total packet size (211 for EXG | 811 for US)*/
        #define DUMMY_SAMPLE_DATA_END 207                                                 /* 207 for EXG, 807 for US - Header(1) + Counter(2) + Timestamp(4) + 4×50forEXG - 800 for US */
        #define DUMMY_SAMPLE_HEADER_SIZE 7                                                 /* Header(1) + Counter(2) + Timestamp(4) */
        #define DUMMY_SAMPLES_PER_PACKET 4                                                /* 4 samples per packet for EXG - 1 for */
        #define DUMMY_BYTES_PER_SAMPLE 50                                                 /* 24 ADS_A + 24 ADS_B + counter + reserved */
        #define DUMMY_PAYLOAD_SIZE (DUMMY_SAMPLES_PER_PACKET * DUMMY_BYTES_PER_SAMPLE)         /* 200 bytes to aggregate */
        #define SENSOR_SAMPLING_PERIOD_MS   100             /* Timer period: one sample from ads every TRANSACTION_PERIOD_MS. One NRF -> ESP transactio occurs every SAMPLING_PERIOD_MS x DUMMY_SAMPLES_PER_PACKET */
#endif

#if (EMULATE_US_DATA == 1)
        /* Packet format constants (aligned with biogap firmware ads_defs.h) */
        #define DUMMY_SENSOR_PCKT_SIZE 811                                                       /* Total packet size (211 for EXG | 811 for US)*/
        #define DUMMY_SAMPLE_HEADER_SIZE 7                                                 /* Header(1) + Counter(2) + Timestamp(4) */
        #define DUMMY_SAMPLE_DATA_END 807                                                 /* 207 for EXG, 807 for US - Header(1) + Counter(2) + Timestamp(4) + 4×50forEXG - 800 for US */
        #define DUMMY_SAMPLES_PER_PACKET 1                                                /* 4 samples per packet for EXG - 1 for */
        #define DUMMY_BYTES_PER_SAMPLE 800                                                 /* 24 ADS_A + 24 ADS_B + counter + reserved */
        #define DUMMY_PAYLOAD_SIZE (DUMMY_SAMPLES_PER_PACKET * DUMMY_BYTES_PER_SAMPLE)         /* 200 bytes to aggregate */
        #define SENSOR_SAMPLING_PERIOD_MS   10             /* Timer period: one sample from ads every TRANSACTION_PERIOD_MS. One NRF -> ESP transactio occurs every SAMPLING_PERIOD_MS x DUMMY_SAMPLES_PER_PACKET */

#endif

#define DUMMY_SENSOR_HEADER 0x55
#define DUMMY_SENSOR_TAILER 0xAA

#define DUMMY_NRF_ESP_TRANSACTION_INTERVAL_MS SENSOR_SAMPLING_PERIOD_MS * DUMMY_SAMPLES_PER_PACKET   /* Expected interval between NRF-ESP transactions, used for logging */


#define DUMMY_SENSOR_THREAD_STACK_SIZE 2048
#define DUMMY_SENSOR_THREAD_PRIORITY 5


/*==============================================================================
 * Type Definitions
 *============================================================================*/

/**
 * @enum dummy_sensor_state_t
 * @brief Dummy sensor states
 */
typedef enum {
  DUMMY_SENSOR_STATE_IDLE,      /**< Dummy sensor idle, not streaming */
  DUMMY_SENSOR_STATE_STARTING,  /**< Dummy sensor initializing and configuring */
  DUMMY_SENSOR_STATE_STREAMING, /**< Dummy sensor actively streaming data */
  DUMMY_SENSOR_STATE_STOPPING,  /**< Dummy sensor stopping */
  DUMMY_SENSOR_STATE_ERROR      /**< Error state */
} dummy_sensor_state_t;


/** @brief Dummy Buffer to hold synthetic data samples before sending. */

extern uint8_t dummy_sensor_buf[DUMMY_SENSOR_PCKT_SIZE];
extern uint32_t chunk_counter; /* Counter to generate unique sample data */

/** @brief Function to produce dummy sensor data */
void fill_dummy_exg_sample(uint8_t *sample, uint32_t counter);

/** @brief Initialize the dummy sensor subsystem (state only; call once at boot) */
int dummy_sensor_init(void);

/** @brief Start the dummy sensor's periodic synthetic-data streaming */
int dummy_sensor_start_streaming(void);

/** @brief Stop the dummy sensor's streaming and wait for the thread to go idle */
int dummy_sensor_stop_streaming(void);

#endif // DUMMY_SENSOR_APPL_H