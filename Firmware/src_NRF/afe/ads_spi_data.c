/*
 * ----------------------------------------------------------------------
 *
 * File: ads_spi_data.c
 *
 * Last edited: 23.07.2025
 *
 * Copyright (C) 2025, ETH Zurich
 *
 * Authors:
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
 * @file ads_spi_data.c
 * @brief ADS1298 Data Processing and BLE Packet Construction
 *
 * This module handles data acquisition from ADS1298 devices, combines data
 * with PPG sensor readings, and constructs BLE packets for transmission.
 */

/* Zephyr RTOS headers */
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>

/* Application headers */
#include "afe/ads_appl.h"
#include "afe/ads_defs.h"
#include "afe/ads_spi_data.h"
#include "ble/ble_appl.h"
#include "sensors/ppg/ppg_appl.h"
#include "ads_spi_comm.h"

/* Inter-board synchronization */
#include "core/board_sync.h"

LOG_MODULE_REGISTER(ads_spi_data, LOG_LEVEL_INF);

/*==============================================================================
 * External Variables
 *============================================================================*/

/**
 * @brief PPG sensor data structure
 *
 * Defined in ppg_appl.c. Contains circular buffer of red and IR LED readings.
 */
extern sense_struct sense;

/**
 * @brief SPI receive buffer for ADS1298 data
 *
 * Receives data from ADS1298 during SPI transfers. Oversized to 40 bytes
 * to accommodate potential extended transfers.
 */
extern uint8_t ads_rx_buf[40];

/*==============================================================================
 * Module Variables - BLE Packet Construction
 *============================================================================*/

/**
 * @brief BLE transmission packet buffer
 *
 * Accumulates 7 samples into a complete 234-byte packet before transmission.
 * Format documented in ads_spi.h header.
 */
uint8_t ble_tx_buf[EXG_PCK_LNGTH] = {0};

/**
 * @brief Current write index in BLE packet buffer
 *
 * Tracks the next position to write sample data. Reset to 0 when packet
 * is complete and ready for transmission.
 */
uint32_t tx_buf_inx = 0;

/**
 * @brief BLE packet counter
 *
 * Increments with each transmitted packet. Wraps around at 255. Used by
 * receiver to detect packet loss.
 */
uint16_t counter = 0;

/**
 * @brief Extra counter for debugging/custom data
 *
 * Increments on each DRDY interrupt. Can be read out by MATLAB conversion
 * scripts for timing analysis.
 */
uint8_t counter_extra = 0;

/*==============================================================================
 * Module Variables - State Flags
 *============================================================================*/

/**
 * @brief SPI transfer complete flag
 *
 * Set by SPI interrupt handler when transfer completes. Polled by
 * blocking code to wait for completion.
 */
volatile bool spi_xfer_done = true;

/**
 * @brief Data ready interrupt flag
 *
 * Set by DRDY GPIO interrupt when new ADC data is available. Cleared
 * by process_ads_data() after reading.
 */
volatile bool ads_data_ready = false;

/**
 * @brief DRDY serviced flag
 *
 * Tracks whether the previous DRDY interrupt was serviced. If false when
 * new DRDY arrives, indicates data overrun and acquisition is stopped.
 */
bool drdy_served = true;

/**
 * @brief BLE packet ready flag
 *
 * Indicates previous packet has not been transmitted yet. If true when
 * completing a new packet, acquisition is stopped to prevent buffer overflow.
 */
bool pck_ble_ready = false;

/**
 * @brief Current ADS device being read
 *
 * Tracks which device (A or B) is currently being read in the SPI handler.
 * Used to properly route data in the interrupt callback.
 */
volatile bool ads_to_read = ADS1298_A;

/*==============================================================================
 * Public Functions - Data Processing
 *============================================================================*/

/**
 * @brief SPI interrupt handler continuation
 *
 * Processes received data and constructs BLE packets from ADS/PPG data.
 * Called from the SPIM hardware interrupt handler.
 */
void ads_spim_handler_done(void) {
  if (ads_get_function() == ADS_READ) {
    memcpy(&ble_tx_buf[tx_buf_inx], &ads_rx_buf[3], 24);
    tx_buf_inx += 24;

    if (Get_PPG_Function() == PPG_ACTIVE) {
      // Replace last two EEG channels with PPG data (IR and red LED)
      tx_buf_inx -= 6;

      // Get the pointers to the start of the most recent PPG data
      uint8_t *address_red = (uint8_t *)&sense.red[sense.head];
      uint8_t *address_IR = (uint8_t *)&sense.IR[sense.head];
      // Add the PPG data such that the Biowolf GUI interprets it correctly
      memcpy(&ble_tx_buf[tx_buf_inx], address_red + 2, 1);
      tx_buf_inx++;
      memcpy(&ble_tx_buf[tx_buf_inx], address_red + 1, 1);
      tx_buf_inx++;
      memcpy(&ble_tx_buf[tx_buf_inx], address_red + 0, 1);
      tx_buf_inx++;

      memcpy(&ble_tx_buf[tx_buf_inx], address_IR + 2, 1);
      tx_buf_inx++;
      memcpy(&ble_tx_buf[tx_buf_inx], address_IR + 1, 1);
      tx_buf_inx++;
      memcpy(&ble_tx_buf[tx_buf_inx], address_IR + 0, 1);
      tx_buf_inx++;
    }

    if (ads_to_read == ADS1298_B) {
      //  Set packet identifier to EEG all channels + PPG inactive
      ble_tx_buf[tx_buf_inx++] = counter_extra; // add here your custom data for each sample.
      ble_tx_buf[tx_buf_inx++] = 0x00; // Reserved byte for future use
    }

    if (tx_buf_inx == EXG_SAMPLE_DATA_END) {
      // Check if the last pck was handled and reset flag

      if (pck_ble_ready == true) {
        ads_set_function(ADS_STOP);
        pck_ble_ready = false;
        LOG_INF("Data packet not processed -- stop ADS");
      } else {

        // Reset and condition BLE buffers
        tx_buf_inx = 0;

        counter++;
        // Prepare the buffer with header, counter, and timestamp
        ble_tx_buf[tx_buf_inx++] = BLE_PCK_HEADER;
        ble_tx_buf[tx_buf_inx++] = (uint8_t)(counter);
        ble_tx_buf[tx_buf_inx++] = (uint8_t)(counter >> 8);

        // Add timestamp (microseconds) for cross-packet synchronization
        uint32_t timestamp_us = k_cyc_to_us_floor32(k_cycle_get_32());
        ble_tx_buf[tx_buf_inx++] = (uint8_t)(timestamp_us & 0xFF);
        ble_tx_buf[tx_buf_inx++] = (uint8_t)((timestamp_us >> 8) & 0xFF);
        ble_tx_buf[tx_buf_inx++] = (uint8_t)((timestamp_us >> 16) & 0xFF);
        ble_tx_buf[tx_buf_inx++] = (uint8_t)((timestamp_us >> 24) & 0xFF);

        // Finish up and send
        // Get the index to write metadata
        int buf_current_size = EXG_SAMPLE_DATA_END;

        // Metadata bytes: board_id, sync_pulse_count, reserved
        ble_tx_buf[buf_current_size++] = board_sync_get_board_id();
        ble_tx_buf[buf_current_size++] = board_sync_get_pulse_count();
        ble_tx_buf[buf_current_size++] = 0x00;

        // BLE PCK tail
        ble_tx_buf[buf_current_size++] = BLE_PCK_TAILER;

        add_data_to_send_buffer(ble_tx_buf, EXG_PCK_LNGTH);
      }
    }

    drdy_served = true;
  }
  spi_xfer_done = true;
  LOG_DBG("Setting spi_xfer_done to true");
}

/**
 * @brief DRDY interrupt callback
 *
 * Signals that new data is available and increments debug counter.
 */
void ads_drdy_callback(void) {
  /* Signal that new data is available */
  ads_data_ready = true;
  /* Increment debug counter for timing analysis */
  counter_extra = counter_extra + 1;
  // LOG_INF("ADS DRDY interrupt");
}

/**
 * @brief Process ADS1298 data when DRDY interrupt occurs
 *
 * Main data acquisition handler called from application main loop.
 * Manages the complete data flow from ADS devices to BLE transmission.
 */
void process_ads_data(void) {
  // LOG_INF("Processing ADS data...");
  if (ads_data_ready) {
    // LOG_INF("ADS DATA READY interrupt received");
    // Clear flag first to avoid missing next interrupt
    ads_data_ready = false;

    // Process received data as needed
    if (ads_get_function() == ADS_READ) {
      if (!drdy_served) {
        ads_set_function(ADS_STOP);
      } else {
        drdy_served = false;

        spi_xfer_done = false;
        ads_to_read = ADS1298_A;
        ads1298_read_samples(ads_rx_buf, 27, ADS1298_A);
        while (spi_xfer_done == false)
          ;
        spi_xfer_done = false;
        ads_to_read = ADS1298_B;
        ads1298_read_samples(ads_rx_buf, 27, ADS1298_B);
        while (spi_xfer_done == false)
          ;
      }
    }
  }
  k_sleep(K_USEC(1));
}