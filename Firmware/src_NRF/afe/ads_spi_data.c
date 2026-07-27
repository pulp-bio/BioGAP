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
 * This module handles data acquisition from ADS1298 devices and constructs
 * BLE packets for transmission.
 */

/* Zephyr RTOS headers */
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>

/* Application headers */
#include "afe/ads_appl.h"
#include "afe/ads_defs.h"
#include "afe/ads_spi_data.h"
#include "ble/ble_appl.h"
#include "ads_spi_comm.h"
#if defined(CONFIG_WI_FI)
#include "wifi_sd_shield/wifi_sd_shield_appl.h"
#endif

/* Inter-board synchronization */
#include "core/board_sync.h"

LOG_MODULE_REGISTER(ads_spi_data, LOG_LEVEL_INF);

/*==============================================================================
 * External Variables
 *============================================================================*/

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

/**
 * @brief Timestamp (µs) of the first sample in the current packet
 *
 * Captured when the first sample of a packet arrives (first DRDY) so the
 * packet timestamp references the first sample: sample_i ≈ ts + i / fs.
 */
static uint32_t exg_packet_timestamp = 0;

/**
 * @brief Per-device sample counters within the current packet
 *
 * ADS1298_A and ADS1298_B are read independently (own DRDY each), so each
 * tracks its own write position into ble_tx_buf instead of sharing tx_buf_inx.
 * Reset to 0 once both reach EXG_SAMPLES_PER_PACKET and the packet is sent.
 */
static uint8_t ads_a_counter = 0;
static uint8_t ads_b_counter = 0;

/**
 * @brief Cycle timestamp of each device's most recent DRDY assert
 *
 * Set in the DRDY ISR, read back in process_ads_data() to measure how long
 * it actually took to service that DRDY -- used to diagnose whether B's
 * irregular cadence is a real ADS1298 overrun (DRDY not serviced before the
 * next conversion) or something else.
 */
static uint32_t ads_a_drdy_cycles = 0;
static uint32_t ads_b_drdy_cycles = 0;

/** @brief DRDY-to-serviced latency above which we log a warning (us) */
#define ADS_DRDY_LATENCY_WARN_US 500

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
 * @brief Data ready interrupt flags
 *
 * Set independently by each device's own DRDY GPIO interrupt when new ADC
 * data is available. Cleared by process_ads_data() after reading.
 */
volatile bool ads_a_data_ready = false;
volatile bool ads_b_data_ready = false;

/**
 * @brief DRDY serviced flags
 *
 * Tracks whether the previous DRDY interrupt for each device was serviced.
 * If false when a new DRDY arrives for that device, indicates data overrun
 * and acquisition is stopped.
 */
bool ads_a_served = true;
bool ads_b_served = true;

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
    if (ads_to_read == ADS1298_A) {
      /* Timestamp the first sample of each packet: ads_a_counter == 0 means
       * this is the first A sample of a fresh packet. */
      if (ads_a_counter == 0) {
        exg_packet_timestamp = k_cyc_to_us_floor32(k_cycle_get_32());
      }
      uint32_t offset = EXG_SAMPLE_DATA_START + EXG_BYTES_PER_SAMPLE * ads_a_counter;
      memcpy(&ble_tx_buf[offset], &ads_rx_buf[3], EXG_ADS_BLOCK_BYTES);
      ads_a_counter++;
      ads_a_served = true;
    } else { // ADS1298_B
      uint32_t offset = EXG_SAMPLE_DATA_START + EXG_ADS_BLOCK_BYTES +
                         EXG_BYTES_PER_SAMPLE * ads_b_counter;
      memcpy(&ble_tx_buf[offset], &ads_rx_buf[3], EXG_ADS_BLOCK_BYTES);
      ble_tx_buf[offset + EXG_ADS_BLOCK_BYTES] = counter_extra; // add here your custom data for each sample.
      ble_tx_buf[offset + EXG_ADS_BLOCK_BYTES + 1] = 0x00; // Reserved byte for future use
      ads_b_counter++;
      ads_b_served = true;
    }

    // Packet is ready once both devices have contributed all their samples --
    // this is the signal that hands the packet off to BLE/ESP.
    if (ads_a_counter == EXG_SAMPLES_PER_PACKET && ads_b_counter == EXG_SAMPLES_PER_PACKET) {
      ads_a_counter = 0;
      ads_b_counter = 0;

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

        // Add timestamp (microseconds) captured at the first sample of this packet
        uint32_t timestamp_us = exg_packet_timestamp;
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

#if defined(CONFIG_WI_FI)
        //LOG_INF("Adding data to ESP send buffer");
        add_data_to_esp_send_buffer(ble_tx_buf, EXG_PCK_LNGTH);
#else
        LOG_INF("Adding data to BLE send buffer");
        add_data_to_send_buffer(ble_tx_buf, EXG_PCK_LNGTH);
#endif
      }
    }
  }
  spi_xfer_done = true;
  LOG_DBG("Setting spi_xfer_done to true");
}

/**
 * @brief DRDY interrupt callback for ADS1298_A
 *
 * Signals that new data is available and increments debug counter.
 */
void ads_drdy_callback_a(void) {
  /* Signal that new data is available */
  ads_a_data_ready = true;
  ads_a_drdy_cycles = k_cycle_get_32();
  /* Increment debug counter for timing analysis */
  counter_extra = counter_extra + 1;
  // LOG_INF("ADS DRDY interrupt");
}

/**
 * @brief DRDY interrupt callback for ADS1298_B
 *
 * Signals that new data is available on ADS1298_B.
 */
void ads_drdy_callback_b(void) {
  ads_b_data_ready = true;
  ads_b_drdy_cycles = k_cycle_get_32();
  // LOG_INF("ADS B DRDY interrupt");
}

/**
 * @brief Process ADS1298 data once both DRDYs have fired
 *
 * Main data acquisition handler called from application main loop.
 * ADS1298_A and ADS1298_B each have their own independent DRDY interrupt,
 * but both are held until the other has also signaled ready -- neither
 * device's SPI interface is touched until both are confirmed ready.
 */
void process_ads_data(void) {
  /* Wait until BOTH devices have signaled their own DRDY before touching
   * either one's SPI interface -- unlike servicing each independently the
   * moment its own flag sets, this guarantees we never assert CS/clock a
   * device while its internal conversion could still be in progress. */
  if (ads_a_data_ready && ads_b_data_ready) {
    ads_a_data_ready = false;
    ads_b_data_ready = false;

    if (ads_get_function() == ADS_READ) {
      if (!ads_a_served || !ads_b_served) {
        ads_set_function(ADS_STOP);
      } else {
        ads_a_served = false;
        spi_xfer_done = false;
        ads_to_read = ADS1298_A;
        ads1298_read_samples(ads_rx_buf, 27, ADS1298_A);
        while (spi_xfer_done == false)
          ;

        ads_b_served = false;
        spi_xfer_done = false;
        ads_to_read = ADS1298_B;
        ads1298_read_samples(ads_rx_buf, 27, ADS1298_B);
        while (spi_xfer_done == false)
          ;
      }
    }

    uint32_t now = k_cycle_get_32();
    uint32_t a_latency_us = k_cyc_to_us_floor32(now - ads_a_drdy_cycles);
    uint32_t b_latency_us = k_cyc_to_us_floor32(now - ads_b_drdy_cycles);
    if (a_latency_us > ADS_DRDY_LATENCY_WARN_US) {
      LOG_WRN("A DRDY serviced after %u us (waited for B)", a_latency_us);
    }
    if (b_latency_us > ADS_DRDY_LATENCY_WARN_US) {
      LOG_WRN("B DRDY serviced after %u us (waited for A)", b_latency_us);
    }
  }
  k_sleep(K_USEC(1));
}