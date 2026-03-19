/*
 * ----------------------------------------------------------------------
 *
 * File: ads_spi_data.h
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

#ifndef ADS_SPI_DATA_H
#define ADS_SPI_DATA_H

#include <stdint.h>
#include <stdbool.h>
#include "afe/ads_defs.h"

/*==============================================================================
 * Module Variables - BLE Packet Construction
 *============================================================================*/

/**
 * @brief BLE transmission packet buffer
 *
 * Accumulates 7 samples into a complete 234-byte packet before transmission.
 * Format documented in ads_spi.h header.
 */
extern uint8_t ble_tx_buf[EXG_PCK_LNGTH];

/**
 * @brief Current write index in BLE packet buffer
 *
 * Tracks the next position to write sample data. Reset to 0 when packet
 * is complete and ready for transmission.
 */
extern uint32_t tx_buf_inx;

/**
 * @brief BLE packet counter
 *
 * Increments with each transmitted packet. Wraps around at 65535. Used by
 * receiver to detect packet loss.
 */
extern uint16_t counter;

/**
 * @brief Extra counter for debugging/custom data
 *
 * Increments on each DRDY interrupt. Can be read out by MATLAB conversion
 * scripts for timing analysis.
 */
extern uint8_t counter_extra;

/*==============================================================================
 * Module Variables - State Flags
 *============================================================================*/

/**
 * @brief SPI transfer complete flag
 *
 * Set by SPI interrupt handler when transfer completes. Polled by
 * blocking code to wait for completion.
 */
extern volatile bool spi_xfer_done;

/**
 * @brief Data ready interrupt flag
 *
 * Set by DRDY GPIO interrupt when new ADC data is available. Cleared
 * by process_ads_data() after reading.
 */
extern volatile bool ads_data_ready;

/**
 * @brief DRDY serviced flag
 *
 * Tracks whether the previous DRDY interrupt was serviced. If false when
 * new DRDY arrives, indicates data overrun and acquisition is stopped.
 */
extern bool drdy_served;

/**
 * @brief BLE packet ready flag
 *
 * Indicates previous packet has not been transmitted yet. If true when
 * completing a new packet, acquisition is stopped to prevent buffer overflow.
 */
extern bool pck_ble_ready;

/**
 * @brief Current ADS device being read
 *
 * Tracks which device (A or B) is currently being read in the SPI handler.
 * Used to properly route data in the interrupt callback.
 */
extern volatile bool ads_to_read;

/*==============================================================================
 * Function Declarations
 *============================================================================*/

/**
 * @brief SPI interrupt handler continuation
 *
 * Called from SPIM handler to process received data and construct BLE packets.
 */
void ads_spim_handler_done(void);

/**
 * @brief DRDY interrupt callback
 *
 * Called from GPIO interrupt to signal new data availability.
 */
void ads_drdy_callback(void);

/**
 * @brief Process ADS1298 data when DRDY interrupt occurs
 *
 * Main data acquisition handler called from application main loop.
 */
void process_ads_data(void);

#endif // ADS_SPI_DATA_H