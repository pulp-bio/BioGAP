/*
 * ----------------------------------------------------------------------
 *
 * File: ads_spi_config.c
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
 * @file ads_spi_config.c
 * @brief ADS1298 Device Configuration and Initialization
 *
 * Functions for configuring ADS1298 registers, verifying device ID,
 * and managing device initialization sequence.
 */

/* Zephyr RTOS headers */
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>

#include "afe/ads_appl.h"
#include "afe/ads_defs.h"
#include "afe/ads_spi.h"
#include "afe/ads_spi_comm.h"
#include "afe/ads_spi_data.h"
#include "afe/ads_spi_hw.h"

LOG_MODULE_REGISTER(ads_spi_config, LOG_LEVEL_INF);

/*==============================================================================
 * External Variables
 *============================================================================*/

/** @brief SPI command/data buffer */
extern uint8_t pr_word[10];

/** @brief SPI transfer complete flag */
extern volatile bool spi_xfer_done;

/** @brief ADS1298 initialization status */
extern bool ads_initialized;

/** @brief BLE transmission packet buffer */
extern uint8_t ble_tx_buf[EXG_PCK_LNGTH];

/** @brief Current write index in BLE packet buffer */
extern uint32_t tx_buf_inx;

/** @brief BLE packet counter */
extern uint16_t counter;

/*==============================================================================
 * Public Functions - Device Verification
 *============================================================================*/

/**
 * @brief Verify ADS1298 device ID
 *
 * Performs device identification to ensure proper SPI communication and
 * correct device population. The ADS1298 ID register should read 0xD2.
 *
 * Sequence:
 * 1. Send RESET command to restore default registers
 * 2. Send SDATAC to stop continuous data mode
 * 3. Read ID register (address 0x00)
 * 4. Verify ID value is 0xD2
 *
 * @param[in] ads_id Device to verify (ADS1298_A or ADS1298_B)
 *
 * @note If ID check fails, function enters infinite loop with error logging
 * @note 30ms delays allow device to complete operations per datasheet timing
 */
void ads_check_id(ads_device_id_t ads_id) {

  // RESET DEVICE
  pr_word[0] = _RESET;
  ads1298_write_spi(1, ads_id);
  while (spi_xfer_done == false)
    ;
  k_msleep(30);

  // STOP DEVICE
  pr_word[0] = _SDATAC;
  ads1298_write_spi(1, ads_id);
  while (spi_xfer_done == false)
    ;
  k_msleep(30);

  // Read out device ID
  pr_word[0] = _RREG | ID;
  pr_word[1] = 0;
  ads1298_read_spi(ads_rx_buf, 3, ads_id);
  k_msleep(30);

  if (ads_rx_buf[2] != 0xd2) {
    // Wait here if ID is not correct
    while (1) {
      LOG_ERR("ADS1298 ID not correct");
    }
  }
  LOG_DBG("ADS1298 id checked");
}

/*==============================================================================
 * Public Functions - Device Configuration
 *============================================================================*/

/**
 * @brief Initialize ADS1298 device with specified parameters
 *
 * Performs complete device initialization sequence:
 * 1. Reset device to default state
 * 2. Stop continuous data mode
 * 3. Configure data rate and reference settings (CONFIG1-CONFIG3)
 * 4. Configure all 8 channel settings (gain, input type)
 * 5. Initialize BLE packet buffer
 *
 * @param[in] InitParams Configuration parameter array:
 *   - [0]: Data rate code (0=16kSPS to 6=250SPS)
 *        Common values: 2=1kSPS, 3=500SPS
 *   - [1]: Channel input configuration
 *        0x00=Normal electrode input
 *        0x01=Shorted input (offset calibration)
 *        0x05=Test signal
 *   - [2]: Reserved (not used)
 *   - [3]: Reserved (not used)
 *   - [4]: Channel gain code
 *        0x00=Gain 6, 0x10=Gain 1, 0x20=Gain 2, etc.
 *
 * @param[in] ads_id Device to initialize (ADS1298_A or ADS1298_B)
 *
 * Register Configuration Details:
 * - CONFIG1: Data rate, CLK connection, daisy-chain disable
 * - CONFIG2: Test signal configuration, reference buffer
 * - CONFIG3: Internal reference, RLD buffer, bias settings
 * - CH1SET-CH8SET: Per-channel power-down, gain, and input MUX
 *
 * @note Function blocks waiting for SPI transfers to complete
 * @note 30ms delays allow device to complete configuration per datasheet
 * @note Sets ads_initialized flag when complete
 */
void ads_init(uint8_t *InitParams, ads_device_id_t ads_id) {

  // buffer_counter = 0;
  tx_buf_inx = 0;
  ble_tx_buf[tx_buf_inx++] = BLE_PCK_HEADER;
  ble_tx_buf[tx_buf_inx++] = (uint8_t)(++counter);
  ble_tx_buf[tx_buf_inx++] = (uint8_t)(counter >> 8);
  // Reserve 4 bytes for timestamp (will be filled when packet is complete)
  tx_buf_inx += 4;

  // RESET DEVICE
  pr_word[0] = _RESET;
  spi_xfer_done = false;
  ads1298_write_spi(1, ads_id);
  while (spi_xfer_done == false)
    ;
  k_msleep(30);

  // STOP DEVICE
  pr_word[0] = _SDATAC;
  spi_xfer_done = false;
  ads1298_write_spi(1, ads_id);
  while (spi_xfer_done == false)
    ;
  k_msleep(30);

  pr_word[0] = _WREG | CONFIG1;
  pr_word[1] = 2;
  pr_word[2] = 0xC0 + InitParams[0];
  pr_word[3] = 0x55;
  pr_word[4] = 0xC0;

  spi_xfer_done = false;
  ads1298_write_spi(5, ads_id);
  while (spi_xfer_done == false)
    ;

  // SET CHANNEL REGS - ON, GAIN 6, SHORTED ELECTRODE INPUT
  pr_word[0] = _WREG | CH1SET;
  pr_word[1] = 7;

  for (int i = 2; i < 10; i++) {
    pr_word[i] = InitParams[4] | InitParams[1];
  }

  spi_xfer_done = false;
  ads1298_write_spi(10, ads_id);
  while (spi_xfer_done == false)
    ;
  k_msleep(30);

  ads_initialized = true;
}

/*==============================================================================
 * Public Functions - Acquisition Control
 *============================================================================*/

/**
 * @brief Stop data acquisition on both ADS1298 devices
 *
 * Sends SDATAC (Stop Data Continuous) command to both devices.
 * This halts conversion and allows register access.
 *
 * @note Does not power down the device, just stops conversions
 * @note Both devices must be stopped for synchronized operation
 */
void ads_stop() {
  pr_word[0] = _SDATAC;
  spi_xfer_done = false;
  LOG_DBG("Stopping ADS1298 A");
  ads1298_write_spi(1, ADS1298_A);
  LOG_DBG("Waiting for ADS1298 A to stop");
  while (spi_xfer_done == false)
    ;
  LOG_DBG("ADS1298 A stopped");
  LOG_DBG("waited 30ms after stopping ADS1298 A");

  pr_word[0] = _SDATAC;
  spi_xfer_done = false;
  ads1298_write_spi(1, ADS1298_B);
  while (spi_xfer_done == false)
    ;
}

/**
 * @brief Start synchronized data acquisition on both ADS1298 devices
 *
 * Sends START and RDATAC commands to both devices in sequence.
 * The devices begin converting and streaming data immediately.
 *
 * Command sequence per device:
 * 1. START (0x08): Begin conversions
 * 2. RDATAC (0x10): Enable continuous data read mode
 *
 * In RDATAC mode, data is automatically clocked out on each DRDY pulse
 * without needing to send read commands.
 *
 * @note Devices are synchronized via shared START pin (if used)
 * @note DRDY interrupts will begin firing once conversion starts
 */
void ads_start() {
  pr_word[0] = _START;
  pr_word[1] = _RDATAC;
  spi_xfer_done = false;
  ads1298_write_spi(2, ADS1298_A);
  while (spi_xfer_done == false)
    ;

  pr_word[0] = _START;
  pr_word[1] = _RDATAC;
  spi_xfer_done = false;
  ads1298_write_spi(2, ADS1298_B);
  while (spi_xfer_done == false)
    ;
}