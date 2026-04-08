/*
 * ----------------------------------------------------------------------
 *
 * File: ads_spi_comm.c
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
 * @file ads_spi_comm.c
 * @brief ADS1298 SPI Communication Functions
 *
 * Low-level SPI communication functions for reading and writing to ADS1298 devices.
 */

/* Zephyr RTOS headers */
#include <nrfx_spim.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>

#include "afe/ads_appl.h"
#include "afe/ads_defs.h"
#include "afe/ads_spi_hw.h"
#include "afe/ads_spi_data.h"

LOG_MODULE_REGISTER(ads_spi_comm, LOG_LEVEL_INF);

/*==============================================================================
 * External Variables
 *============================================================================*/

/** @brief SPI command/data buffer */
extern uint8_t pr_word[10];

/** @brief SPI transfer complete flag */
extern volatile bool spi_xfer_done;

/*==============================================================================
 * Module Variables
 *============================================================================*/

/**
 * @brief Empty buffer for SPI reads
 *
 * Transmitted during data read operations. Size is 27 bytes:
 * - 3 status bytes
 * - 24 data bytes (8 channels × 3 bytes each)
 */
static uint8_t empty_buffer[27] = {0};

/**
 * @brief SPI receive buffer for ADS1298 data
 *
 * Receives data from ADS1298 during SPI transfers. Oversized to 40 bytes
 * to accommodate potential extended transfers.
 */
extern uint8_t ads_rx_buf[40];

/*==============================================================================
 * Private Functions - GPIO Control
 *============================================================================*/

/**
 * @brief Assert chip select for specified ADS device
 *
 * @param ads_id Device to select (ADS1298_A or ADS1298_B)
 * @return 0 on success, -1 on error
 */
static int ads_chip_select(ads_device_id_t ads_id) {
  // Implementation depends on hardware pins defined in ads_spi_hw.c
  extern const struct gpio_dt_spec gpio_dt_ads1298_a_cs;
  extern const struct gpio_dt_spec gpio_dt_ads1298_b_cs;

  if (ads_id == ADS1298_A) {
    if (gpio_pin_set_dt(&gpio_dt_ads1298_a_cs, 1) < 0) { // Set pin to enable
      LOG_ERR("ADS1298 power GPIO set error");
      return -1;
    }
  } else {
    if (gpio_pin_set_dt(&gpio_dt_ads1298_b_cs, 1) < 0) { // Set pin to enable
      LOG_ERR("ADS1298 power GPIO set error");
      return -1;
    }
  }
  return 0;
}

/*==============================================================================
 * Public Functions - Low-Level SPI Transfers
 *============================================================================*/

/**
 * @brief Perform SPI read transaction with command
 *
 * Executes a full-duplex SPI transfer, transmitting command bytes from pr_word
 * and receiving response into ads_rx_buf. Used for register reads and device
 * identification.
 *
 * @param[in] data    Unused parameter (data comes from pr_word buffer)
 * @param[in] size    Number of bytes to transfer
 * @param[in] ads_id  Target device (ADS1298_A or ADS1298_B)
 *
 * @return 0 on success, -1 on GPIO error
 *
 * @note This function disables interrupts during transfer for timing accuracy
 * @note CS is asserted by this function and deasserted by interrupt handler
 */
int ads1298_read_spi(uint8_t *data, uint8_t size, ads_device_id_t ads_id) {
  extern struct k_mutex spi_mutex;
  extern nrfx_spim_t spim_inst;

  unsigned int key = irq_lock(); // Disable all interrupts
  k_mutex_lock(&spi_mutex, K_FOREVER);
  nrfx_spim_xfer_desc_t spim_xfer_desc = NRFX_SPIM_XFER_TRX(pr_word, size, ads_rx_buf, size);

  if (ads_chip_select(ads_id) < 0) {
    k_mutex_unlock(&spi_mutex);
    irq_unlock(key);
    return -1;
  }

  nrfx_err_t status;
  status = nrfx_spim_xfer(&spim_inst, &spim_xfer_desc, 0);
  NRFX_ASSERT(status == NRFX_SUCCESS);

  k_mutex_unlock(&spi_mutex);
  irq_unlock(key); // Restore interrupts

  return 0;
}

/**
 * @brief Read ADC sample data in continuous mode
 *
 * Reads conversion data when in RDATAC (Read Data Continuous) mode.
 * Transmits dummy bytes (zeros) and receives 27 bytes of data:
 * - 3 status bytes (24-bit status word)
 * - 24 data bytes (8 channels × 3 bytes per channel)
 *
 * @param[in] data    Unused parameter (data received into ads_rx_buf)
 * @param[in] size    Number of bytes to transfer (typically 27)
 * @param[in] ads_id  Target device (ADS1298_A or ADS1298_B)
 *
 * @return 0 on success, -1 on GPIO error
 *
 * @note Called from DRDY interrupt context via process_ads_data()
 * @note empty_buffer contains zeros for dummy TX bytes
 */
int ads1298_read_samples(uint8_t *data, uint8_t size, ads_device_id_t ads_id) {
  extern struct k_mutex spi_mutex;
  extern nrfx_spim_t spim_inst;

  unsigned int key = irq_lock(); // Disable all interrupts
  k_mutex_lock(&spi_mutex, K_FOREVER);

  nrfx_spim_xfer_desc_t spim_xfer_desc = NRFX_SPIM_XFER_TRX(empty_buffer, size, ads_rx_buf, size);
  if (ads_chip_select(ads_id) < 0) {
    k_mutex_unlock(&spi_mutex);
    irq_unlock(key);
    return -1;
  }

  nrfx_err_t status;
  status = nrfx_spim_xfer(&spim_inst, &spim_xfer_desc, 0);
  NRFX_ASSERT(status == NRFX_SUCCESS);
  k_mutex_unlock(&spi_mutex);
  irq_unlock(key); // Restore interrupts

  return 0;
}

/**
 * @brief Write command or register data to ADS1298
 *
 * Transmits command bytes from pr_word buffer to configure the device.
 * Used for:
 * - Single-byte commands (RESET, START, STOP, etc.)
 * - Register writes (WREG followed by address and data)
 *
 * @param[in] size    Number of bytes to write from pr_word
 * @param[in] ads_id  Target device (ADS1298_A or ADS1298_B)
 *
 * @return 0 on success, -1 on GPIO error
 *
 * @note pr_word buffer must be populated before calling this function
 * @note Transfer is non-blocking; poll spi_xfer_done for completion
 */
int ads1298_write_spi(uint8_t size, ads_device_id_t ads_id) {
  extern struct k_mutex spi_mutex;
  extern nrfx_spim_t spim_inst;

  nrfx_spim_xfer_desc_t spim_xfer_desc = NRFX_SPIM_XFER_TRX(pr_word, sizeof(pr_word), ads_rx_buf, sizeof(ads_rx_buf));
  nrfx_err_t status;

  if (ads_chip_select(ads_id) < 0) {
    return -1;
  }

  LOG_DBG("Starting SPI write transfer");
  status = nrfx_spim_xfer(&spim_inst, &spim_xfer_desc, 0);
  LOG_DBG("Status: %d", status);
  LOG_DBG("SPI write transfer started");
  NRFX_ASSERT(status == NRFX_SUCCESS);
  LOG_DBG("SPI write transfer asserted CS");

  return 0;
}