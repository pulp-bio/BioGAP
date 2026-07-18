/*
 * ----------------------------------------------------------------------
 *
 * File: ads_spi_hw.c
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
 * @file ads_spi_hw.c
 * @brief ADS1298 Hardware Abstraction Layer
 *
 * This module handles all hardware-related initialization and configuration
 * for ADS1298 communication, including SPI peripheral setup, GPIO pin
 * configuration, and interrupt handling.
 */

/* Zephyr RTOS headers */
#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>

#include "afe/ads_defs.h"
#include "afe/ads_spi.h"
#include "afe/ads_spi_hw.h"
#include "spi/spi_a.h"

LOG_MODULE_REGISTER(ads_spi_hw, LOG_LEVEL_INF);

/*==============================================================================
 * Device Tree Node References
 *============================================================================*/

/** @brief Data Ready pin for ADS1298_A */
#define ADS_A_DR_NODE DT_NODELABEL(gpio_ads1298_a_dr)

/** @brief Chip Select pin for ADS1298_A */
#define CS_A_NODE DT_NODELABEL(gpio_ads1298_a_spi_cs)

/** @brief Chip Select pin for ADS1298_B */
#define CS_B_NODE DT_NODELABEL(gpio_ads1298_b_spi_cs)

/** @brief Shared START pin for synchronized acquisition */
#define ADS_START_NODE DT_NODELABEL(gpio_ads1298_start_pin)

/*==============================================================================
 * GPIO Device Specifications
 *============================================================================*/

/** @brief ADS1298_A chip select GPIO spec */
const struct gpio_dt_spec gpio_dt_ads1298_a_cs = GPIO_DT_SPEC_GET(CS_A_NODE, gpios);

/** @brief ADS1298_B chip select GPIO spec */
const struct gpio_dt_spec gpio_dt_ads1298_b_cs = GPIO_DT_SPEC_GET(CS_B_NODE, gpios);

/** @brief ADS1298 START pin GPIO spec (shared by both devices) */
static const struct gpio_dt_spec gpio_dt_ads1298_start_pin = GPIO_DT_SPEC_GET(ADS_START_NODE, gpios);

/** @brief ADS1298_A data ready pin GPIO spec */
static const struct gpio_dt_spec gpio_dt_ads1298_a_dr = GPIO_DT_SPEC_GET(ADS_A_DR_NODE, gpios);

/*==============================================================================
 * SPI Configuration
 *============================================================================*/

/*==============================================================================
 * Private Functions - Interrupt Callbacks
 *============================================================================*/

/** @brief GPIO callback data structure for DRDY interrupt */
static struct gpio_callback ads1298_a_dr_cb_data;

/**
 * @brief GPIO callback for ADS1298 data ready interrupt
 *
 * Invoked by GPIO subsystem when DRDY pin goes active. Sets flag for
 * main loop processing and increments debug counter.
 *
 * @param[in] dev   GPIO device (unused)
 * @param[in] cb    Callback structure (unused)
 * @param[in] pins  Pin mask that triggered interrupt (unused)
 *
 * @note Runs in interrupt context - keep processing minimal
 * @note Both ADS devices share this DRDY signal (synchronized)
 */
static void cb_ads_a_dr(const struct device *dev, struct gpio_callback *cb, uint32_t pins);

/*==============================================================================
 * Public Functions - Hardware Initialization
 *============================================================================*/

/**
 * @brief Initialize SPI peripheral and GPIO pins for ADS1298 communication
 *
 * Performs complete hardware initialization:
 * 1. Brings up the shared SPI_A bus (see spi/spi_a.h) if not already done
 * 2. Initializes chip select pins for both ADS devices
 * 3. Initializes START pin for synchronized acquisition
 *
 * SPI Mode 1 timing:
 * - CPOL = 0 (clock idle low)
 * - CPHA = 1 (data sampled on rising edge, shifted on falling edge)
 *
 * @note This function must be called before any other ADS functions.
 *       Errors are logged but not returned to allow graceful degradation.
 */
void init_spi() {
  /* Bring up the shared SPI_A (SPIM4) peripheral. Safe to call even if
   * another shield on SPI_A (e.g. WiFi/SD) already brought it up first --
   * whichever consumer initializes first wins; nrfx_spim_init() is not
   * called a second time here. */
  if (init_spi_a_bus() != 0) {
    return;
  }

  // Initialize SPI CS pin for ADS A
  if (!device_is_ready(gpio_dt_ads1298_a_cs.port)) {
    LOG_ERR("ADS1298 power GPIO port not ready");
    return;
  }
  if (gpio_pin_configure_dt(&gpio_dt_ads1298_a_cs, GPIO_OUTPUT_INACTIVE) < 0) {
    LOG_ERR("ADS pwr GPIO init error");
    return;
  }

  // Initialize SPI CS pin for ADS B
  if (!device_is_ready(gpio_dt_ads1298_b_cs.port)) {
    LOG_ERR("ADS1298 power GPIO port not ready");
    return;
  }
  if (gpio_pin_configure_dt(&gpio_dt_ads1298_b_cs, GPIO_OUTPUT_INACTIVE) < 0) {
    LOG_ERR("ADS pwr GPIO init error");
    return;
  }

  // Initialize SPI START pin for synchronized start of ADS A and B
  if (!device_is_ready(gpio_dt_ads1298_start_pin.port)) {
    LOG_ERR("ADS1298 power GPIO port not ready");
    return;
  }
  if (gpio_pin_configure_dt(&gpio_dt_ads1298_start_pin, GPIO_OUTPUT_INACTIVE) < 0) {
    LOG_ERR("ADS pwr GPIO init error");
    return;
  }
}

/*==============================================================================
 * Public Functions - Data Ready Signal
 *============================================================================*/

/**
 * @brief Read current state of DRDY pin
 *
 * @return 1 if data ready (pin active low on ADS1298), 0 if not ready
 */
int ads_dr_read() { return gpio_pin_get_dt(&gpio_dt_ads1298_a_dr); }

/**
 * @brief Initialize data ready (DRDY) GPIO interrupt
 *
 * Configures the DRDY pin for interrupt-driven data acquisition.
 * The DRDY pin is shared between both ADS1298 devices since they
 * are synchronized via the START pin.
 *
 * DRDY characteristics:
 * - Active low signal
 * - Pulses high when new data is ready
 * - Interrupt on low-to-high edge (EDGE_TO_ACTIVE)
 * - At 1 kSPS, occurs every 1 ms
 *
 * @return 0 on success, -1 on error
 */
int ads_dr_init() {
  // Initialize the data ready pin
  if (!device_is_ready(gpio_dt_ads1298_a_dr.port)) {
    LOG_ERR("ADS1298 DRDY GPIO port not ready");
    return -1;
  }
  if (gpio_pin_configure_dt(&gpio_dt_ads1298_a_dr, GPIO_INPUT) < 0) {
    LOG_ERR("ADS1298 DRDY GPIO init error");
    return -1;
  }
  // Enable interrrupt
  int ret;
  ret = gpio_pin_interrupt_configure_dt(&gpio_dt_ads1298_a_dr, GPIO_INT_EDGE_TO_ACTIVE);
  if (ret < 0) {
    LOG_ERR("ADS1298 DRDY GPIO interrupt enable error");
    return -1;
  }

  // Initialize the static struct gpio_callback variable
  gpio_init_callback(&ads1298_a_dr_cb_data, cb_ads_a_dr, BIT(gpio_dt_ads1298_a_dr.pin));
  // Add the callback function by calling gpio_add_callback()
  ret = gpio_add_callback(gpio_dt_ads1298_a_dr.port, &ads1298_a_dr_cb_data);
  if (ret < 0) {
    LOG_ERR("ADS1298 DRDY GPIO interrupt enable error");
    return -1;
  }

  return 0;
}

/*==============================================================================
 * Implementation Details - SPI Handler and Callbacks
 *============================================================================*/

#include "afe/ads_spi_data.h" // Include for data processing functions

/*==============================================================================
 * External SPI Buffer
 *============================================================================*/

/** @brief SPI receive buffer for ADS1298 data */
uint8_t ads_rx_buf[40];

/*==============================================================================
 * Global Variables
 *============================================================================*/

/**
 * @brief SPI command/data buffer
 *
 * Used for constructing SPI transactions. First byte typically contains
 * the command, followed by register addresses and data.
 */
uint8_t pr_word[10] = {_RESET, 0, 0, 0, 0, 0, 0, 0, 0, 0};

/**
 * @brief ADS1298 initialization status flag
 *
 * Set to true after both ADS1298 devices have been successfully initialized
 * and verified. Used to prevent data acquisition before initialization.
 */
bool ads_initialized = false;

void ads_spim_transfer_complete(void) {
  LOG_DBG("ads_spim_transfer_complete called");

  if (gpio_pin_set_dt(&gpio_dt_ads1298_a_cs, 0) < 0) { // Set CS pin to disable
    LOG_ERR("ADS1298 power GPIO set error");
    return;
  }
  if (gpio_pin_set_dt(&gpio_dt_ads1298_b_cs, 0) < 0) { // Set CS pin to disable
    LOG_ERR("ADS1298 power GPIO set error");
    return;
  }

  ads_spim_handler_done();
}

static void cb_ads_a_dr(const struct device *dev, struct gpio_callback *cb, uint32_t pins) {
  ads_drdy_callback();
}