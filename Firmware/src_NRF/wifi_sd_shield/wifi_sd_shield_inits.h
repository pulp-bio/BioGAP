/*
 * ----------------------------------------------------------------------
 *
 * File: wifi_sd_shield_inits.h
 *
 * Last edited: 15.05.2026
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
 * @file wifi_sd_shield_inits.h
 * @brief Wi-Fi and SD Shield Initialization Functions
 *
 * This header contains the initialization functions for the Wi-Fi and SD card shields.
 */

#ifndef WIFI_SD_SHIELD_INITS_H
#define WIFI_SD_SHIELD_INITS_H

#include <zephyr/kernel.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/logging/log.h>
#include <hal/nrf_gpio.h>
#include "zephyr/irq.h"
#include "wifi_sd_shield_defs.h"
#include "core/command_dispatcher.h"

extern struct k_sem spi_nrf_esp_transfer_done;
/**
 * @brief Data ready interrupt flag
 *
 * Set by ESP_DRDY GPIO interrupt when ESP wants to send new data. Cleared
 * by process_esp_data() after reading.
 */
extern bool esp_data_ready;

/** @brief ESP32-C6 chip select GPIO spec (SPI_A, software-toggled) */
extern const struct gpio_dt_spec esp_cs_gpio;

/**
 * @brief Direct NRF -> SD card chip select GPIO spec (SPI_A, software-toggled)
 *
 * Configured (deasserted) at init time but otherwise unused for now -- the
 * NRF writing to the SD card directly, bypassing the ESP32, is not yet
 * implemented.
 */
extern const struct gpio_dt_spec sd_cs_gpio;

/** @brief nRF QSPI CS passthrough GPIO spec (see QSPI_CS_GPIO_NODE) */
extern const struct gpio_dt_spec qspi_cs_gpio;

/** @brief QSPI CS mux select GPIO spec (see QSPI_CS_SEL_GPIO_NODE) */
extern const struct gpio_dt_spec qspi_cs_sel_gpio;

/**
 * @brief Initialize CS (ESP32, SD card) and DRDY GPIOs for the WiFi/SD shield
 *
 * Does NOT bring up SPI_A itself: SPI_A is shared with the ExG shield, so
 * whichever shield initializes first calls init_spi_a_bus() (spi/spi_a.h);
 * subsequent calls are no-ops. Call this after init_spi_a_bus() (directly,
 * or transitively via the ExG shield's init_spi()) has run at least once.
 */
int wifi_sd_shield_cs_init(void);

/** @brief Helper function to set DRDY pin (replaces direct gpio_pin_set calls) */
int drdy_pin_set(int value);
/** @brief Initialize and Enable Data Ready GPIO for ESP -> NRF communication */
int esp_drydy_gpio_init(void);


/** @brief SPI master transaction functions (nrfx-based) */
int spi_nrf_master_esp_slave_transceive(const uint8_t *tx_buf, uint8_t *rx_buf, size_t len);
int spi_nrf_master_esp_slave_write(const uint8_t *tx_buf, size_t len);
int spi_nrf_master_esp_slave_read(uint8_t *rx_buf, size_t len);



#endif // WIFI_SD_SHIELD_INITS_H