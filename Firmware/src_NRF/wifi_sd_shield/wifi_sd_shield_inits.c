/*
 * ----------------------------------------------------------------------
 *
 * File: wifi_sd_shield_inits.c
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
 * @file wifi_sd_shield_inits.c
 * @brief Wi-Fi and SD Shield Initialization Functions
 *
 * This file contains the initialization functions for the Wi-Fi and SD card shields.
 */

#include "wifi_sd_shield_inits.h"
#include "spi/spi_a.h"

LOG_MODULE_REGISTER(wifi_sd_shield_inits, LOG_LEVEL_INF);

bool esp_data_ready = false;
bool first_esp_data_ready = true;
bool serve_esp_requests = false; // Flag to control whether to process incoming ESP data, to avoid SPI bus contention during critical operations
nrf_to_esp_comm_state_t nrf_esp_comm_state = NRF_ESP_IDLE; // State variable to manage NRF-ESP communication flow

/* GPIO device specs - defined in device tree overlay */
static const struct gpio_dt_spec drdy_gpio = GPIO_DT_SPEC_GET(DRDY_GPIO_NODE, gpios);
const struct gpio_dt_spec esp_cs_gpio = GPIO_DT_SPEC_GET(ESP32_CS_GPIO_NODE, gpios);
const struct gpio_dt_spec sd_cs_gpio = GPIO_DT_SPEC_GET(SD_CS_GPIO_NODE, gpios);
const struct gpio_dt_spec qspi_cs_gpio = GPIO_DT_SPEC_GET(QSPI_CS_GPIO_NODE, gpios);
const struct gpio_dt_spec qspi_cs_sel_gpio = GPIO_DT_SPEC_GET(QSPI_CS_SEL_GPIO_NODE, gpios);

/*==============================================================================
 * Private Functions - Interrupt Callbacks
 *============================================================================*/

/** @brief GPIO callback data structure for DRDY interrupt */
static struct gpio_callback esp_drdy_cb_data;

/** @brief DRDY GPIO callback function for ESP -> NRF communication.
*/
static void cb_esp_drdy(const struct device *dev, struct gpio_callback *cb, uint32_t pins) {
  /* Signal that ESP has new data to send */
  esp_data_ready = true;
  // Signal also that ESP must be served 
  serve_esp_requests = true;
  //LOG_INF("ESP DRDY interrupt: new data ready from ESP");
}


/** @brief Initialize adn Enable Data Ready GPIO for ESP -> NRF communication */
int esp_drydy_gpio_init(){
    int ret; 
    /* Data Ready GPIO for ESP <--> NRF comminication */
    if (!device_is_ready(drdy_gpio.port)) {
        LOG_ERR("DRDY GPIO controller not ready!");
        return -1;
    }
    ret = gpio_pin_configure_dt(&drdy_gpio, GPIO_INPUT);
    if (ret != 0) {
        LOG_ERR("Failed to configure DRDY GPIO (%d)", ret);
        return -1;
    }

    // Initialize the static struct gpio_callback variable
    gpio_init_callback(&esp_drdy_cb_data, cb_esp_drdy, BIT(drdy_gpio.pin));
    // Add the callback function by calling gpio_add_callback()
    ret = gpio_add_callback(drdy_gpio.port, &esp_drdy_cb_data);
    if (ret < 0) {
        LOG_ERR("ESP DRDY GPIO interrupt enable error");
        return -1;
    }

    // Enable the DRDY interrupt only after the callback is registered.
    ret = gpio_pin_interrupt_configure_dt(&drdy_gpio, GPIO_INT_EDGE_TO_ACTIVE);
    if (ret < 0) {
        LOG_ERR("ESP DRDY GPIO interrupt enable error");
        return -1;
    }

    LOG_INF("ESP DRDY GPIO initialized and interrupt enabled");


    
    return 0; 

}
/**
 * @brief Initialize the WiFi/SD shield: shared SPI_A bus, CS/DRDY/QSPI-mux GPIOs
 *
 * SPI_A itself is shared with the ExG shield -- init_spi_a_bus() is
 * idempotent, so it is safe to call here regardless of whether the ExG
 * shield already brought it up (or will do so later).
 *
 * Also parks the on-board QSPI PSRAM's mux out of the way: with the WiFi
 * shield enabled, the nRF's on-chip QSPI peripheral is not used, so
 * QSPI_CS_SEL_GPIO_NODE is set to 0 (routes QSPI_CS_GPIO_NODE to the
 * on-board PSRAM's CS) and QSPI_CS_GPIO_NODE is set to 1 (deactivates
 * that PSRAM) so it never sits selected/floating.
 *
 * @note The bus-direction/level-translator control lines found on the
 * WiFi shield schematic (NRF_ESP_DIR_CTRL, NRF_ESP_DRDY_DIR_CTRL) are not
 * yet modeled in the devicetree overlay or initialized here -- they need
 * dedicated GPIO nodes (with real pin numbers from the shield schematic)
 * added to SENSEI_WiFi_SD_Shield_V1.overlay before this shield can be
 * brought up on real hardware.
 */
int wifi_sd_shield_cs_init(void) {
    int ret;

    if (init_spi_a_bus() != 0) {
        LOG_ERR("Failed to initialize shared SPI_A bus");
        return -1;
    }

    ret = esp_drydy_gpio_init();
    if (ret != 0) {
        LOG_ERR("Failed to initialize DRDY GPIO (%d)", ret);
        return -1;
    }
    LOG_INF("DRDY GPIO initialized");

    if (!device_is_ready(esp_cs_gpio.port)) {
        LOG_ERR("ESP32-CS GPIO controller not ready!");
        return -1;
    }
    ret = gpio_pin_configure_dt(&esp_cs_gpio, GPIO_OUTPUT_INACTIVE);
    if (ret != 0) {
        LOG_ERR("Failed to configure ESP32-CS GPIO (%d)", ret);
        return -1;
    }
    LOG_INF("ESP32-CS initialized");

    if (!device_is_ready(sd_cs_gpio.port)) {
        LOG_ERR("SD-CS GPIO controller not ready!");
        return -1;
    }
    ret = gpio_pin_configure_dt(&sd_cs_gpio, GPIO_OUTPUT_INACTIVE);
    if (ret != 0) {
        LOG_ERR("Failed to configure SD-CS GPIO (%d)", ret);
        return -1;
    }
    LOG_INF("SD-CS initialized");

    if (!device_is_ready(qspi_cs_sel_gpio.port) || !device_is_ready(qspi_cs_gpio.port)) {
        LOG_ERR("QSPI mux GPIO controller not ready!");
        return -1;
    }
    ret = gpio_pin_configure_dt(&qspi_cs_sel_gpio, GPIO_OUTPUT);
    ret |= gpio_pin_set_dt(&qspi_cs_sel_gpio, 0); // route QSPI_CS to the on-board PSRAM
    ret |= gpio_pin_configure_dt(&qspi_cs_gpio, GPIO_OUTPUT);
    ret |= gpio_pin_set_dt(&qspi_cs_gpio, 1); // deactivate the on-board PSRAM
    if (ret != 0) {
        LOG_ERR("Failed to configure QSPI mux GPIOs (%d)", ret);
        return -1;
    }
    LOG_INF("QSPI mux parked (on-board PSRAM deactivated)");

    return 0;
}
