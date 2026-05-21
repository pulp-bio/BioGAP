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

LOG_MODULE_REGISTER(wifi_sd_shield_inits, LOG_LEVEL_INF);

bool esp_data_ready = false;
nrfx_spim_t spim_b_wifi_sd_shield_inst = NRFX_SPIM_INSTANCE(SPIM_WIFI_SD_SHIELD_INST_IDX);


/* GPIO device specs - defined in device tree overlay */
static const struct gpio_dt_spec drdy_gpio = GPIO_DT_SPEC_GET(DRDY_GPIO_NODE, gpios);
static const struct gpio_dt_spec esp_cs_gpio = GPIO_DT_SPEC_GET(ESP32_CS_GPIO_NODE, gpios);
static const struct gpio_dt_spec volt_translator_oe_gpio = GPIO_DT_SPEC_GET(VOLT_TRANSLATOR_OE_GPIO_NODE, gpios);

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
  LOG_DBG("ESP DRDY interrupt: new data ready from ESP");
}


/**
 * @brief SPI interrupt handler for nrfx SPIM transactions between BIOGAP and Wi-Fi shield (ESP32-C6)
*/
void spim_biogap_wifi_shield_handler(nrfx_spim_evt_t const *p_event, void *p_context)
{
    (void)p_event;
    (void)p_context;
    /* Handle SPI transfer completion if needed */
    LOG_DBG("SPI transfer complete");
    k_sem_give(&spi_nrf_esp_transfer_done);
}


/**
 * @brief SPI B (NRF Master) for communication with WiFi - SD card shield initialization
 * 
 * Intended to be used when US (WULPUS shield) is not in used, which uses SPI B.  
 *
 * Performs complete hardware initialization:
 * 1. Connects SPI interrupt handler
 * 2. Configures SPIM peripheral (4 MHz, Mode 1, MSB first)
 *
 * SPI Mode 1 timing:
 * - CPOL = 0 (clock idle low)
 * - CPHA = 1 (data sampled on rising edge, shifted on falling edge) 
 *
 * @note To enable DMA on ESP32-C6, SPI mode must be set to Mode 1 or 3.
 * @note This function must be called before any other WiFi-SD card functions.
*/ 
int init_spi_b_wifi_sd_card_shield() {
    nrfx_err_t status;
    (void)status;
    #if defined(__ZEPHYR__)
        IRQ_CONNECT(NRFX_IRQ_NUMBER_GET(NRF_SPIM_INST_GET(SPIM_WIFI_SD_SHIELD_INST_IDX)), SPI_WIFI_SD_SHIELD_INT_PRIO,
               NRFX_SPIM_INST_HANDLER_GET(SPIM_WIFI_SD_SHIELD_INST_IDX), 0, 0);
    #endif


    // 
    nrfx_spim_config_t spim_config = NRFX_SPIM_DEFAULT_CONFIG(SCK_SPI_B_PIN, SPI_B_MOSI_PIN, SPI_B_MISO_PIN, esp_cs_gpio.pin);
    spim_config.frequency = NRFX_MHZ_TO_HZ(4);
    spim_config.bit_order = NRF_SPIM_BIT_ORDER_MSB_FIRST;
    spim_config.irq_priority = SPI_WIFI_SD_SHIELD_INT_PRIO;
    // Mode 1, for compatibility with ESP32-C6 and DMA support. 
    spim_config.mode = NRF_SPIM_MODE_1;

    void *p_context = "SPI B used for WiFi and SD card shield; NRF is SPI master, ESP32-C6 is SPI slave";
    status = nrfx_spim_init(&spim_b_wifi_sd_shield_inst, &spim_config, spim_biogap_wifi_shield_handler, p_context);
    NRFX_ASSERT(status == NRFX_SUCCESS);

    /* Verify SPI CS GPIO is ready (configuration is in device tree) */
    if (!device_is_ready(esp_cs_gpio.port)) {
        LOG_ERR("ESP32-CS GPIO port not ready");
        return -1;
    }
    LOG_INF("SPI B initialized for Wi-Fi and SD card shield communication");
    return 0;
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
    /** @brief Initialize GPIOs for Wi-Fi and SD card shield */
int gpios_wifi_sd_card_shield_init() {

    /* Initialize all GPIO devices */
    int ret;

    
    ret = esp_drydy_gpio_init();
    if (ret != 0) {
        LOG_ERR("Failed to initialize DRDY GPIO (%d)", ret);
        return -1;
    }
    else{
        LOG_INF("DRDY GPIO (P0.4) initialized");
    }
    if (!device_is_ready(esp_cs_gpio.port)) {
        LOG_ERR("ESP32-CS GPIO controller not ready!");
        return -1;
    }

    
    ret = gpio_pin_configure_dt(&esp_cs_gpio, GPIO_OUTPUT_INACTIVE);
    if (ret != 0) {
        LOG_ERR("Failed to configure ESP32-CS GPIO (%d)", ret);
        return -1;
    }
    LOG_INF("ESP32-CS (P0.12) initialized");

    if (!device_is_ready(volt_translator_oe_gpio.port)) {
        LOG_ERR("Voltage Translator OE controller not ready!");
        LOG_ERR("=== INITIALIZATION FAILURE - NRF53 HALTED ===");
        return -1;
    }
    ret = gpio_pin_configure_dt(&volt_translator_oe_gpio, GPIO_OUTPUT_ACTIVE);
    if (ret != 0) {
        LOG_ERR("Failed to configure volt translator OE GPIO (%d)", ret);
        return -1;
    }
    LOG_INF("Volt translator OE (P1.1) initialized");

    return 0;
}
