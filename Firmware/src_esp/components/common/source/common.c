/*
 * ----------------------------------------------------------------------
 *
 * File: common.c
 *
 * Last edited: 17.07.2026
 *
 * Copyright (c) 2026 ETH Zurich and University of Bologna
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

#include "common.h"
#include "esp_log.h"
#include "biogap.h"

static const char *COMMON_TAG = "[common.c]: ";

// Track current SPI bus mode for safe mode switching
spi_mode_t current_spi_mode = SPI_MODE_IDLE;
SemaphoreHandle_t spi_bus_mutex = NULL;

/* Initialize ESP as SPI Slave, NRF will be the master */
esp_err_t init_nrf_spi_master_esp_slave_bus(void)
{   
    // Configuration for the SPI bus
    spi_bus_config_t buscfg = {
        .mosi_io_num = NRF_ESP_MOSI,
        .miso_io_num = NRF_ESP_MISO,
        .sclk_io_num = NRF_ESP_SCLK,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
        .data4_io_num = -1,
        .data5_io_num = -1,
        .data6_io_num = -1,
        .data7_io_num = -1,
        .max_transfer_sz = SPI_FROM_BIOGAP_MAX_SIZE,
        .flags = 0,
        .intr_flags = 0,
    };

    spi_slave_interface_config_t slvcfg = {
        .mode = 1,                   // SPI Mode 1 required for DMA (CPOL=0, CPHA=1)
        .spics_io_num = NRF_ESP_CS,
        .queue_size = QUEUE_COUNT,
        .flags = 0
    };


    esp_err_t ret = spi_slave_initialize(SPI_HOST_DEVICE, &buscfg, &slvcfg, SPI_DMA_CH_AUTO);
    if (ret != ESP_OK) {
        ESP_LOGE(COMMON_TAG, ">>> FAILED to initialize NRF SPI slave: %s", esp_err_to_name(ret));
        return ret;
    }

    return ESP_OK;
}


/** @brief Configure the DRDY, DRDY_DIR_CTRL, and DIR_CTRL GPIOs for the NRF<->ESP SPI link. */
esp_err_t config_spi_nrf_master_esp_slave_pins(void){
    esp_err_t ret;
    // Configure the Data Ready pin as output. 
    gpio_config_t io_conf = {
        .pin_bit_mask = (1ULL << NRF_ESP_DATA_READY),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,  
        .pull_down_en = GPIO_PULLDOWN_ENABLE, 
        .intr_type = GPIO_INTR_DISABLE, 
    };

    ESP_ERROR_CHECK(gpio_reset_pin(NRF_ESP_DATA_READY));
    ESP_ERROR_CHECK(gpio_config(&io_conf));

    // Default state: LOW 
    ret = gpio_set_level(NRF_ESP_DATA_READY, 0);
    if (ret!=ESP_OK){
        ESP_LOGE(COMMON_TAG, "Failed to set NRF data-ready pin LOW: %s", esp_err_to_name(ret));
        return ret;
    }

    // Configure the DRDY_DIR_CTRL pin (IC5, dedicated DRDY level-translator channel).
    gpio_config_t io_conf_dir_ctrl = {
        .pin_bit_mask = (1ULL << NRF_ESP_DRDY_DIR_CTRL),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_ENABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };

    ESP_ERROR_CHECK(gpio_reset_pin(NRF_ESP_DRDY_DIR_CTRL));
    ESP_ERROR_CHECK(gpio_config(&io_conf_dir_ctrl));

    // Fixed HIGH: DRDY always flows ESP -> NRF (IC5 channel A->B).
    ret = gpio_set_level(NRF_ESP_DRDY_DIR_CTRL, 1);
    if (ret!=ESP_OK){
        ESP_LOGE(COMMON_TAG, "Failed to set NRF data-ready direction pin HIGH: %s", esp_err_to_name(ret));
        return ret;
    }

    // Configure the DIR_CTRL pin (IC2, MOSI/MISO/CS/CLK level-translator channels).
    // The polarity depends on which side is SPI master. If the NRF is SPI master:
    // LOW routes MOSI/CS/CLK as NRF(B) -> ESP(A), MISO as ESP(A) -> NRF(B) 
    gpio_config_t io_conf_spi_dir_ctrl = {
        .pin_bit_mask = (1ULL << NRF_ESP_DIR_CTRL),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_ENABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };

    ESP_ERROR_CHECK(gpio_reset_pin(NRF_ESP_DIR_CTRL));
    ESP_ERROR_CHECK(gpio_config(&io_conf_spi_dir_ctrl));

    ret = gpio_set_level(NRF_ESP_DIR_CTRL, 0);
    if (ret!=ESP_OK){
        ESP_LOGE(COMMON_TAG, "Failed to set NRF/ESP SPI direction pin LOW: %s", esp_err_to_name(ret));
        return ret;
    }

    return ESP_OK;
}

