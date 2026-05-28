/*
 * ----------------------------------------------------------------------
 *
 * File: wifi_sd_shield_defs.h
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
 * @file wifi_sd_shield_defs.h
 * @brief Wi-Fi and SD Shield Common Definitions and Constants
 *
 * This header contains all shared definitions, constants, and macros used
 * across the Wi-Fi and SD shields. It includes:
 * - GPIO pin definitions
 * - Hardware pin assignments
 * - SPIM instance for NRF <-> ESP communication
 * - Wi-Fi and SD shield specific constants
*/

#ifndef WIFI_SD_SHIELD_DEFS_H
#define WIFI_SD_SHIELD_DEFS_H

#include <zephyr/kernel.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/spi.h>
#include <zephyr/logging/log.h>
#include <hal/nrf_gpio.h>
#include "nrfx_spim.h"
#include <stdbool.h>


/** @brief SPIM instance index to be used for SPIB - communication with WiFi - SD card shield */
#define SPIM_WIFI_SD_SHIELD_INST_IDX 3
/**
 * @brief SPIM driver instance for Wi-Fi / SD card Shield
 *
 * Configured in init_spi_b_wifi_sd_card_shield()
*/
extern nrfx_spim_t spim_b_wifi_sd_shield_inst; 


/** @brief SPI interrupt priority level for Wi-Fi and SD card shield */
#define SPI_WIFI_SD_SHIELD_INT_PRIO 1


/** @brief PIN definitions for SPI-B interface */

#define SCK_SPI_B_PIN NRF_GPIO_PIN_MAP(1, 7)   // P1.07
#define SPI_B_MOSI_PIN NRF_GPIO_PIN_MAP(0, 30)  // P0.30
#define SPI_B_MISO_PIN NRF_GPIO_PIN_MAP(0, 29)  // P0.29
#define ESP32_CS_GPIO_NODE DT_NODELABEL(gpio_esp32_cs) //P0.12


/** @brief Additional GPIO definition */
#define DRDY_GPIO_NODE DT_NODELABEL(gpio_wifi_sd_shield_drdy)       // Data-ready signal (P0.4)
#define VOLT_TRANSLATOR_OE_GPIO_NODE DT_NODELABEL(gpio_volt_translator_oe) // Voltage translator output enable (P1.1)

/** @brief SPI configuration constants */
#define SPI_NRF_ESP_INT_PRIO 5
#define HANDSHAKE_MARKER  0xA5
#define HANDSHAKE_MARKER_RCV 0x5A
#define ESP_SPI_HEADER 0x66             // Header byte for every ESP <--> NRF transaction, to verify correct data parsing
#define ESP_SPI_TAILER 0xBB             // Tailer byte for every ESP <--> NRF transaction, to verify correct data parsing

#define ESP_PCKT_MAX_SIZE 820           // Set for now equal to the max size of WULPUS packets    
#define ESP_SEND_QUEUE_SIZE 4           // Max number of packets that can be queued for sending to ESP. 
                                        // Adjust as needed based on expected traffic and memory constraints.     
/** @brief Boolean to verify successful connection between NRF and ESP */
extern bool handshake_done;


/**
 * @brief ESP packet structure for variable-size packet support
 * 
 * Allows sending packets of different sizes through the same queue.
 */
typedef struct {
  uint16_t size;                              // Actual size of data in this packet
  uint8_t data[ESP_PCKT_MAX_SIZE];              // Packet data buffer
} esp_packet_t;

typedef enum {
  NRF_ESP_IDLE,
  SEND_TO_ESP,
} nrf_to_esp_comm_state_t; 
extern nrf_to_esp_comm_state_t nrf_esp_comm_state;
extern bool serve_esp_requests; // Flag to control whether to process incoming ESP data,to avoid SPI bus contention during critical operations
#endif // WIFI_SD_SHIELD_DEFS_H