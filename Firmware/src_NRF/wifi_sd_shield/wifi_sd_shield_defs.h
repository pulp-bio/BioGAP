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


/**
 * @brief WiFi/SD shield now sits on the shared SPI_A bus (see spi/spi_a.h)
 * rather than a dedicated SPIM instance -- SPI_A is physically shared with
 * the ADS1298 (ExG shield), per the new mainboard's default SPI-A jumper
 * population on the WiFi/SD shield.
 */

/** @brief Chip select GPIO node for the ESP32-C6 (P0.14, see
 *  custom_shields/SENSEI_WiFi_SD_Shield_V1/SENSEI_WiFi_SD_Shield_V1.overlay) */
#define ESP32_CS_GPIO_NODE DT_NODELABEL(gpio_nrf_esp_cs)

/** @brief Chip select GPIO node for direct NRF -> SD card access (P0.16).
 *  Reserved for when the NRF writes to the SD card directly instead of
 *  through the ESP32; no such data path exists yet. */
#define SD_CS_GPIO_NODE DT_NODELABEL(gpio_nrf_sd_cs)

/** @brief Data-ready signal, ESP -> NRF (P0.15) */
#define DRDY_GPIO_NODE DT_NODELABEL(gpio_nrf_esp_shield_drdy)

/**
 * @brief nRF on-chip QSPI chip-select passthrough (P0.18)
 *
 * The nRF5340's dedicated QSPI_CS pin, unused as QSPI here (WiFi/SD shield
 * uses SPI_A instead), so it's free to bit-bang as GPIO. Feeds an analog
 * mux that routes it to either the on-board QSPI PSRAM's CS or the
 * board-to-board connector, selected by QSPI_CS_SEL_GPIO_NODE. Driven to 1
 * (deactivate the on-board PSRAM) when the WiFi shield is enabled.
 */
#define QSPI_CS_GPIO_NODE DT_NODELABEL(gpio_nrf_qspi_cs)

/**
 * @brief QSPI CS mux select (P0.19)
 *
 * Driven to 0 (route QSPI_CS_GPIO_NODE to the on-board PSRAM's CS) when
 * the WiFi shield is enabled.
 */
#define QSPI_CS_SEL_GPIO_NODE DT_NODELABEL(gpio_nrf_qspi_cs_sel)

/** @brief SPI configuration constants */
#define SPI_NRF_ESP_INT_PRIO 5
#define HANDSHAKE_MARKER  0xA5
#define HANDSHAKE_MARKER_RCV 0x5A
#define ESP_SPI_HEADER 0x66             // Header byte for every ESP <--> NRF transaction, to verify correct data parsing
#define ESP_SPI_TAILER 0xBB             // Tailer byte for every ESP <--> NRF transaction, to verify correct data parsing

/** @brief Bit 7 of a packet's header byte: set by the NRF on a dedicated
 *  ack transaction (biogap_to_esp_transaction()) to explicitly confirm it
 *  received and processed the ESP's piggybacked STOP marker. Checked by the
 *  ESP against data it actually reads FROM the NRF (its rx_buffer), not its
 *  own previously-stomped tx buffer -- see enter_stop_quiesce_state()
 *  (espc6/components/biogap/source/biogap_read.c) for why that distinction
 *  matters. */
#define NRF_STOP_ACK_MASK 0x80

#define ESP_PCKT_MAX_SIZE 250           // Set for now equal to the max size of WULPUS packets    
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

extern uint16_t esp_spi_packet_size; 
typedef enum {
  NRF_ESP_IDLE,
  SEND_TO_ESP,
} nrf_to_esp_comm_state_t; 
extern nrf_to_esp_comm_state_t nrf_esp_comm_state;
extern bool serve_esp_requests; // Flag to control whether to process incoming ESP data,to avoid SPI bus contention during critical operations

/* TEMP DIAGNOSTIC: when true, both WiFi/SD SPI_A threads skip all bus
 * activity, so ADS reads can be tested in isolation. Remove once the
 * ADS/WiFi_SD SPI_A hang is root-caused. */
extern volatile bool wifi_sd_paused;
#endif // WIFI_SD_SHIELD_DEFS_H