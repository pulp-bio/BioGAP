/*
 * ----------------------------------------------------------------------
 *
 * File: wifi_sd_shield_appl.h
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
 * @file wifi_sd_shield_appl.h
 * @brief Wi-Fi and SD Shield Application Interface
 *
 * This header contains the application-level interface for interacting with
 * the Wi-Fi and SD shields. 
*/

#ifndef WIFI_SD_SHIELD_APPL_H
#define WIFI_SD_SHIELD_APPL_H

#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>

/** @brief Perform initial handshake with ESP32 to verify that the connection is established */
int initial_handshake_nrf_esp();

/** @brief Process ESP data when DRDY interrupt occurs*/
void process_esp_data(void); 

/** @brief SPI receiver thread for NRF and ESP communication*/
/* This thread should be considered the counterpart of process_received_data_thread() used for BLE data reception.
*/
void spi_nrf_esp_receiver_thread(void *arg1, void *arg2, void *arg3); 

/** @brief SPI sender thread for NRF and ESP communication*/
void spi_nrf_esp_sender_thread(void *arg1, void *arg2, void *arg3);

/** @brief Add data to ESP send buffer */
void add_data_to_esp_send_buffer(uint8_t *data, uint16_t size);

#endif /* WIFI_SD_SHIELD_APPL_H */