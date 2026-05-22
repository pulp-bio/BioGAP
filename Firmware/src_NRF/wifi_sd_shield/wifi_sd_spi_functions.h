/*
 * ----------------------------------------------------------------------
 *
 * File: wifi_sd_shield_spi_functions.h
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
 * @file wifi_sd_shield_spi_functions.h
 * @brief Wi-Fi and SD Shield SPI Functions Interface
 *
 * This header contains the SPI function declarations for interacting with
 * the Wi-Fi and SD shields. 
*/

#ifndef WIFI_SD_SHIELD_SPI_FUNCTIONS_H
#define WIFI_SD_SHIELD_SPI_FUNCTIONS_H

#include "wifi_sd_shield_defs.h"
#include "nrfx_spim.h"


/**  @brief SPI master transmit/receive transaction */
int spi_master_transceive(const uint8_t *tx_buf, uint8_t *rx_buf, size_t len);

/**  @brief BIOGAP to ESP transaction */
int biogap_to_esp_transaction(esp_packet_t *packet);

#endif /* WIFI_SD_SHIELD_SPI_FUNCTIONS_H */