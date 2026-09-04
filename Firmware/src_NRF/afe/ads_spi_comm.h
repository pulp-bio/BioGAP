/*
 * ----------------------------------------------------------------------
 *
 * File: ads_spi_comm.h
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

#ifndef ADS_SPI_COMM_H
#define ADS_SPI_COMM_H

#include <stdint.h>
#include "afe/ads_appl.h"

/*==============================================================================
 * Function Declarations - Low-Level SPI Transfers
 *============================================================================*/

/**
 * @brief Perform SPI read transaction with command
 */
int ads1298_read_spi(uint8_t *data, uint8_t size, ads_device_id_t ads_id);

/**
 * @brief Read ADC sample data in continuous mode
 */
int ads1298_read_samples(uint8_t *data, uint8_t size, ads_device_id_t ads_id);

/**
 * @brief Write command or register data to ADS1298
 */
int ads1298_write_spi(uint8_t size, ads_device_id_t ads_id);

#endif // ADS_SPI_COMM_H