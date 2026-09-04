/*
 * ----------------------------------------------------------------------
 *
 * File: ads_spi_hw.h
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

#ifndef ADS_SPI_HW_H
#define ADS_SPI_HW_H

#include <zephyr/drivers/gpio.h>
#include <nrfx_spim.h>
#include "afe/ads_defs.h"
#include "spi/spi_a.h"

/*==============================================================================
 * External Hardware Resources
 *============================================================================*/

/** @brief ADS1298_A chip select GPIO spec */
extern const struct gpio_dt_spec gpio_dt_ads1298_a_cs;

/** @brief ADS1298_B chip select GPIO spec */
extern const struct gpio_dt_spec gpio_dt_ads1298_b_cs;

/** @brief SPI receive buffer for ADS1298 data */
extern uint8_t ads_rx_buf[40];

/** @brief SPI command/data buffer */
extern uint8_t pr_word[10];

/** @brief ADS1298 initialization status flag */
extern bool ads_initialized;

/**
 * @brief SPI_A completion handler for ADS1298 transfers
 *
 * Called by the shared spi/spi_a.c interrupt dispatcher when the in-flight
 * transfer's owner is SPI_A_OWNER_ADS. Deasserts both ADS1298 CS lines and
 * runs the read/write completion logic (see ads_spi_data.c).
 */
void ads_spim_transfer_complete(void);

#endif // ADS_SPI_HW_H