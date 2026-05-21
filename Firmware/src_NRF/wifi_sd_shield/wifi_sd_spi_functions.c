/*
 * ----------------------------------------------------------------------
 *
 * File: wifi_sd_shield_spi_functions.c
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


#include "wifi_sd_spi_functions.h"
#include "wifi_sd_shield_inits.h"
LOG_MODULE_REGISTER(wifi_sd_spi_functions, LOG_LEVEL_INF);

/* Serialize async SPIM transfers so callers can treat the API as blocking. */
K_SEM_DEFINE(spi_nrf_esp_transfer_done, 0, 1);

/**
 * @brief SPI master transmit/receive transaction
 * 
 * Performs a full-duplex SPI master transaction with the ESP32 slave.
 * Uses nrfx SPIM driver configured for master mode.
 * 
 * @param tx_buf Transmit buffer (NULL for dummy TX)
 * @param rx_buf Receive buffer (NULL to discard RX)
 * @param len Number of bytes to transfer
 * @return 0 on success, negative error code on failure
 */
int spi_master_transceive(const uint8_t *tx_buf, uint8_t *rx_buf, size_t len)
{
    if (len == 0) {
        return -EINVAL;
    }

    /* Drain any stale completion before starting a new async transfer. */
    while (k_sem_take(&spi_nrf_esp_transfer_done, K_NO_WAIT) == 0) {
    }

    /* Configure SPI transaction buffers */
    nrfx_spim_xfer_desc_t xfer = {
        .p_tx_buffer = (uint8_t *)tx_buf,
        .tx_length = tx_buf ? len : 0,
        .p_rx_buffer = rx_buf,
        .rx_length = rx_buf ? len : 0,
    };

    /* Execute transfer (blocking) */
    nrfx_err_t status = nrfx_spim_xfer(&spim_b_wifi_sd_shield_inst, &xfer, 0);
    if(!handshake_done){
        LOG_INF("SPI master transceive initiated, waiting for handshake to complete...");
        LOG_INF("SPI master transceive initiated, sent: 0x%02X 0x%02X 0x%02X 0x%02X", 
            tx_buf[0], tx_buf[1], tx_buf[2], tx_buf[3]);
    }
    if (status != NRFX_SUCCESS) {
        LOG_INF("SPI master transfer failed: %d", status);
        return -EIO;
    }
    else{
        LOG_DBG("SPI master transfer initiated, waiting for completion...");
    }

    // Note: 1msec works for ExG packets, but not for US
    // 811 bytes = 6488 bits, at 4 Mbps takes approx 1.6 msec
    if(!handshake_done){
        status = k_sem_take(&spi_nrf_esp_transfer_done, K_MSEC(5000));
    }
    else{
        // smaller timeout for regular packets after handshake is done
        status = k_sem_take(&spi_nrf_esp_transfer_done, K_MSEC(2));
    }
    if (status != 0) {
        LOG_INF("SPI master transfer timed out: %d", status);
        return -ETIMEDOUT;
    }

    if(!handshake_done){
        LOG_INF("SPI handshake transfer complete, received: 0x%02X 0x%02X 0x%02X 0x%02X", 
                rx_buf[0], rx_buf[1], rx_buf[2], rx_buf[3]);
    }
    

    return 0;
}

