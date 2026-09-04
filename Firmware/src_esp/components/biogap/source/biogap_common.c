/*
 * ----------------------------------------------------------------------
 *
 * File: biogap_common.c
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

#include "biogap.h"

static const char *TAG = "biogap_common.c";

/* DMA-capable handshake buffers */
static uint8_t handshake_pq_tx_buffer[4] __attribute__((aligned(4)));
static uint8_t handshake_pq_rx_buffer[4] __attribute__((aligned(4)));
static uint8_t expected_pq_handshake_buffer[4] __attribute__((aligned(4)));

// =============================================================================
// HELPER: Initial handshake
// =============================================================================

/** @brief Block until the NRF master completes the 4-byte SPI handshake. */
int initial_handshake_nrf_master_esp_slave_pq(void)
{
    ESP_LOGW(TAG, ">>> HANDSHAKE: Waiting for NRF master to pull CS and clock 4 bytes (marker=0x%02X)", HANDSHAKE_MARKER);
    
    handshake_pq_tx_buffer[0] = HANDSHAKE_MARKER;
    handshake_pq_tx_buffer[1] = HANDSHAKE_MARKER;
    handshake_pq_tx_buffer[2] = HANDSHAKE_MARKER;
    handshake_pq_tx_buffer[3] = HANDSHAKE_MARKER;
    handshake_pq_rx_buffer[0] = 0x00;
    handshake_pq_rx_buffer[1] = 0x00;
    handshake_pq_rx_buffer[2] = 0x00;
    handshake_pq_rx_buffer[3] = 0x00;
    expected_pq_handshake_buffer[0] = HANDSHAKE_RESPONSE_MARKER;
    expected_pq_handshake_buffer[1] = HANDSHAKE_RESPONSE_MARKER;
    expected_pq_handshake_buffer[2] = HANDSHAKE_RESPONSE_MARKER;
    expected_pq_handshake_buffer[3] = HANDSHAKE_RESPONSE_MARKER;

    spi_slave_transaction_t t = {0};
    t.length = 32;  /* 4 bytes = 32 bits */
    t.tx_buffer = handshake_pq_tx_buffer;
    t.rx_buffer = handshake_pq_rx_buffer;

    ESP_LOGW(TAG, ">>> TX buffer addr=%p, contents=[0x%02X 0x%02X 0x%02X 0x%02X]", 
            handshake_pq_tx_buffer, handshake_pq_tx_buffer[0], handshake_pq_tx_buffer[1], 
            handshake_pq_tx_buffer[2], handshake_pq_tx_buffer[3]);
    ESP_LOGW(TAG, ">>> RX buffer addr=%p", handshake_pq_rx_buffer);
    ESP_LOGW(TAG, ">>> Calling spi_slave_transmit() - will block until CS pulled low");
    
    if (!SPI_BUS_LOCK(portMAX_DELAY)) {
        ESP_LOGE(TAG, "Failed to lock SPI bus mutex for handshake");
        return -1;
    }

    esp_err_t ret = spi_slave_transmit(SPI_HOST_DEVICE, &t, portMAX_DELAY);
    SPI_BUS_UNLOCK();
    ESP_LOGW(TAG, ">>> HANDSHAKE: spi_slave_transmit returned: %s", esp_err_to_name(ret));
    
    if (ret == ESP_OK) {
        ESP_LOGI(TAG, "SPI xact OK: sent [0x%02X 0x%02X 0x%02X 0x%02X], received [0x%02X 0x%02X 0x%02X 0x%02X]", 
                handshake_pq_tx_buffer[0], handshake_pq_tx_buffer[1], handshake_pq_tx_buffer[2], handshake_pq_tx_buffer[3],
                handshake_pq_rx_buffer[0], handshake_pq_rx_buffer[1], handshake_pq_rx_buffer[2], handshake_pq_rx_buffer[3]);
    } else {
        ESP_LOGE(TAG, "SPI xact failed: %s", esp_err_to_name(ret));
        return -1;
    }
    
    /* Validate handshake response */
    if (memcmp(handshake_pq_rx_buffer, expected_pq_handshake_buffer, 4) == 0) {
        ESP_LOGI(TAG, "Handshake successful: received expected response");
        // Set the BIT flag to indicate that the initial handshake was successful
        xEventGroupSetBits(g_evt, B_BIOGAP_CONECTED);
        handshake_pq_done = true;
    } else {
        ESP_LOGW(TAG, "Handshake warning: received response does not match expected marker");
        return -1;
    }
    return 0;
}