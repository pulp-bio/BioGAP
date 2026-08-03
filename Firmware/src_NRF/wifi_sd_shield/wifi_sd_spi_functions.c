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
#include "spi/spi_a.h"
LOG_MODULE_REGISTER(wifi_sd_spi_functions, LOG_LEVEL_INF);

/* Serialize async SPIM transfers so callers can treat the API as blocking. */
K_SEM_DEFINE(spi_nrf_esp_transfer_done, 0, 1);

/**
 * @brief SPI_A completion handler for WiFi/SD transfers
 *
 * Called by the shared spi/spi_a.c interrupt dispatcher when the in-flight
 * transfer's owner is SPI_A_OWNER_WIFI_SD. CS deassertion is already
 * handled generically by spi_a.c (it was given &esp_cs_gpio in
 * spi_a_begin_transfer()); this just wakes up the blocked caller.
 */
void wifi_sd_spim_transfer_complete(void) {
    k_sem_give(&spi_nrf_esp_transfer_done);
}

/**
 * @brief SPI master transmit/receive transaction
 *
 * Performs a full-duplex SPI master transaction with the ESP32 slave, on
 * the shared SPI_A bus (spi/spi_a.h). Holds spi_a_mutex for the whole
 * transfer (kickoff through completion) since this call already blocks on
 * a completion semaphore -- unlike the ADS driver, there's no separate
 * caller-side wait to fold this into.
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

    k_mutex_lock(&spi_a_mutex, K_FOREVER);

    /* ADS releases spi_a_mutex right after kicking off a transfer, not once
     * it physically completes. nrfx_spim_xfer() below would reprogram the
     * same peripheral's DMA while that transfer is still in flight, so
     * spin here (tens of microseconds, same order as an ADS transfer
     * itself -- no sleep, same idiom as ads_spi_data.c's spi_xfer_done
     * wait) until spi_a_current_owner() confirms ADS is really done. */
    while (spi_a_current_owner() == SPI_A_OWNER_ADS)
        ;

    /* Drain any stale completion before starting a new async transfer. */
    while (k_sem_take(&spi_nrf_esp_transfer_done, K_NO_WAIT) == 0) {
    }

    /* A plain runtime check, not #if: NRFX_MHZ_TO_HZ()'s expansion isn't
     * guaranteed to be valid in a preprocessor constant expression (nrfx
     * doesn't write its macros with #if-safety in mind), so comparing here
     * instead of at compile time avoids relying on that. The cost of one
     * comparison is negligible next to the reinit it guards. See
     * SPI_A_ESP_STREAMING_FREQ_HZ's doc comment (spi_a.h). */
    if (SPI_A_ADS_STREAMING_FREQ_HZ != SPI_A_ESP_STREAMING_FREQ_HZ) {
        spi_a_set_frequency(SPI_A_ESP_STREAMING_FREQ_HZ);
    }

    /* Configure SPI transaction buffers */
    nrfx_spim_xfer_desc_t xfer = {
        .p_tx_buffer = (uint8_t *)tx_buf,
        .tx_length = tx_buf ? len : 0,
        .p_rx_buffer = rx_buf,
        .rx_length = rx_buf ? len : 0,
    };

    /* Execute transfer (blocking) */
    spi_a_begin_transfer(SPI_A_OWNER_WIFI_SD, &esp_cs_gpio);
    nrfx_err_t status = nrfx_spim_xfer(&spi_a_inst, &xfer, 0);
    if(!handshake_done){
        LOG_INF("SPI master transceive initiated, waiting for handshake to complete...");
        LOG_INF("SPI master transceive initiated, sent: 0x%02X 0x%02X 0x%02X 0x%02X",
            tx_buf[0], tx_buf[1], tx_buf[2], tx_buf[3]);
    }
    if (status != NRFX_SUCCESS) {
        LOG_INF("SPI master transfer failed: %d", status);
        if (SPI_A_ADS_STREAMING_FREQ_HZ != SPI_A_ESP_STREAMING_FREQ_HZ) {
            spi_a_set_frequency(SPI_A_ADS_STREAMING_FREQ_HZ);
        }
        k_mutex_unlock(&spi_a_mutex);
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
        status = k_sem_take(&spi_nrf_esp_transfer_done, K_USEC(500));                 //M_MSEC(1)
    }

    /* Restore the ADS rate now regardless of outcome -- ADS's own next
     * transfer must not run at the ESP rate. */
    if (SPI_A_ADS_STREAMING_FREQ_HZ != SPI_A_ESP_STREAMING_FREQ_HZ) {
        spi_a_set_frequency(SPI_A_ADS_STREAMING_FREQ_HZ);
    }

    if (status != 0) {
        LOG_INF("SPI master transfer timed out: %d", status);
        k_mutex_unlock(&spi_a_mutex);
        return -ETIMEDOUT;
    }

    if(!handshake_done){
        LOG_INF("SPI handshake transfer complete, received: 0x%02X 0x%02X 0x%02X 0x%02X",
                rx_buf[0], rx_buf[1], rx_buf[2], rx_buf[3]);
    }

    k_mutex_unlock(&spi_a_mutex);
    return 0;
}


int biogap_to_esp_transaction(esp_packet_t *packet){

    /* NRF53 as SPI master sends packet to ESP32 slave */
    if (packet->size == 0) {
        LOG_WRN("Empty packet, nothing to send");
        return -EINVAL;
    }

    /* Determine actual send length: if producer left size as max, try to find tailer marker to avoid sending garbage */
    size_t send_len = packet->size;
    uint8_t spi_rx_buf[send_len];
    memset(spi_rx_buf, 0, send_len);
    int ret = spi_master_transceive(packet->data, spi_rx_buf, send_len);
    if(ret != 0){
            LOG_ERR("SPI transaction failed (ret=%d)", ret);
            LOG_ERR("=== SPI FAILURE - NRF53 HALTED ===");
            while (1) {k_sleep(K_FOREVER);}
            return ret;
    }

    // check if expected received header and tailer are correct
    if(spi_rx_buf[0] != ESP_SPI_HEADER || spi_rx_buf[send_len - 1] != ESP_SPI_TAILER){
            LOG_ERR("SPI transaction response has invalid header/tailer: received header 0x%02X, expected 0x%02X; received tailer 0x%02X, expected 0x%02X",
                spi_rx_buf[0], ESP_SPI_HEADER, spi_rx_buf[send_len - 1], ESP_SPI_TAILER);
            LOG_ERR("=== SPI FAILURE - NRF53 HALTED ===");
            while (1) {k_sleep(K_FOREVER);}
            return -EIO;   
    }
    
    //check if the received data contain an implicit stop command from ESP
    if(spi_rx_buf[2] == ESP_STOP_COMMAND){
        LOG_INF("Received ESP STOP command, resetting NRF-ESP communication state and waiting for stop sensor command");
        /* Save the opcode before spi_rx_buf gets reused below as the RX
        * buffer for the ack transaction -- otherwise handle_connectivity_command()
        * ends up dispatching whatever the ESP responded with to the ACK,
        * not the original STOP request's opcode byte. */

        uint8_t stop_opcode = spi_rx_buf[3];
        handle_connectivity_command(&stop_opcode, 1);
        

        // reset state to idle to block sending data to ESP
        nrf_esp_comm_state = NRF_ESP_IDLE;


        // Explicitly acknowledge STOP with a dedicated transaction whose
        // header byte has NRF_STOP_ACK_MASK set.
        //
        // Retried: this transceive races with the ADS read thread on the
        // same SPI_A bus (same thread priority, CONFIG_TIMESLICING is off,
        // and process_ads_data() busy-spins on spi_xfer_done with no yield
        // in between), so under real acquisition load a single attempt can
        // miss spi_master_transceive()'s short post-handshake completion
        // timeout even though nothing is actually wrong. The ESP has no
        // fallback of its own here -- enter_stop_quiesce_state() just polls
        // forever for this exact ack -- so silently giving up after one try
        // left it waiting indefinitely. ads_stop() (via emg_stop_streaming(),
        // already called above) should quiesce that contention within a
        // sample period or two, so a handful of retries is normally enough.
        memset(spi_rx_buf, 0, send_len);
        uint8_t dummy_buff[send_len];
        memset(dummy_buff, 0, sizeof(dummy_buff));
        dummy_buff[0] = ESP_SPI_HEADER | NRF_STOP_ACK_MASK; // set the ack bit in the header
        dummy_buff[send_len - 1] = ESP_SPI_TAILER;

        int ack_ret;
        int ack_attempts = 0;
        const int max_ack_attempts = 50;
        do {
            ack_ret = spi_master_transceive(dummy_buff, spi_rx_buf, send_len);
            if (ack_ret != 0) {
                ack_attempts++;
                LOG_WRN("Failed to send STOP ack to ESP (ret=%d), attempt %d/%d", ack_ret, ack_attempts, max_ack_attempts);
            }
        } while (ack_ret != 0 && ack_attempts < max_ack_attempts);

        if (ack_ret != 0) {
            LOG_ERR("Giving up on STOP ack to ESP after %d attempts", ack_attempts);
        }
    }


    return 0;
}


