/*
 * ----------------------------------------------------------------------
 *
 * File: wifi_sd_shield_appl.c
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
 * @file wifi_sd_shield_appl.c
 * @brief Wi-Fi and SD Shield Application Interface
 *
 * This file contains the application-level implementation for interacting with
 * the Wi-Fi and SD shields. 
*/

#include "wifi_sd_shield_appl.h"
#include "wifi_sd_shield_inits.h"
#include "wifi_sd_spi_functions.h"
#include "core/command_dispatcher.h"
#include <stdbool.h>

LOG_MODULE_REGISTER(wifi_sd_shield, LOG_LEVEL_INF);

#define SPI_NRF_ESP_SENDER_STACK_SIZE 2048
#define SPI_NRF_ESP_RECEIVER_STACK_SIZE 2048
#define SPI_NRF_ESP_SENDER_PRIORITY 5           // Same priority as BLE send thread
#define SPI_NRF_ESP_RECEIVER_PRIORITY 5           // Same priority as BLE receive thread

bool handshake_done = false;
/* Semaphore to synchronize NRF-ESP communication: producer waits for handshake to complete */
K_SEM_DEFINE(handshake_complete_sem, 0, 1);

/**
 * @brief Process ESP data when DRDY interrupt occurs 
 *
*/
void process_esp_data(void) {
  // LOG_INF("Processing ESP data...");
  if (esp_data_ready) {
    // LOG_INF("ESP DATA READY interrupt received");
    // Clear flag first to avoid missing next interrupt
    esp_data_ready = false;
    
    // Allocate buffer for incoming data. Need to allocate 4 Bytes for ESP DMA transaction
    uint8_t spi_tx_buf[4] = {0x00, 0x00, 0x00, 0x00}; 
    uint8_t spi_rx_buf[4] = {0x00, 0x00, 0x00, 0x00};
    (void)spi_master_transceive(spi_tx_buf, spi_rx_buf, sizeof(spi_tx_buf));
    LOG_INF("SPI transaction with ESP completed, received: 0x%02X 0x%02X 0x%02X 0x%02X", spi_rx_buf[0], spi_rx_buf[1], spi_rx_buf[2], spi_rx_buf[3]);
    // Check if expected bytes match
    if (spi_rx_buf[0] == ESP_SPI_HEADER && spi_rx_buf[3] == ESP_SPI_TAILER) {
        LOG_INF("Received valid data from ESP: 0x%02X 0x%02X 0x%02X 0x%02X", spi_rx_buf[0], spi_rx_buf[1], spi_rx_buf[2], spi_rx_buf[3]);
        uint8_t cmd = spi_rx_buf[1]; 
        handle_connectivity_command(cmd);
        }
    }

  k_sleep(K_USEC(1));
}


/** @brief SPI receiver thread for NRF and ESP communication*/
/* This thread should be considered the counterpart of process_received_data_thread() used for BLE data reception.
*/

void spi_nrf_esp_receiver_thread(void *arg1, void *arg2, void *arg3)
{
    /* Enable this thread only if the commfirst hadns communication has been established*/
    ARG_UNUSED(arg1);
    ARG_UNUSED(arg2);
    ARG_UNUSED(arg3);
    LOG_INF("SPI NRF-ESP receiver thread started, waiting for handshake to complete...");
    k_sem_take(&handshake_complete_sem, K_FOREVER);
    LOG_INF("SPI NRF-ESP receiver took semaphore, starting receiver thread"); 

    while(1){
        if(esp_data_ready){
            LOG_INF("ESP data ready flag set, processing incoming data...");
            process_esp_data();
        } else {
            /* Yield CPU when no DRDY event is pending. */
            k_msleep(1);
        }

    }
}

/** @brief SPI sender thread for NRF and ESP communication*/
void spi_nrf_esp_sender_thread(void *arg1, void *arg2, void *arg3)
{
    /* Enbale this thread only if the communication has bee established*/
    ARG_UNUSED(arg1);
    ARG_UNUSED(arg2);
    ARG_UNUSED(arg3);
    LOG_INF("SPI NRF-ESP sender thread started, waiting for handshake to complete...");

    k_sem_take(&handshake_complete_sem, K_FOREVER);
    LOG_INF("SPI NRF-ESP sender took semaphore, starting sender thread");
    while(1){
            // Important. we need to check that the data-ready flag is not set

            // To be Updated
            // // wait notification from producer that data are available in the queue
            // exg_packet_t packet;
            // int ret = k_msgq_get(&send_msgq, &packet, K_FOREVER);
            // LOG_INF("Recieved new packet with header 0x%02X, counter %d, tailer 0x%02X from producer thread",
            //         packet.data[0], (packet.data[1] | packet.data[2] <<8), packet.data[EXG_PCK_LNGTH-1]);
            // if (ret != 0) {
            //         LOG_ERR("k_msgq_get failed (%d)! System halted.", ret);
            //         LOG_ERR("=== QUEUE FAILURE - NRF53 HALTED ===");
            //         while (1) {k_sleep(K_FOREVER);}
            // }
            // biogap_to_esp_transaction(&packet, &time_last_log, &time_curr_log);

                /* Placeholder thread: avoid tight spinning until queue-based sender is enabled. */
                k_msleep(10);

    }

}



/** @brief Perform initial handshake with ESP32 to verify that the connection is established */
int initial_handshake_nrf_esp() {


    uint8_t max_handshake_attempts = 5;
    uint8_t attempt = 0; 

    static uint8_t spi_tx_buf[4] = {HANDSHAKE_MARKER, HANDSHAKE_MARKER, HANDSHAKE_MARKER, HANDSHAKE_MARKER};
    static uint8_t spi_rx_buf[4] = {0x00, 0x00, 0x00, 0x00};
    static const uint8_t expected_handshake_response[4] = {
                HANDSHAKE_MARKER_RCV,
                HANDSHAKE_MARKER_RCV,
                HANDSHAKE_MARKER_RCV,
                HANDSHAKE_MARKER_RCV,
    };
    
    while (!handshake_done && attempt < max_handshake_attempts){
        
        LOG_INF("Attempting SPI handshake with ESP32 (attempt %d/%d)", attempt+1, max_handshake_attempts);
        /* SPI master transaction for handshake */
        memset(spi_rx_buf, 0x00, sizeof(spi_rx_buf));

        int ret = spi_master_transceive(spi_tx_buf, spi_rx_buf, sizeof(spi_tx_buf));
        attempt++;
            
        if (ret == 0) {
            LOG_INF("SPI handshake, received: 0x%02X 0x%02X 0x%02X 0x%02X", spi_rx_buf[0], spi_rx_buf[1], spi_rx_buf[2], spi_rx_buf[3]);
            if (memcmp(spi_rx_buf, expected_handshake_response, sizeof(expected_handshake_response)) == 0) {
                //LOG_INF("SPI handshake accepted: expected marker echoed by slave.");
                // wait couple of seconds

                //k_sleep(K_MSEC(2000));
                handshake_done = true;
                k_sem_give(&handshake_complete_sem);    // Signal sender and receiver threads to start
                LOG_INF("Initial handshake with ESP32 successful, semaphore given ");
                return 0; 
            }
            
            else if (((spi_rx_buf[0] | spi_rx_buf[1] | spi_rx_buf[2] | spi_rx_buf[3]) == 0x00) ||
                        ((spi_rx_buf[0] & spi_rx_buf[1] & spi_rx_buf[2] & spi_rx_buf[3]) == 0xFF)) {
                LOG_WRN("SPI handshake rejected: RX is all 0x00/0xFF, likely slave absent, not ready, or MISO floating.");

                // wait 2 seconds before retrying handshake, to avoid spamming logs and give slave time to boot
                handshake_done = false;

            } 
            else {
                LOG_WRN("SPI handshake rejected: unexpected response [0x%02X 0x%02X 0x%02X 0x%02X]",
                        spi_rx_buf[0], spi_rx_buf[1], spi_rx_buf[2], spi_rx_buf[3]);
                
                handshake_done = false;
            }

        } 
        else {
                LOG_ERR("SPI handshake failed (ret=%d)", ret);
                // sleep 2 seconds before retrying
                handshake_done = false;
        }
        LOG_INF("Retrying SPI handshake with ESP32...");
        LOG_INF("Retry delay start (2s, busy wait)");
        //k_busy_wait(5000000000); // 5 seconds in microseconds. Keeps CPU occupied 
        k_sleep(K_MSEC(3000)); // 2 seconds sleep, allows other threads to run and reduces CPU usage during handshake retries
        LOG_INF("Retry delay done, continuing handshake loop");
        continue;


    }
    return -1; // Handshake failed after max attempts
}

#if defined (CONFIG_WI_FI)
    K_THREAD_DEFINE(spi_nrf_esp_sender_tid, SPI_NRF_ESP_SENDER_STACK_SIZE, spi_nrf_esp_sender_thread, NULL, NULL, NULL,SPI_NRF_ESP_SENDER_PRIORITY,
                    0, 0);
    K_THREAD_DEFINE(spi_nrf_esp_receiver_tid, SPI_NRF_ESP_RECEIVER_STACK_SIZE, spi_nrf_esp_receiver_thread, NULL, NULL, NULL,SPI_NRF_ESP_RECEIVER_PRIORITY,
                    0, 0);

#endif 
