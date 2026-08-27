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
#include "sensors/emg/emg_appl.h"
#include "sensors/wulpus/wulpus_appl.h"
#include "spi/spi_a.h"
#include <stdbool.h>

LOG_MODULE_REGISTER(wifi_sd_shield, LOG_LEVEL_INF);

/* ESP_PCKT_MAX_SIZE-sized stack buffers (process_esp_data(),
 * biogap_to_esp_transaction()) now take up to ~1700 bytes on their own at
 * ESP_PCKT_MAX_SIZE=850 -- sized up from 2048 to leave real headroom for the
 * rest of the call chain (spi_master_transceive(), handle_connectivity_command()
 * and everything it calls for WULPUS/EEG/EMG start). */
#define SPI_NRF_ESP_SENDER_STACK_SIZE 4096
#define SPI_NRF_ESP_RECEIVER_STACK_SIZE 4096
#define SPI_NRF_ESP_SENDER_PRIORITY 5           // Same priority as BLE send thread
#define SPI_NRF_ESP_RECEIVER_PRIORITY 5           // Same priority as BLE receive thread

K_MSGQ_DEFINE(esp_send_msgq, sizeof(esp_packet_t), ESP_SEND_QUEUE_SIZE, 4);
bool handshake_done = false;
/* Semaphore to synchronize NRF-ESP communication: both sender and receiver wait for handshake to complete */
K_SEM_DEFINE(handshake_complete_sem, 0, 2);

/*==============================================================================
 * Diagnostic status heartbeat
 *
 * The concurrent EMG+WULPUS-over-WiFi pipeline has several unbounded waits
 * spread across ads_spi_data.c, wulpus_appl.c and wifi_sd_spi_functions.c;
 * a stall in any one of them looks identical from the outside (streaming
 * just stops, nothing logs). Rather than keep guessing which specific wait
 * is stuck, periodically log every stage's state so whichever value stops
 * changing right before a stall pinpoints where it happened. Low priority
 * and a slow period so it never competes for CPU or the bus. */
/* Was 1024 -- too tight. LOG_INF() here formats 8 arguments into a fairly
 * long string; at a 2ms period that runs often enough that a real stack
 * overflow (Zephyr's own fault handler: "USAGE FAULT: Stack overflow
 * (context area not valid)") showed up during concurrent EMG+WULPUS Wi-Fi
 * testing. 2048 matches the minimum used by every other thread in this
 * codebase. */
#define STATUS_HEARTBEAT_STACK_SIZE 2048
#define STATUS_HEARTBEAT_PRIORITY   10
#define STATUS_HEARTBEAT_PERIOD_MS  2

static void status_heartbeat_thread(void *a, void *b, void *c) {
    ARG_UNUSED(a); ARG_UNUSED(b); ARG_UNUSED(c);
    k_thread_name_set(NULL, "heartbeat");
    while (1) {
        k_msleep(STATUS_HEARTBEAT_PERIOD_MS);
        LOG_INF("STATUS: emg_state=%d wulpus_streaming=%d wulpus_cfg_sent=%d "
                "spi_a_owner=%d ads_cp=%d wifi_cp=%d nrf_esp_comm_state=%d esp_send_msgq=%u/%u",
                emg_get_state(), wulpus_is_streaming(), wulpus_cfg_sent,
                spi_a_current_owner(), spi_a_get_ads_checkpoint(), spi_a_get_wifi_checkpoint(),
                nrf_esp_comm_state, k_msgq_num_used_get(&esp_send_msgq), ESP_SEND_QUEUE_SIZE);
    }
}

// Enable this only for logging

//K_THREAD_DEFINE(status_heartbeat_tid, STATUS_HEARTBEAT_STACK_SIZE, status_heartbeat_thread,
//                 NULL, NULL, NULL, STATUS_HEARTBEAT_PRIORITY, 0, 0);

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

    bool cmd_processed = false;

    while(!cmd_processed){    
        uint8_t spi_tx_buf[ESP_PCKT_MAX_SIZE]; // Dummy data to clock out the ESP response
        uint8_t spi_rx_buf[ESP_PCKT_MAX_SIZE]; 
        for (uint16_t i = 0; i < ESP_PCKT_MAX_SIZE; i++) {
            spi_tx_buf[i] = 0xBB; // Fill the rest of the buffer with zeros
            spi_rx_buf[i] = 0x00; // Initialize the receive buffer to zeros
        }
        
        (void)spi_master_transceive(spi_tx_buf, spi_rx_buf, sizeof(spi_tx_buf));
        LOG_INF("SPI transaction with ESP completed, received: 0x%02X 0x%02X 0x%02X 0x%02X", spi_rx_buf[0], spi_rx_buf[1], spi_rx_buf[2], spi_rx_buf[3]);
        // reset the flag
        //first_esp_data_ready = false;
        // Check if expected bytes match (header)

        // Check the header byte
        if(spi_rx_buf[0] != ESP_SPI_HEADER){
            LOG_WRN("SPI transaction response has invalid header: received header 0x%02X, expected 0x%02X", spi_rx_buf[0], ESP_SPI_HEADER);
            LOG_WRN("=== SPI FAILURE - NRF53 HALTED ===");
            //while (1) {k_sleep(K_FOREVER);}
        }
        else{
            uint8_t payload_size = spi_rx_buf[1]; // The second byte indicates the payload size
            // Check the tailer byte based on the payload size
            if(spi_rx_buf[2 + payload_size] != ESP_SPI_TAILER){
                LOG_WRN("SPI transaction response has invalid tailer: received tailer 0x%02X, expected 0x%02X", spi_rx_buf[2 + payload_size], ESP_SPI_TAILER);
                LOG_WRN("=== SPI FAILURE - NRF53 HALTED ===");
                //while (1) {k_sleep(K_FOREVER);}
            }
            else{
                handle_connectivity_command(&spi_rx_buf[2], payload_size); // Process the payload
                cmd_processed = true;
            }
        }
        serve_esp_requests = false; // Clear pending request flag after processing to allow sender thread to resume if it was yielding
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
    k_thread_name_set(NULL, "esp_receiver");
    LOG_INF("SPI NRF-ESP receiver thread started, waiting for handshake to complete...");
    k_sem_take(&handshake_complete_sem, K_FOREVER);
    LOG_INF("SPI NRF-ESP receiver took semaphore, starting receiver thread"); 

    while(1){
        // TEMP DIAGNOSTIC: see wifi_sd_shield_defs.h
        if (wifi_sd_paused) {
            esp_data_ready = false;
            k_msleep(1);
            continue;
        }
        if(esp_data_ready){
            LOG_INF("ESP data ready flag set, processing incoming data...");
            process_esp_data();
        }
        else {
            /* Yield CPU when no DRDY event is pending. */
            k_msleep(1);
        }

    }
}

/**
 * @brief Add Data to ESP Send Buffer
 *
 * Enqueues data into the send message queue for transmission to ESP32.
 * Supports variable packet sizes.
 * This function is the counterpart of add_data_to_send_buffer() used for BLE transmission.
 *
 * @param data The pointer to the byte array to be sent over BLE.
 * @param size The size of the data to send in bytes.
 */
void add_data_to_esp_send_buffer(uint8_t *data, uint16_t size) {

    int ret;
    /* esp_packet_t is ~852 bytes -- too big for a stack local here. This
     * function is called from the ADS SPI-completion ISR (via
     * ads_spim_handler_done(), for the EMG path) as well as from
     * wulpus_esp_thread() (normal thread, for WULPUS). ISR context on this
     * SoC runs on a small stack shared by all nested interrupts, separate
     * from every application thread's own stack -- an 852-byte local here
     * was exactly the "USAGE FAULT: stack overflow" seen during concurrent
     * EMG+WULPUS Wi-Fi streaming (confirmed by elimination: main.c's
     * per-thread stack dump at fault time showed every application thread
     * with healthy headroom, which only makes sense if the overflow was on
     * a stack that dump doesn't even enumerate).
     *
     * Static instead of a stack local removes that footprint, but a single
     * shared static would let an ISR call preempt a thread call mid-write
     * and corrupt the packet in flight -- so two separate buffers, picked
     * by k_is_in_isr(), keep the two call paths from ever touching the same
     * memory. discard_scratch is shared across both since its contents are
     * immediately thrown away either way. */
    static esp_packet_t isr_packet_buf;
    static esp_packet_t thread_packet_buf;
    static esp_packet_t discard_scratch;
    esp_packet_t *packet = k_is_in_isr() ? &isr_packet_buf : &thread_packet_buf;

    // Validate size
    if (size > ESP_PCKT_MAX_SIZE) {
        LOG_ERR("Packet size %d exceeds max %d", size, ESP_PCKT_MAX_SIZE);
        return;
    }

    packet->size = size;
    memcpy(packet->data, data, size);
    //LOG_INF("Enqueuing packet for ESP sending, size: %d", size);

    /* Called from SPI-completion ISR context (via ads_spim_handler_done()),
     * so this must never block -- K_FOREVER, or worse k_sleep(), here would
     * not just stall a thread: there is no thread to suspend from ISR
     * context, so it locks up the entire CPU forever (this was the "random
     * silent freeze, even the heartbeat thread stops" during concurrent
     * EMG+WULPUS Wi-Fi streaming -- confirmed via the diagnostic checkpoint
     * staying frozen exactly at "ADS transfer done, about to hand data off",
     * which is this call). If full, drop the oldest queued packet to make
     * room and move on: a stalled sender loses old data rather than
     * hanging everything waiting for space. */
    ret = k_msgq_put(&esp_send_msgq, packet, K_NO_WAIT);
    if (ret != 0) {
        k_msgq_get(&esp_send_msgq, &discard_scratch, K_NO_WAIT);
        ret = k_msgq_put(&esp_send_msgq, packet, K_NO_WAIT);
        LOG_ERR("ESP send queue full, dropped oldest packet");
    }
    if (ret != 0) {
        LOG_ERR("Failed to enqueue data for ESP sending (err %d)", ret);
    } else {
        LOG_DBG("Data enqueued for ESP sending: %d", size);
    }
}


/** @brief SPI sender thread for NRF and ESP communication during actual streaming*/
void spi_nrf_esp_sender_thread(void *arg1, void *arg2, void *arg3)
{
    /* Enbale this thread only if the communication has bee established*/
    ARG_UNUSED(arg1);
    ARG_UNUSED(arg2);
    ARG_UNUSED(arg3);
    k_thread_name_set(NULL, "esp_sender");
    LOG_INF("SPI NRF-ESP sender thread started, waiting for handshake to complete...");

    k_sem_take(&handshake_complete_sem, K_FOREVER);
    LOG_INF("SPI NRF-ESP sender took semaphore, starting sender thread");
    esp_packet_t packet;
    while(1){
            // TEMP DIAGNOSTIC: see wifi_sd_shield_defs.h
            if (wifi_sd_paused) {
                k_msleep(1);
                continue;
            }
            int ret = k_msgq_get(&esp_send_msgq, &packet, K_FOREVER);
            //LOG_INF("SPI NRF-ESP sender got from queue %d bytes for sending", packet.size);
            if (ret == 0) {
                // Process packet for transmission to ESP32

                // make sure that the read-transaction from the ESP has the priority to avoid SPI bus contention. 
                if (serve_esp_requests) {
                    LOG_INF("ESP requests pending, prioritizing incoming data processing over sending");
                    /* Yield to allow processing of incoming ESP data. Was
                     * k_msleep(1) (1ms) -- at 2000 Hz ADS sampling that's 2+
                     * full conversion periods per hit; the RTC system tick
                     * is ~30.5us, so a much shorter sleep still yields
                     * without needlessly delaying the next real EXG send. */
                    k_sleep(K_USEC(50));
                    continue; // Skip this send iteration to prioritize incoming data
                }

                // check we are in the correct state to send data to ESP
                if (nrf_esp_comm_state != SEND_TO_ESP) {
                    LOG_INF("NRF-ESP comm state not ready for sending (current state: %d), yielding to allow state change", nrf_esp_comm_state);
                    // Yield and retry later
                    k_sleep(K_USEC(50));
                    continue;
                }
                //LOG_INF("Starting BIOGAP to ESP transaction, size: %d", packet.size);
                ret = biogap_to_esp_transaction(&packet);
                if (ret == -EBUSY) {
                    /* Transient SPI_A bus stall, already logged where it was
                     * detected -- packet dropped, keep streaming. */
                    continue;
                }
                if (ret != 0) {
                    LOG_ERR("Failed to send packet to ESP32 (err %d)", ret);
                    // halt
                    LOG_ERR("=== SPI FAILURE in spi_nrf_esp_sender_thread - NRF53 HALTED ===");
                    while (1)
                    {
                        k_sleep(K_FOREVER);
                    }
                }
            } 
            else {
                LOG_ERR("Failed to get packet from esp_send_msgq (err %d)", ret);
            }
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
    uint8_t hs_completed = 0;
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
                hs_completed++;
                LOG_INF("==SPI handshake attempt %d done==",hs_completed);
                k_sleep(K_MSEC(2000));
                
                
                if (hs_completed == 1){
                    handshake_done = true;
                    
                    k_sem_give(&handshake_complete_sem);    // Signal sender thread to start
                    k_sem_give(&handshake_complete_sem);    // Signal receiver thread to start
                    LOG_INF("Initial handshake with ESP32 successful, semaphore given ");
                    return 0;
                }
                //return 0; 
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
