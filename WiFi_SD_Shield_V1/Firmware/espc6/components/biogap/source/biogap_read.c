/*
 * ----------------------------------------------------------------------
 *
 * File: biogap_read.c
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
#include "common.h"
#include <string.h>
#include <stdbool.h>
#include <stdint.h>
#include <inttypes.h>
#include <stdlib.h>
#include "esp_log.h"
#include "esp_timer.h"
#include "esp_rom_sys.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "freertos/ringbuf.h"
#include "driver/gptimer.h"
#include "driver/gpio.h"
#include "driver/spi_master.h"
#include "driver/spi_slave.h"
#include "driver/spi_common.h"
#include "esp_heap_caps.h"
#include "esp_intr_alloc.h"
#include "gui_task.h"

#define BIOGAP_READ_TAG "biogap_read_hs"

#define PACKET_SZ SPI_FROM_BIOGAP_MAX_SIZE

// =============================================================================
// GLOBALS FOR PRE-QUEUE PATTERN
// =============================================================================
uint8_t *rx_bufs[QUEUE_COUNT] = {NULL};           /* RX buffers for each pre-queued desc */
uint8_t *tx_bufs[QUEUE_COUNT] = {NULL};           /* TX buffers for each pre-queued desc */
spi_slave_transaction_t trans_descs[QUEUE_COUNT]; /* Transaction descriptors */
uint8_t *sendbuf_persistent = NULL;               /* Single TX buffer for all desciptors */
static bool prequeued = false;
bool handshake_pq_done = false;

// =============================================================================
// HELPER: Pre-queue all RX transactions
// =============================================================================
/**
 * @brief Pre-queue QUEUE_COUNT transactions so slave remains armed at all times.
 * 
 * This function initializes the transaction descriptors and queues them all,
 * so the slave is ready to receive the first packet immediately.
 * 
 * @return ESP_OK on success, ESP_FAIL if any queue operation fails
 */
static esp_err_t prequeue_transactions(void)
{
    if (!SPI_BUS_LOCK(portMAX_DELAY)) {
        ESP_LOGE(BIOGAP_READ_TAG, "Failed to lock SPI bus mutex while pre-queueing transactions");
        return ESP_FAIL;
    }

    for (int i = 0; i < QUEUE_COUNT; i++) {
        memset(&trans_descs[i], 0, sizeof(trans_descs[i]));
        trans_descs[i].length = PACKET_SZ * 8;  /* Convert bytes to bits */
        trans_descs[i].tx_buffer = tx_bufs[i];
        trans_descs[i].rx_buffer = rx_bufs[i];

        memcpy(tx_bufs[i], sendbuf_persistent, PACKET_SZ);

        esp_err_t ret = spi_slave_queue_trans(SPI_HOST_DEVICE, &trans_descs[i], 0);
        if (ret != ESP_OK) {
            ESP_LOGE(BIOGAP_READ_TAG, "Failed to pre-queue transaction %d: %s", i, esp_err_to_name(ret));
            SPI_BUS_UNLOCK();
            return ret;
        }
    }
    SPI_BUS_UNLOCK();
    return ESP_OK;
}

// =============================================================================
// HELPER: Add packet to ringbuffer
// =============================================================================
/** @brief Copy a packet into biogap_ringbuf for tx_to_gui() to relay to BioGUI. */
esp_err_t add_to_ringbuffer(const uint8_t *data, size_t len)
{
    void *rb_ptr = NULL;
    if (xRingbufferSendAcquire(biogap_ringbuf, &rb_ptr, len, 0) != pdTRUE || rb_ptr == NULL) {
        ESP_LOGE(BIOGAP_READ_TAG, "Failed to acquire ringbuffer space");
        return ESP_FAIL;
    }
    memcpy(rb_ptr, data, len);
    xRingbufferSendComplete(biogap_ringbuf, rb_ptr);
    return ESP_OK;
}

// =============================================================================
// HELPER: Validate packet markers
// =============================================================================
/** @brief Check NRF_EXG_HEADER/TAILER framing on a received (RX) buffer. */
static inline bool is_valid_packet(const uint8_t *data, const size_t packet_size)
{

    /* Expand to match with the expected packet format*/
    if (data[0] == NRF_EXG_HEADER && data[packet_size - 1] == NRF_EXG_TAILER){
        /*Header and tailer for ExG data streaming*/
        return true;
    }

    // elif(data[0] == WULPUS_HDR_XFER_0){
    //     return true;}
    // elif(data[0] == WULPUS_HDR_XFER_1){
    //     return true;}
    // elif(data[0] == WULPUS_HDR_XFER_2){
    //     return true;}
    // elif(data[0] == WULPUS_HDR_XFER_3){
    //     return true;}

    return false; 
}

/** @brief Free the DMA-capable TX/RX pre-queue buffers. */
esp_err_t free_prequeue_resources(void)
{
    if (sendbuf_persistent) {
        free(sendbuf_persistent);
        sendbuf_persistent = NULL;
        ESP_LOGW(BIOGAP_READ_TAG, "Freed TX buffer");
    }

    for (int i = 0; i < QUEUE_COUNT; i++) {
        if (rx_bufs[i]) {
            free(rx_bufs[i]);
            rx_bufs[i] = NULL;
        }
    }
    for (int i = 0; i < QUEUE_COUNT; i++) {
        if (tx_bufs[i]) {
            free(tx_bufs[i]);
            tx_bufs[i] = NULL;
        }
    }
    return ESP_OK;
}

/** @brief Allocate (or reset) the DMA-capable TX/RX pre-queue buffers. */
esp_err_t allocate_prequeue_resources(void)
{
    if (sendbuf_persistent != NULL) {
        ESP_LOGW(BIOGAP_READ_TAG, "TX buffer already allocated, reinitializing for fresh start");
    }

    if (sendbuf_persistent == NULL) {
        sendbuf_persistent = heap_caps_malloc(PACKET_SZ, MALLOC_CAP_DMA);
        if (!sendbuf_persistent) {
            ESP_LOGE(BIOGAP_READ_TAG, "Failed to allocate sendbuf");
            return ESP_FAIL;
        }
    }

    memset(sendbuf_persistent, 0, PACKET_SZ);
    sendbuf_persistent[0] = ESP_EXG_HEADER;
    // fill everything apart from header, first and second bytes with ESP_EXG_TAILER
    for(uint16_t i=4;i<PACKET_SZ-1;i++){
        sendbuf_persistent[i] = ESP_EXG_TAILER;
    }
    sendbuf_persistent[PACKET_SZ - 1] = ESP_EXG_TAILER;
    sendbuf_persistent[1] = 0;
    sendbuf_persistent[2] = 0;

    for (int i = 0; i < QUEUE_COUNT; i++) {
        if (rx_bufs[i] == NULL) {
            rx_bufs[i] = heap_caps_malloc(PACKET_SZ, MALLOC_CAP_DMA);
            if (!rx_bufs[i]) {
                ESP_LOGE(BIOGAP_READ_TAG, "Failed to allocate RX buffer %d", i);
                free_prequeue_resources();
                return ESP_FAIL;
            }
        }
        memset(rx_bufs[i], 0, PACKET_SZ);

        if (tx_bufs[i] == NULL) {
            tx_bufs[i] = heap_caps_malloc(PACKET_SZ, MALLOC_CAP_DMA);
            if (!tx_bufs[i]) {
                ESP_LOGE(BIOGAP_READ_TAG, "Failed to allocate TX buffer %d", i);
                free_prequeue_resources();
                return ESP_FAIL;
            }
        }
        memset(tx_bufs[i], 0, PACKET_SZ);
        tx_bufs[i][0] = ESP_EXG_HEADER;
        tx_bufs[i][PACKET_SZ - 1] = ESP_EXG_TAILER;
    }

    ESP_LOGI(BIOGAP_READ_TAG, "Allocated shared quiesce TX buffer: %p", sendbuf_persistent);
    ESP_LOGI(BIOGAP_READ_TAG, "Allocated %d RX buffers (DMA-capable, total %.1f KB)",
             QUEUE_COUNT, (float)(QUEUE_COUNT * PACKET_SZ) / 1024.0);
    return ESP_OK;
}

/** @brief Handle STOP command received from the GUI: 
 * piggyback the request, confirm the NRF's ACK, then reset/re-arm the SPI bus. */
static esp_err_t enter_stop_quiesce_state(void)
{
    ESP_LOGI(BIOGAP_READ_TAG, "Stop requested, idling until next START (sender will drain)");

    spi_slave_transaction_t *ret_trans;
    esp_err_t ret;

    if (sendbuf_persistent) {
        memset(sendbuf_persistent, 0, PACKET_SZ);
        sendbuf_persistent[0] = ESP_EXG_HEADER;
        sendbuf_persistent[2] = ESP_STOP_COMMAND;
        sendbuf_persistent[3] = rx_gui_data_to_fwd[0];
        for(uint16_t i=4;i<PACKET_SZ-1;i++){
            sendbuf_persistent[i] = ESP_EXG_TAILER;
        }
    }

    for (int i = 0; i < QUEUE_COUNT; i++) {
        if (tx_bufs[i]) {
            memcpy(tx_bufs[i], sendbuf_persistent, PACKET_SZ);
        }
    }
    bool stop_delivered = false;
    uint8_t send_stop_command_count = 0;
    bool stop_ack_received = false;
    
    while(stop_ack_received == false) {
        ret = spi_slave_get_trans_result(SPI_HOST_DEVICE, &ret_trans, pdMS_TO_TICKS(50));
        if (ret == ESP_OK) {
            uint8_t *tx_data = (uint8_t *)ret_trans->tx_buffer;
            uint8_t *rx_data = (uint8_t *)ret_trans->rx_buffer;

            ESP_LOGI(BIOGAP_READ_TAG,
                    "Sent:     [0]=0x%02X [1]=0x%02X [2]=0x%02X [3]=0x%02X",
                    tx_data[0], tx_data[1], tx_data[2], tx_data[3]);

            ESP_LOGI(BIOGAP_READ_TAG,
                    "Received: [0]=0x%02X [1]=0x%02X [2]=0x%02X [3]=0x%02X",
                    rx_data[0], rx_data[1], rx_data[2], rx_data[3]);

            bool stop_ack = (rx_data[0] & NRF_STOP_ACK_MASK) != 0;
            uint8_t header = rx_data[0] & ~NRF_STOP_ACK_MASK;

            if ((header == ESP_SPI_HEADER) && stop_ack) {
                ESP_LOGI(BIOGAP_READ_TAG, "STOP ACK received");
                stop_ack_received = true;
                send_stop_command_count = 0;
            }
            else {
                send_stop_command_count++;
            }

            /* Re-queue regardless of outcome -- tx_bufs[]'s content is
             * untouched since the stomp above, so this just re-arms the
             * same STOP pattern. Without this, the NRF's regular relay
             * thread (draining whatever backlog it had, plus its own
             * retried STOP-ack attempts) consumes QUEUE_COUNT slots one by
             * one and is never given anything back; once all of them are
             * gone the SPI slave has nothing left armed, so every further
             * NRF transceive -- including the one carrying the actual
             * STOP-ack -- just stalls with nothing to match against. That
             * left this loop polling forever even though the NRF had
             * already logged processing the stop. */
            if (!SPI_BUS_LOCK(portMAX_DELAY)) {
                ESP_LOGE(BIOGAP_READ_TAG, "Failed to lock SPI bus mutex for stop-quiesce re-queue");
            } else {
                esp_err_t requeue_ret = spi_slave_queue_trans(SPI_HOST_DEVICE, ret_trans, 0);
                SPI_BUS_UNLOCK();
                if (requeue_ret != ESP_OK) {
                    ESP_LOGE(BIOGAP_READ_TAG, "Failed to re-queue transaction during stop-quiesce: %s", esp_err_to_name(requeue_ret));
                }
            }
        }
    }

    ESP_LOGI(BIOGAP_READ_TAG, "STOP command delivery confirmed after %d attempts", send_stop_command_count);
    node_state = STATE_IDLE;
    prequeued = false;
    xEventGroupSetBits(g_evt, B_STOP_CMD_FWD_TO_BIOGAP);
    reset_spi_bus_for_restart();
    prepare_for_restart();
    return ESP_OK;
}

// =============================================================================
// MAIN TASK: High-speed SPI slave with pre-queue pattern
// =============================================================================
/**
 * @brief Main SPI slave task using pre-queue pattern for high-speed streaming.
 */
void read_from_biogap_task_nrf_master_esp_slave_prequeue(void *pv)
{

    uint64_t packet_count = 0;
    uint16_t tx_counter = 0;        /* Transmit counter sent to master */
    uint16_t rx_counter_prev = 0;   /* Previous receive counter for detecting missed packets */
    esp_err_t ret;
    read_from_biogap_task_nrf_master_pq_esp_slave_handle = xTaskGetCurrentTaskHandle();
    bool first_read = true;
    /* === MAIN LOOP === */
    while (1) {

        /* === HANDSHAKE PHASE === */
        /* Handshake is executed at startup in main; block here until the connected bit is set */
        xEventGroupWaitBits(g_evt, B_BIOGAP_CONECTED, pdFALSE, pdFALSE, portMAX_DELAY);

        if (!prequeued) {
            /* propagate_first_start_command_to_biogap_master() (biogap_send.c)
             * relays the GUI's whole config+start sequence as one blocking
             * spi_slave_transmit() call, then -- once that's confirmed
             * delivered -- allocates the bulk buffers and sets this bit.
             * Waiting for it here means the bulk pool is never armed, and
             * spi_slave_get_trans_result() is never called, while that
             * transmit is still in flight: mixing spi_slave_transmit() with
             * an ongoing queue on the same host is unsafe per the ESP-IDF
             * SPI slave driver docs, and this avoids it by construction
             * rather than by coordinating two tasks on the same queue. */
            xEventGroupWaitBits(g_evt, B_START_CMD_FWD_TO_BIOGAP, pdTRUE, pdFALSE, portMAX_DELAY);

            ret = prequeue_transactions();
            while (ret != ESP_OK) {
                ESP_LOGE(BIOGAP_READ_TAG, "Failed to pre-queue transactions, retrying: %s", esp_err_to_name(ret));
                vTaskDelay(pdMS_TO_TICKS(1000));
                ret = prequeue_transactions();
            }
            prequeued = true;
        }

        /* === STREAMING PHASE (main high-speed loop) === */
        if (current_spi_mode != SPI_MODE_NRF) {
            ESP_LOGW(BIOGAP_READ_TAG, "Not in NRF SPI mode, skipping");
            vTaskDelay(pdMS_TO_TICKS(100));
            continue;
        }

        /* First, check if STOP command has arrived from either the GUI or the overflow/backpressure path. */
        if (xEventGroupGetBits(g_evt) & (B_STOP_CMD_RCV_GUI | B_STOP_CMD_RCV_FORCED)) {

            if(node_state != STATE_IDLE){
                ret = enter_stop_quiesce_state();
                ESP_LOGI(BIOGAP_READ_TAG, "STOP command processed, returning to idle state");
                first_read = true;
                continue;
            }
            continue; // jump to the top of the loop
        }
        else{
            /* By this point the bulk pool is always armed (see the
             * B_START_CMD_FWD_TO_BIOGAP wait above) -- config-relay is
             * handled entirely by biogap_send.c via a blocking
             * spi_slave_transmit(), before this task ever calls
             * spi_slave_get_trans_result(). */
            {
                spi_slave_transaction_t *ret_trans = NULL;
                if (!SPI_BUS_LOCK(portMAX_DELAY)) {
                    ESP_LOGE(BIOGAP_READ_TAG, "Failed to lock SPI bus mutex for transaction receive");
                    break;
                }
                ret = spi_slave_get_trans_result(SPI_HOST_DEVICE, &ret_trans, pdMS_TO_TICKS(50));
                SPI_BUS_UNLOCK();
                if (ret == ESP_ERR_TIMEOUT) {
                    continue;
                }
                if (ret != ESP_OK) {
                    ESP_LOGE(BIOGAP_READ_TAG, "Failed to get SPI result: %s", esp_err_to_name(ret));
                    break;
                }

                size_t rx_bytes = (size_t)(ret_trans->trans_len / 8);

                if(rx_bytes < 4){
                    continue; // ignore empty packets
                }
                uint8_t *rx_data = (uint8_t *)ret_trans->rx_buffer;
                uint8_t *tx_data = (uint8_t *)ret_trans->tx_buffer;

                if (node_state != STATE_STREAMING) {
                    /* Not streaming yet -- drop and re-arm rather than
                     * misinterpreting stray bytes as data. */
                    ESP_LOGW(BIOGAP_READ_TAG, "Unexpected transaction while idle: hdr=0x%02X, dropping", rx_data[0]);
                    if (!SPI_BUS_LOCK(portMAX_DELAY)) {
                        ESP_LOGE(BIOGAP_READ_TAG, "Failed to lock SPI bus mutex for re-queue while idle");
                        break;
                    }
                    ret = spi_slave_queue_trans(SPI_HOST_DEVICE, ret_trans, 0);
                    SPI_BUS_UNLOCK();
                    if (ret != ESP_OK) {
                        ESP_LOGE(BIOGAP_READ_TAG, "Failed to re-queue transaction while idle: %s", esp_err_to_name(ret));
                        break;
                    }
                    continue;
                }

                /* === NRF DATA PROCESSING === (modality-agnostic: EXG,
                 * ultrasound/WULPUS, or whatever else shares this pool --
                 * the counter check and add_to_ringbuffer() below don't
                 * care which, they key off the header byte itself.)
                 *
                 * Validate packet markers (header + tailer) on what the NRF actually
                 * sent. This is the only meaningful integrity check available: the
                 * ESP's own echoed tx buffer has ESP_EXG_TAILER pre-filled at every
                 * position from byte[3] onward (allocate_prequeue_resources()) so
                 * that whatever send_len the NRF uses, its own header/tailer check
                 * in biogap_to_esp_transaction() passes -- which also means a tx-side
                 * check here would trivially pass regardless of transfer integrity. */
                if (!is_valid_packet(rx_data, rx_bytes)) {
                    /* Not necessarily corruption: the NRF sends a dedicated
                     * STOP-ACK transceive (header/tailer 0xE6/0xBB) whenever
                     * it thinks the ESP asked to stop, which can be a false
                     * positive -- ESP_STOP_COMMAND (245) collides with the
                     * regular streaming reply's counter MSB once every
                     * 65536-count wrap. Drop this one transaction and keep
                     * the slot armed rather than killing the whole task. */
                    ESP_LOGW(BIOGAP_READ_TAG, "Invalid packet: hdr=0x%02X tail=0x%02X (expected 0x%02X/0x%02X) -- dropping, re-arming slot",
                            rx_data[0], rx_data[rx_bytes - 1], NRF_EXG_HEADER, NRF_EXG_TAILER);
                    if (!SPI_BUS_LOCK(portMAX_DELAY)) {
                        ESP_LOGE(BIOGAP_READ_TAG, "Failed to lock SPI bus mutex for re-queue after invalid packet");
                        break;
                    }
                    ret = spi_slave_queue_trans(SPI_HOST_DEVICE, ret_trans, 0);
                    SPI_BUS_UNLOCK();
                    if (ret != ESP_OK) {
                        ESP_LOGE(BIOGAP_READ_TAG, "Failed to re-queue transaction after invalid packet: %s", esp_err_to_name(ret));
                        break;
                    }
                    continue;
                }

                /* Extract counter (little-endian at bytes[1:2]) */
                /* Note: this will break with the WULPUS logic*/

                if(rx_data[0] == WULPUS_HDR_XFER_0 || rx_data[0] == NRF_EXG_HEADER){
                    uint16_t counter = (rx_data[2] << 8) | rx_data[1];
                    if(counter != (rx_counter_prev+1)){
                        ESP_LOGW(BIOGAP_READ_TAG, "Missed packet(s): expected counter %d, got %d", rx_counter_prev+1, counter);
                    }
                    rx_counter_prev = counter;
                    packet_count++;
                }

                /* Add packet to ringbuffer */
                if(first_read == true){
                    ESP_LOGI(BIOGAP_READ_TAG, "First transaction done");
                    first_read = false;
                }
                ret = add_to_ringbuffer(rx_data, rx_bytes);
                if (ret != ESP_OK) {
                    ESP_LOGW(BIOGAP_READ_TAG, "Failed to add packet to ringbuffer");
                    xEventGroupSetBits(g_evt, B_RINGBUFFER_FULL);
                    // check if the stop command has not been set already to avoid setting it multiple times
                    if(xEventGroupGetBits(g_evt) & B_STOP_CMD_RCV_GUI){
                        ESP_LOGI(BIOGAP_READ_TAG, "STOP command already received, not forwarding another STOP to BIOGAP master");
                    }
                    else{
                        ESP_LOGI(BIOGAP_READ_TAG,"Setting Stop CMD from rinfugg");
                        rx_gui_data_to_fwd[0] = STOP_DUMMY_STREAMING;
                        xEventGroupSetBits(g_evt, B_STOP_CMD_RCV_FORCED);
                    }
                    continue; // jump to next iteration so the STOP check can run immediately
                }
                /* Update the descriptor's TX buffer for its next transaction */
                tx_counter++;  /* Advance transmit counter */
                /* The NRF treats byte[2] == ESP_STOP_COMMAND (245) as a real
                 * stop request (biogap_to_esp_transaction()), but byte[2] is
                 * just this counter's high byte -- it would otherwise pass
                 * through 245 once every 65536-count wrap with no stop
                 * actually requested. Skip the whole colliding block so that
                 * never happens. */
                if (((tx_counter >> 8) & 0xFF) == ESP_STOP_COMMAND) {
                    tx_counter += 256;
                }
                tx_data[0] = ESP_EXG_HEADER;
                //tx_data[PACKET_SZ - 1] = ESP_EXG_TAILER;
                tx_data[1] = tx_counter & 0xFF;  /* LSB of counter */
                tx_data[2] = (tx_counter >> 8) & 0xFF;  /* MSB of counter */

                if (!SPI_BUS_LOCK(portMAX_DELAY)) {
                    ESP_LOGE(BIOGAP_READ_TAG, "Failed to lock SPI bus mutex for re-queue");
                    break;
                }
                // make the descriptor available for the next transaction by re-queuing it immediately
                ret = spi_slave_queue_trans(SPI_HOST_DEVICE, ret_trans, 0);
                SPI_BUS_UNLOCK();
                if (ret != ESP_OK) {
                    ESP_LOGE(BIOGAP_READ_TAG, "Failed to re-queue transaction: %s", esp_err_to_name(ret));
                    break;
                }  
            }  
        }
    }

    /* === CLEANUP === */
    ESP_LOGI(BIOGAP_READ_TAG, "Freeing pre-queued DMA buffers and exiting task");
    free_prequeue_resources();
    vTaskDelete(NULL);
}
