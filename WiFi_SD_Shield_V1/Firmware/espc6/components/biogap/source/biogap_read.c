/**
 * @file biogap_read_highspeed_prequeue.c
 * @brief High-speed SPI slave implementation using pre-queue pattern
 * 
 * BACKGROUND:
 * The original implementation uses sequential queue+get pattern:
 *   1. spi_slave_queue_trans(desc)
 *   2. spi_slave_get_trans_result(desc)
 *   3. repeat
 * 
 * At high transaction rates (low inter-packet delay), the master sends back-to-back
 * packets faster than the slave can arm its receive descriptors. During the gap between
 * step 2 and step 1 in the next iteration, the slave is UNARMED and loses data.
 * 
 * SOLUTION:
 * Use pre-queue pattern with N buffers (QUEUE_COUNT=4) to keep slave always armed:
 *   1. Pre-allocate QUEUE_COUNT RX buffers with MALLOC_CAP_DMA
 *   2. Pre-queue all N transactions before main loop
 *   3. In main loop: get_result() → validate → re-queue (keeps queue full)
 *   4. Slave is ALWAYS armed; no gaps between transactions
 * 
 * EXPECTED RESULT:
 * Continuous high-speed packet streaming without data corruption or gaps.
 */

#include "biogap.h"
#include "common.h"
#include "led_app.h"

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

#define DRDY_WAIT_TIMEOUT_MS 100
#define DRDY_LED_BLINK_INTERVAL_MS 150
#define DRDY_LOG_INTERVAL_MS 500
#define HANDSHAKE_RETRY_DELAY_MS 20
#define HANDSHAKE_MAX_RETRIES 50


#define PACKET_SZ NRF_EXG_PACKET_SIZE

/* Ensure per-transaction packet size never exceeds configured SPI bus transfer cap */
_Static_assert(PACKET_SZ <= SPI_FROM_BIOGAP_MAX_SIZE,
               "NRF_EXG_PACKET_SIZE exceeds SPI_FROM_BIOGAP_MAX_SIZE");

// =============================================================================
// GLOBALS FOR PRE-QUEUE PATTERN
// =============================================================================

/* Persistent DMA buffers and descriptors (pre-allocated once at task start) */
static uint8_t *rx_bufs[QUEUE_COUNT] = {NULL};           /* RX buffers for each pre-queued desc */
static uint8_t *tx_bufs[QUEUE_COUNT] = {NULL};           /* TX buffers for each pre-queued desc */
static spi_slave_transaction_t trans_descs[QUEUE_COUNT]; /* Transaction descriptors */
uint8_t *sendbuf_persistent = NULL;               /* Single TX buffer for all desciptors */
static bool prequeued = false;

bool handshake_pq_done = false;





// =============================================================================
// HELPER: Pre-queue all RX transactions
// =============================================================================
/**
 * @brief Pre-queue QUEUE_COUNT transactions so slave remains armed at all times.
 * 
 * This function must be called once after handshake succeeds, before the main
 * streaming loop. It initializes transaction descriptors and queues them all,
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
    //ESP_LOGI(BIOGAP_READ_TAG, "Pre-queued %d SPI slave transactions (slave now ALWAYS armed)", QUEUE_COUNT);
    return ESP_OK;
}

// =============================================================================
// HELPER: Add packet to ringbuffer
// =============================================================================
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
static inline bool is_valid_packet(const uint8_t *data)
{
    return (data[0] == NRF_EXG_HEADER && data[PACKET_SZ - 1] == NRF_EXG_TAILER);
}
static inline bool is_valid_tx_packet(const uint8_t *data)
{
    return (data[0] == ESP_EXG_HEADER && data[PACKET_SZ - 1] == ESP_EXG_TAILER);
}

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

static esp_err_t enter_stop_quiesce_state(void)
{
    ESP_LOGI(BIOGAP_READ_TAG, "Stop requested, idling until next START (sender will drain)");

    /* Overwrite the shared streaming TX buffer so the active pre-queued
     * descriptor returns the STOP command frame during quiesce.
     */
    if (sendbuf_persistent) {
        memset(sendbuf_persistent, 0, PACKET_SZ);
        sendbuf_persistent[0] = ESP_EXG_HEADER;
        sendbuf_persistent[2] = ESP_STOP_COMMAND;
        sendbuf_persistent[3] = rx_gui_data_to_fwd[0];
        sendbuf_persistent[PACKET_SZ - 1] = ESP_EXG_TAILER;

        for (int i = 0; i < QUEUE_COUNT; i++) {
            if (tx_bufs[i]) {
                memcpy(tx_bufs[i], sendbuf_persistent, PACKET_SZ);
            }
        }
    }

    spi_slave_transaction_t *ret_trans;
    esp_err_t ret;
    uint8_t drained = 0;

    if (!SPI_BUS_LOCK(portMAX_DELAY)) {
        ESP_LOGE(BIOGAP_READ_TAG, "Failed to lock SPI bus mutex during STOP quiesce");
        return ESP_FAIL;
    }

    ret = spi_slave_get_trans_result(SPI_HOST_DEVICE, &ret_trans, pdMS_TO_TICKS(50));
    SPI_BUS_UNLOCK();
    if (ret != ESP_OK) {
        ESP_LOGI(BIOGAP_READ_TAG, "Failed to drain in-flight transaction during STOP quiesce");
    } else {
        uint8_t *tx_data = (uint8_t *)ret_trans->tx_buffer;
        ESP_LOGI(BIOGAP_READ_TAG, "Sent packet: [0]=0x%02X [1]=0x%02X [2] 0x%02X [3]=0x%02X)",
                 tx_data[0], tx_data[1], tx_data[2], tx_data[3]);
    }

        xEventGroupSetBits(g_evt, B_SPI_QUIESCED);
        node_state = STATE_IDLE;
        prequeued = false;
        return ESP_OK;
}

// =============================================================================
// MAIN TASK: High-speed SPI slave with pre-queue pattern
// =============================================================================
/**
 * @brief Main SPI slave task using pre-queue pattern for high-speed streaming.
 * 
 * FLOW:
 *   1. Handshake with NRF master (exchange 0x5A ↔ 0xA5)
 *   2. Allocate QUEUE_COUNT RX buffers (DMA-capable)
 *   3. Pre-queue all transactions (slave armed QUEUE_COUNT times)
 *   4. Main loop: get_result() → validate → re-queue (maintains constant queue depth)
 * 
 * KEY ADVANTAGE:
 *   - No gap between consecutive transactions
 *   - Slave ALWAYS has at least one descriptor armed
 *   - Master never finds slave unarmed; no packet loss
 * 
 * @param pv Task parameter (unused)
 */
void read_from_biogap_task_nrf_master_esp_slave_prequeue(void *pv)
{

    uint64_t packet_count = 0;
    uint16_t tx_counter = 0;  /* Transmit counter sent to master */
    int64_t last_log_us = 0;
    esp_err_t ret;
    ret = allocate_prequeue_resources();
    read_from_biogap_task_nrf_master_pq_esp_slave_handle = xTaskGetCurrentTaskHandle();
    bool first_read = true;
    /* === MAIN LOOP === */
    while (1) {
        /* Wait for SD card writes to complete */
        if ((xEventGroupGetBits(g_evt) & B_WRITING_TO_SD) != 0) {
            vTaskDelay(pdMS_TO_TICKS(10));
            continue;
        }
        /* === HANDSHAKE PHASE === */
        /* Handshake is executed at startup in main; block here until the connected bit is set */
        xEventGroupWaitBits(g_evt, B_BIOGAP_CONECTED, pdFALSE, pdFALSE, portMAX_DELAY);
        /* Wait until a start command has been forwarded to BIOGAP (sleep until bit set) */
        xEventGroupWaitBits(g_evt, B_START_CMD_FWD_TO_BIOGAP, pdFALSE, pdFALSE, portMAX_DELAY);

        if(!prequeued){
            ret = prequeue_transactions();
            if (ret != ESP_OK) {
                ESP_LOGE(BIOGAP_READ_TAG, "Failed to pre-queue transactions, cannot start streaming");
                vTaskDelay(pdMS_TO_TICKS(1000));
                continue;
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
            //ESP_LOGI(BIOGAP_READ_TAG, "STOP command received, entering quiesce state before processing more transactions");
            if(node_state != STATE_IDLE){
                ret = enter_stop_quiesce_state();
                // break to give priority to the STOP command processing before we check for new transactions again
                xEventGroupClearBits(g_evt, B_START_CMD_FWD_TO_BIOGAP); 
                first_read = true;   
                continue;
                  
            }
            //enter_stop_quiesce_state();
            /* Block here until a START has been forwarded to BIOGAP */
            //xEventGroupWaitBits(g_evt, B_START_CMD_FWD_TO_BIOGAP, pdFALSE, pdFALSE, portMAX_DELAY);
            //ESP_LOGI(BIOGAP_READ_TAG, "START command forwarded to BIOGAP after quiesc, resuming streaming");
            //xEventGroupClearBits(g_evt, B_SPI_QUIESCED);
            continue; // jump to the top of the loop
        }
        else{

            if (node_state == STATE_STREAMING){
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
                if (rx_bytes != PACKET_SZ) {
                    ESP_LOGW(BIOGAP_READ_TAG,
                            "SPI length mismatch: expected=%uB got=%uB ret_trans=%p tx_buf=%p tx=[%02X %02X %02X %02X]",
                            (unsigned)PACKET_SZ,
                            (unsigned)rx_bytes,
                            (void *)ret_trans,
                            (void *)ret_trans->tx_buffer,
                            ((uint8_t *)ret_trans->tx_buffer)[0],
                            ((uint8_t *)ret_trans->tx_buffer)[1],
                            ((uint8_t *)ret_trans->tx_buffer)[2],
                            ((uint8_t *)ret_trans->tx_buffer)[PACKET_SZ - 1]);
                }

                uint8_t *rx_data = (uint8_t *)ret_trans->rx_buffer;
                uint8_t *tx_data = (uint8_t *)ret_trans->tx_buffer;
                /* Validate packet markers (header + tailer) */
                if (!is_valid_packet(rx_data)) {
                    ESP_LOGW(BIOGAP_READ_TAG, "Invalid packet: hdr=0x%02X tail=0x%02X (expected 0x%02X/0x%02X)",
                            rx_data[0], rx_data[PACKET_SZ - 1], NRF_EXG_HEADER, NRF_EXG_TAILER);
                    break;
                }
                if (!is_valid_tx_packet(tx_data)) {
                    ESP_LOGW(BIOGAP_READ_TAG,
                        "Invalid packet: ret_trans=%p tx_buf=%p hdr=0x%02X tail=0x%02X (expected 0x%02X/0x%02X)",
                        (void *)ret_trans,
                        (void *)tx_data,
                        tx_data[0],
                        tx_data[PACKET_SZ - 1],
                        ESP_EXG_HEADER,
                        ESP_EXG_TAILER);
                    break;
                }

                /* Extract counter (little-endian at bytes[1:2]) */
                uint16_t counter = (rx_data[2] << 8) | rx_data[1];
                packet_count++;

                /* Add packet to ringbuffer */
                if(first_read == true){
                    ESP_LOGI(BIOGAP_READ_TAG, "First transaction done");
                    first_read = false;
                }
                ret = add_to_ringbuffer(rx_data, PACKET_SZ);
                if (ret != ESP_OK) {
                    ESP_LOGW(BIOGAP_READ_TAG, "Failed to add packet to ringbuffer");
                    xEventGroupSetBits(g_evt, B_RINGBUFFER_FULL);
                    // check if thr stop command has not been set already to avoid setting it multiple times
                    if(xEventGroupGetBits(g_evt) & B_STOP_CMD_RCV_GUI){
                        ESP_LOGI(BIOGAP_READ_TAG, "STOP command already received, not forwarding another STOP to BIOGAP master");
                    }
                    else{
                        ESP_LOGI(BIOGAP_READ_TAG,"Setting Stop CMD from rinfugg"); 
                        xEventGroupSetBits(g_evt, B_STOP_CMD_RCV_FORCED);
                    }
                    continue; // jump to next iteration so the STOP check can run immediately
                }
                /* Update this descriptor's TX buffer for its next transaction */
                tx_counter++;  /* Advance transmit counter */
                tx_data[0] = ESP_EXG_HEADER;
                tx_data[PACKET_SZ - 1] = ESP_EXG_TAILER;
                tx_data[1] = tx_counter & 0xFF;  /* LSB of counter */
                tx_data[2] = (tx_counter >> 8) & 0xFF;  /* MSB of counter */

                /* === KEY PATTERN: RE-QUEUE IMMEDIATELY ===
                * This is the core of the high-speed pattern:
                * Return the descriptor to the queue IMMEDIATELY after processing.
                * This keeps the slave armed continuously with no gaps.
                */
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
