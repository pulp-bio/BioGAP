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

/* STOP is delivered to the NRF only by piggybacking on the response of the
 * NEXT regular data transaction the NRF (master) happens to initiate on its
 * own sampling cadence -- there is no dedicated STOP transfer. That cadence
 * can be as slow as ~400ms (dummy sensor: 100ms sample period x 4 samples per
 * packet), so this must poll/retry for long enough to actually observe that
 * delivery before the caller tears down and reinitializes the SPI slave bus
 * (prepare_for_restart()) -- otherwise the bus gets reset before the NRF ever
 * saw the STOP marker, and the NRF is left stuck trying to stream to a bus
 * that no longer has anything queued for it. */
#define STOP_DRAIN_TIMEOUT_MS 1500
#define STOP_DRAIN_POLL_MS 100


#define PACKET_SZ NRF_EXG_PACKET_SIZE

/* Ensure per-transaction packet size never exceeds configured SPI bus transfer cap */
_Static_assert(PACKET_SZ <= SPI_FROM_BIOGAP_MAX_SIZE,
               "NRF_EXG_PACKET_SIZE exceeds SPI_FROM_BIOGAP_MAX_SIZE");

// =============================================================================
// GLOBALS FOR PRE-QUEUE PATTERN
// =============================================================================

/* Persistent DMA buffers and descriptors (pre-allocated once at task start) */
// static uint8_t *rx_bufs[QUEUE_COUNT] = {NULL};           /* RX buffers for each pre-queued desc */
// static uint8_t *tx_bufs[QUEUE_COUNT] = {NULL};           /* TX buffers for each pre-queued desc */
// static spi_slave_transaction_t trans_descs[QUEUE_COUNT]; /* Transaction descriptors */
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

    spi_slave_transaction_t *ret_trans;
    esp_err_t ret;

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


    bool stop_delivered = false;
    // int64_t deadline_us = esp_timer_get_time() + (int64_t)STOP_DRAIN_TIMEOUT_MS * 1000;
    
    // send stop command multiple times
    uint8_t send_stop_command_count = 0;
    bool stop_ack_received = false;
    //while(send_stop_command_count < 3 && !stop_delivered) {
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
                // Transition the ESP-side communication state here.
            } 
            else {
                send_stop_command_count++;
            }
        }
        }
        //SPI_BUS_UNLOCK(); removed

    ESP_LOGI(BIOGAP_READ_TAG, "STOP command delivery confirmed after %d attempts", send_stop_command_count);
    //EventGroupSetBits(g_evt, B_SPI_QUIESCED);
    node_state = STATE_IDLE;
    prequeued = false;
    xEventGroupSetBits(g_evt, B_STOP_CMD_FWD_TO_BIOGAP);
    reset_spi_bus_for_restart();
    prepare_for_restart();
    return ESP_OK;
}

/**
 * @brief Re-arm QUEUE_COUNT idle-content transactions right after the SPI
 * slave bus is torn down and reinitialized for the next session.
 *
 * prepare_for_restart() (gui_task.c) frees/reinitializes the SPI slave bus
 * after a STOP, but previously left it with nothing queued until the NEXT
 * START's prequeue_transactions() call -- an unarmed slave for however long
 * the system then sits idle. If the NRF ever clocks a transaction during
 * that window (e.g. STOP delivery confirmation timed out and it never got
 * the memo, or any other stray attempt), it reads back 0xFF/0xFF garbage
 * instead of a valid header/tailer, which trips its own halt-on-bad-frame
 * check (wifi_sd_spi_functions.c). Call this immediately after
 * allocate_prequeue_resources() so the bus is never left unarmed.
 */
esp_err_t rearm_idle_prequeue(void)
{
    esp_err_t ret = prequeue_transactions();
    if (ret == ESP_OK) {
        prequeued = true;
    } else {
        ESP_LOGE(BIOGAP_READ_TAG, "Failed to re-arm idle pre-queue: %s", esp_err_to_name(ret));
    }
    return ret;
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
        
        /* === HANDSHAKE PHASE === */
        /* Handshake is executed at startup in main; block here until the connected bit is set */
        xEventGroupWaitBits(g_evt, B_BIOGAP_CONECTED, pdFALSE, pdFALSE, portMAX_DELAY);
        /* Wait until a start command has been forwarded to BIOGAP (sleep until bit set) */
        xEventGroupWaitBits(g_evt, B_START_CMD_FWD_TO_BIOGAP, pdFALSE, pdFALSE, portMAX_DELAY);
        //ESP_LOGI(BIOGAP_READ_TAG, "B_START_CMD_FWD_TO_BIOGAP received");
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

            if(node_state != STATE_IDLE){
                ret = enter_stop_quiesce_state();
                xEventGroupClearBits(g_evt, B_START_CMD_FWD_TO_BIOGAP); 
                ESP_LOGI(BIOGAP_READ_TAG, "STOP command processed, returning to idle state"); 
                first_read = true; 
                continue;
                  
            }
            continue; // jump to the top of the loop
        }
        else{

            if (node_state == STATE_STREAMING){
                if(first_read == true){
                    ESP_LOGI(BIOGAP_READ_TAG, "First transaction in progress");
                }
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
                        /* Piggybacked STOP frame carries rx_gui_data_to_fwd[0] as its
                         * opcode byte (see enter_stop_quiesce_state()); without setting
                         * it here it would still hold whatever command was last
                         * forwarded (typically the session's START opcode), so the NRF
                         * would dispatch the wrong command. */
                        rx_gui_data_to_fwd[0] = STOP_DUMMY_STREAMING;
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
