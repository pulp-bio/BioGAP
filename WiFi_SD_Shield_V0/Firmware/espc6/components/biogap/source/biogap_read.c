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

#define BIOGAP_READ_TAG "biogap_read_hs"

#define DRDY_WAIT_TIMEOUT_MS 100
#define DRDY_LED_BLINK_INTERVAL_MS 150
#define DRDY_LOG_INTERVAL_MS 500
#define HANDSHAKE_RETRY_DELAY_MS 20
#define HANDSHAKE_MAX_RETRIES 50

/* Pre-queue configuration: keep this many transactions armed at all times */
#define QUEUE_COUNT 4
#define PACKET_SZ NRF_EXG_PACKET_SIZE

/* Ensure per-transaction packet size never exceeds configured SPI bus transfer cap */
_Static_assert(PACKET_SZ <= SPI_FROM_BIOGAP_MAX_SIZE,
               "NRF_EXG_PACKET_SIZE exceeds SPI_FROM_BIOGAP_MAX_SIZE");

// =============================================================================
// GLOBALS FOR PRE-QUEUE PATTERN
// =============================================================================

/* Persistent DMA buffers and descriptors (pre-allocated once at task start) */
static uint8_t *rx_bufs[QUEUE_COUNT] = {NULL};           /* RX buffers for each pre-queued desc */
static spi_slave_transaction_t trans_descs[QUEUE_COUNT]; /* Transaction descriptors */
static uint8_t *sendbuf_persistent = NULL;               /* Single TX buffer for all desciptors */

bool handshake_pq_done = false;

/* DMA-capable handshake buffers */
static uint8_t handshake_pq_tx_buffer[4] __attribute__((aligned(4)));
static uint8_t handshake_pq_rx_buffer[4] __attribute__((aligned(4)));
static uint8_t expected_pq_handshake_buffer[4] __attribute__((aligned(4)));

// // =============================================================================
// // ISR: NRF data ready interrupt handler
// // =============================================================================
// void IRAM_ATTR nrf_data_ready_isr(void *arg)
// {
//     BaseType_t hp_woken = pdFALSE;

//     if (read_from_biogap_task_nrf_master_pq_esp_slave_handle != NULL) {
//         vTaskNotifyGiveFromISR(read_from_biogap_task_nrf_master_pq_esp_slave_handle, &hp_woken);
//     }

//     if (hp_woken == pdTRUE) {
//         portYIELD_FROM_ISR();
//     }
// }

// =============================================================================
// HELPER: Initial handshake
// =============================================================================

void initial_handshake_nrf_master_esp_slave_pq(void)
{
    ESP_LOGW(BIOGAP_READ_TAG, ">>> HANDSHAKE: Waiting for NRF master to pull CS and clock 4 bytes (marker=0x%02X)", HANDSHAKE_MARKER);
    
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

    ESP_LOGW(BIOGAP_READ_TAG, ">>> TX buffer addr=%p, contents=[0x%02X 0x%02X 0x%02X 0x%02X]", 
            handshake_pq_tx_buffer, handshake_pq_tx_buffer[0], handshake_pq_tx_buffer[1], 
            handshake_pq_tx_buffer[2], handshake_pq_tx_buffer[3]);
    ESP_LOGW(BIOGAP_READ_TAG, ">>> RX buffer addr=%p", handshake_pq_rx_buffer);
    ESP_LOGW(BIOGAP_READ_TAG, ">>> Calling spi_slave_transmit() - will block until CS pulled low");
    
    esp_err_t ret = spi_slave_transmit(SPI_HOST_DEVICE, &t, portMAX_DELAY);
    ESP_LOGW(BIOGAP_READ_TAG, ">>> HANDSHAKE: spi_slave_transmit returned: %s", esp_err_to_name(ret));
    
    if (ret == ESP_OK) {
        ESP_LOGI(BIOGAP_READ_TAG, "SPI xact OK: sent [0x%02X 0x%02X 0x%02X 0x%02X], received [0x%02X 0x%02X 0x%02X 0x%02X]", 
                handshake_pq_tx_buffer[0], handshake_pq_tx_buffer[1], handshake_pq_tx_buffer[2], handshake_pq_tx_buffer[3],
                handshake_pq_rx_buffer[0], handshake_pq_rx_buffer[1], handshake_pq_rx_buffer[2], handshake_pq_rx_buffer[3]);
    } else {
        ESP_LOGE(BIOGAP_READ_TAG, "SPI xact failed: %s", esp_err_to_name(ret));
    }
    
    /* Validate handshake response */
    if (memcmp(handshake_pq_rx_buffer, expected_pq_handshake_buffer, 4) == 0) {
        ESP_LOGI(BIOGAP_READ_TAG, "Handshake successful: received expected response");
        handshake_pq_done = true;
    } else {
        ESP_LOGW(BIOGAP_READ_TAG, "Handshake warning: received response does not match expected marker");
    }
}

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
    for (int i = 0; i < QUEUE_COUNT; i++) {
        memset(&trans_descs[i], 0, sizeof(trans_descs[i]));
        trans_descs[i].length = PACKET_SZ * 8;  /* Convert bytes to bits */
        trans_descs[i].tx_buffer = sendbuf_persistent;
        trans_descs[i].rx_buffer = rx_bufs[i];

        esp_err_t ret = spi_slave_queue_trans(SPI_HOST_DEVICE, &trans_descs[i], 0);
        if (ret != ESP_OK) {
            ESP_LOGE(BIOGAP_READ_TAG, "Failed to pre-queue transaction %d: %s", i, esp_err_to_name(ret));
            return ret;
        }
    }
    //ESP_LOGI(BIOGAP_READ_TAG, "Pre-queued %d SPI slave transactions (slave now ALWAYS armed)", QUEUE_COUNT);
    return ESP_OK;
}

// =============================================================================
// HELPER: Add packet to ringbuffer
// =============================================================================
esp_err_t add_to_ringbuffer(const uint8_t *data, size_t len)
{
    void *rb_ptr = NULL;
    if (xRingbufferSendAcquire(ringbuff, &rb_ptr, len, 0) != pdTRUE || rb_ptr == NULL) {
        ESP_LOGE(BIOGAP_READ_TAG, "Failed to acquire ringbuffer space");
        return ESP_FAIL;
    }
    memcpy(rb_ptr, data, len);
    xRingbufferSendComplete(ringbuff, rb_ptr);
    return ESP_OK;
}

// =============================================================================
// HELPER: Validate packet markers
// =============================================================================
static inline bool is_valid_packet(const uint8_t *data)
{
    return (data[0] == NRF_EXG_HEADER && data[PACKET_SZ - 1] == NRF_EXG_TAILER);
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
    ESP_LOGI(BIOGAP_READ_TAG, "Starting HIGH-SPEED SPI slave task (pre-queue pattern)");
    vTaskDelay(pdMS_TO_TICKS(500));
    ESP_LOGW(BIOGAP_READ_TAG, ">>> Startup delay complete, ready for master");
    
    uint64_t packet_count = 0;
    uint16_t tx_counter = 0;  /* Transmit counter sent to master */
    int64_t last_log_us = 0;

    /* Allocate persistent TX buffer (same for all transactions) */
    sendbuf_persistent = heap_caps_malloc(PACKET_SZ, MALLOC_CAP_DMA);
    if (!sendbuf_persistent) {
        ESP_LOGE(BIOGAP_READ_TAG, "Failed to allocate sendbuf");
        vTaskDelete(NULL);
        return;
    }
    memset(sendbuf_persistent, 0, PACKET_SZ);
    /* Initialize TX buffer with header/tailer and initial counter */
    sendbuf_persistent[0] = ESP_EXG_HEADER;
    sendbuf_persistent[PACKET_SZ - 1] = ESP_EXG_TAILER;
    sendbuf_persistent[1] = 0;  /* Initial counter LSB */
    sendbuf_persistent[2] = 0;  /* Initial counter MSB */
    ESP_LOGI(BIOGAP_READ_TAG, "Allocated TX buffer: %p", sendbuf_persistent);

    /* Allocate QUEUE_COUNT RX buffers for pre-queuing pattern */
    for (int i = 0; i < QUEUE_COUNT; i++) {
        rx_bufs[i] = heap_caps_malloc(PACKET_SZ, MALLOC_CAP_DMA);
        if (!rx_bufs[i]) {
            ESP_LOGE(BIOGAP_READ_TAG, "Failed to allocate RX buffer %d", i);
            vTaskDelete(NULL);
            return;
        }
        memset(rx_bufs[i], 0, PACKET_SZ);
    }
    ESP_LOGI(BIOGAP_READ_TAG, "Allocated %d RX buffers (DMA-capable, total %.1f KB)", 
             QUEUE_COUNT, (float)(QUEUE_COUNT * PACKET_SZ) / 1024.0);

    read_from_biogap_task_nrf_master_pq_esp_slave_handle = xTaskGetCurrentTaskHandle();

    /* === MAIN LOOP === */
    while (1) {
        /* Check for stop signal */
        if (xEventGroupGetBits(g_evt) & B_END_ACQUISITION) {
            ESP_LOGI(BIOGAP_READ_TAG, "Stop requested, exiting");
            break;
        }

        /* Wait for SD card writes to complete */
        if ((xEventGroupGetBits(g_evt) & B_WRITING_TO_SD) != 0) {
            vTaskDelay(pdMS_TO_TICKS(10));
            continue;
        }

        /* === HANDSHAKE PHASE === */
        if (!handshake_pq_done) {
            initial_handshake_nrf_master_esp_slave_pq();
            continue;
        }

        if(!send_start_command_to_biogap_master){
            // Wait to send start command to NRF master to set-up the device 
            vTaskDelay(pdMS_TO_TICKS(1));
            continue;
        }

        /* === PRE-QUEUE PHASE (run once after handshake) === */
        static bool prequeued = false;
        if (!prequeued) {
            if (prequeue_transactions() != ESP_OK) {
                ESP_LOGE(BIOGAP_READ_TAG, "Failed to pre-queue transactions, breaking");
                break;
            }
            prequeued = true;
            continue;  /* Wait for first transaction to complete */
        }

        /* === STREAMING PHASE (main high-speed loop) === */
        if (current_spi_mode != SPI_MODE_NRF) {
            ESP_LOGW(BIOGAP_READ_TAG, "Not in NRF SPI mode, skipping");
            vTaskDelay(pdMS_TO_TICKS(100));
            continue;
        }

        /* Get result of current transaction (blocks until complete) */
        spi_slave_transaction_t *ret_trans = NULL;
        esp_err_t ret = spi_slave_get_trans_result(SPI_HOST_DEVICE, &ret_trans, portMAX_DELAY);
        if (ret != ESP_OK) {
            ESP_LOGE(BIOGAP_READ_TAG, "Failed to get SPI result: %s", esp_err_to_name(ret));
            break;
        }

        size_t rx_bytes = (size_t)(ret_trans->trans_len / 8);
        if (rx_bytes != PACKET_SZ) {
            ESP_LOGW(BIOGAP_READ_TAG,
                     "SPI length mismatch: expected=%uB got=%uB (master likely clocks different length)",
                     (unsigned)PACKET_SZ,
                     (unsigned)rx_bytes);
        }

        uint8_t *rx_data = (uint8_t *)ret_trans->rx_buffer;

        /* Validate packet markers (header + tailer) */
        if (!is_valid_packet(rx_data)) {
            ESP_LOGW(BIOGAP_READ_TAG, "Invalid packet: hdr=0x%02X tail=0x%02X (expected 0x%02X/0x%02X)",
                     rx_data[0], rx_data[PACKET_SZ - 1], NRF_EXG_HEADER, NRF_EXG_TAILER);
            break;
        }

        /* Extract counter (little-endian at bytes[1:2]) */
        uint16_t counter = (rx_data[2] << 8) | rx_data[1];
        packet_count++;

        /* Log periodically */
        int64_t now_us = esp_timer_get_time();
        if ((now_us - last_log_us) >= (DRDY_LOG_INTERVAL_MS * 1000LL)) {
            // ESP_LOGI(BIOGAP_READ_TAG, "Packet #%" PRIu64 " OK: counter=%u, hdr=0x%02X, tail=0x%02X",
            //          packet_count, counter, rx_data[0], rx_data[PACKET_SZ - 1]);
            last_log_us = now_us;
        }

        /* Add packet to ringbuffer */
        ret = add_to_ringbuffer(rx_data, PACKET_SZ);
        if (ret != ESP_OK) {
            ESP_LOGE(BIOGAP_READ_TAG, "Failed to add packet to ringbuffer");
            break;
        }

        /* Update TX buffer with new counter for next transaction */
        tx_counter++;  /* Advance transmit counter */
        sendbuf_persistent[0] = ESP_EXG_HEADER;
        sendbuf_persistent[PACKET_SZ - 1] = ESP_EXG_TAILER;
        sendbuf_persistent[1] = tx_counter & 0xFF;  /* LSB of counter */
        sendbuf_persistent[2] = (tx_counter >> 8) & 0xFF;  /* MSB of counter */

        /* === KEY PATTERN: RE-QUEUE IMMEDIATELY ===
         * This is the core of the high-speed pattern:
         * Return the descriptor to the queue IMMEDIATELY after processing.
         * This keeps the slave armed continuously with no gaps.
         */
        ret = spi_slave_queue_trans(SPI_HOST_DEVICE, ret_trans, 0);
        if (ret != ESP_OK) {
            ESP_LOGE(BIOGAP_READ_TAG, "Failed to re-queue transaction: %s", esp_err_to_name(ret));
            break;
        }
    }

    /* === CLEANUP === */
    ESP_LOGI(BIOGAP_READ_TAG, "Freeing pre-queued DMA buffers and exiting task");
    if (sendbuf_persistent) {
        free(sendbuf_persistent);
    }
    for (int i = 0; i < QUEUE_COUNT; i++) {
        if (rx_bufs[i]) {
            free(rx_bufs[i]);
        }
    }
    vTaskDelete(NULL);
}
