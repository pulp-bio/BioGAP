/*
 * ----------------------------------------------------------------------
 *
 * File: biogap_send.c
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

#include <string.h>
#include "biogap.h"
#include "common.h"
#include "gui_task.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "driver/spi_slave.h"


bool send_start_command_to_biogap_master = false;
#define BIOGAP_SEND_TAG "biogap_send"
#define NRF_DRDY_PULSE_US 100

/* SPI buffers to transmit - receive data from BIOGAP NRF. Only ever touched
 * from send_to_biogap_task_nrf_master_esp_slave() via spi_slave_transmit() --
 * nothing else on the bulk pre-queue side is armed yet at this point (see
 * biogap_read.c), so there's no other task to coordinate with. */
static uint8_t tx_to_biogap_buf[SPI_FROM_BIOGAP_MAX_SIZE] __attribute__((aligned(4)));
static uint8_t rx_from_biogap_buf[SPI_FROM_BIOGAP_MAX_SIZE] __attribute__((aligned(4)));

static TaskHandle_t send_to_biogap_task_handle = NULL;
static esp_timer_handle_t drdy_pulse_timer_handle = NULL;

/** @brief Timer callback that ends the DRDY pulse and wakes the waiting sender task. */
static void drdy_pulse_timer_callback(void *arg)
{
    gpio_set_level(NRF_ESP_DATA_READY, 0);
    if (send_to_biogap_task_handle != NULL) {
        xTaskNotifyGive(send_to_biogap_task_handle);
    }
}

/** @brief Pulse the DRDY GPIO high for NRF_DRDY_PULSE_US to signal a pending command. */
static esp_err_t data_ready_pulse(void)
{
    esp_err_t ret;

    gpio_set_level(NRF_ESP_DATA_READY, 1);
    ret = esp_timer_start_once(drdy_pulse_timer_handle, NRF_DRDY_PULSE_US);
    if (ret != ESP_OK) {
        ESP_LOGE(BIOGAP_SEND_TAG, "Failed to start DRDY pulse timer: %s", esp_err_to_name(ret));
        return ret;
    }

    ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
    return ESP_OK;
}

/**
 * @brief Transmit a variable-length control frame [HEADER, LENGTH, ...LENGTH
 * bytes of payload..., TAILER] to the NRF and block until it's confirmed
 * delivered. cmd_len is the number of command/config bytes as received from
 * the GUI (the whole accumulated config+start sequence, not just a single
 * opcode) -- total frame size on the wire is cmd_len + 3. The explicit
 * LENGTH byte lets the NRF know exactly how many payload bytes to expect
 * without scanning for the tailer, which would be ambiguous if a payload
 * byte happened to equal ESP_SPI_TAILER.
 *
 * Uses the blocking spi_slave_transmit(), not the queue-based
 * spi_slave_queue_trans()/get_trans_result() API -- safe here specifically
 * because the bulk pre-queue pool is never armed until after this call
 * returns (see biogap_read.c), so nothing else is ever using the queue-based
 * API on this host at the same time. Mixing spi_slave_transmit() with an
 * ongoing queue on the same host is unsafe per the ESP-IDF SPI slave driver
 * docs, but there is no ongoing queue during this call.
 *
 * spi_slave_transmit() blocks until a full transaction actually completes,
 * so by the time it returns, our command content has genuinely been clocked
 * out to the NRF. The retry loop below only guards against a corrupted/
 * glitched completion (rx content not matching the NRF's known dummy-poll
 * reply pattern, all 0xBB) -- it re-arms and waits again with the same
 * command content rather than trusting a bad transfer.
 *
 * @note The transaction is always a fixed SPI_FROM_BIOGAP_MAX_SIZE bytes,
 * matching the NRF's poll (process_esp_data()) exactly -- SPI full-duplex
 * requires both sides to agree on transaction length, so this can't be
 * sized to cmd_len even though only the first cmd_len + 3 bytes are
 * meaningful. The NRF already only reads that meaningful prefix (header,
 * length, payload, tailer) and ignores the rest, per the explicit length
 * byte -- it doesn't need the transaction itself to be short.
 */
static esp_err_t send_first_start_command_to_biogap_master(const uint8_t *command, size_t cmd_len)
{
    esp_err_t ret;

    if (cmd_len > SPI_FROM_BIOGAP_MAX_SIZE - 3 || cmd_len > 255) {
        ESP_LOGE(BIOGAP_SEND_TAG, "Command too large to relay (cmd_len=%u)", (unsigned)cmd_len);
        return ESP_ERR_INVALID_SIZE;
    }
    size_t frame_len = cmd_len + 3;  /* header + length byte + payload + tailer -- meaningful prefix only */

    tx_to_biogap_buf[0] = ESP_SPI_HEADER;
    tx_to_biogap_buf[1] = (uint8_t)cmd_len;
    memcpy(&tx_to_biogap_buf[2], command, cmd_len);
    tx_to_biogap_buf[2 + cmd_len] = ESP_SPI_TAILER;
    /* Padding beyond the meaningful prefix, up to the fixed transaction
     * size -- content doesn't matter, the NRF never reads past the tailer. */
    memset(&tx_to_biogap_buf[frame_len], 0x00, SPI_FROM_BIOGAP_MAX_SIZE - frame_len);
    ESP_LOGI(BIOGAP_SEND_TAG, "Relaying %u-byte control frame to NRF (first byte=0x%02X)", (unsigned)frame_len, tx_to_biogap_buf[0]);

    bool delivered = false;
    while (!delivered) {
        memset(rx_from_biogap_buf, 0x00, SPI_FROM_BIOGAP_MAX_SIZE);

        spi_slave_transaction_t t = {0};
        t.length = SPI_FROM_BIOGAP_MAX_SIZE * 8;
        t.tx_buffer = tx_to_biogap_buf;
        t.rx_buffer = rx_from_biogap_buf;

        if (!SPI_BUS_LOCK(portMAX_DELAY)) {
            ESP_LOGE(BIOGAP_SEND_TAG, "Failed to lock SPI bus mutex for command transmit");
            return ESP_FAIL;
        }
        ret = spi_slave_transmit(SPI_HOST_DEVICE, &t, portMAX_DELAY);
        SPI_BUS_UNLOCK();
        if (ret != ESP_OK) {
            ESP_LOGE(BIOGAP_SEND_TAG, "Failed to transmit command to BIOGAP master: %s", esp_err_to_name(ret));
            return ret;
        }

        delivered = true;
        for (size_t i = 0; i < SPI_FROM_BIOGAP_MAX_SIZE; i++) {
            if (rx_from_biogap_buf[i] != 0xBB) {
                delivered = false;
                break;
            }
        }
        if (!delivered) {
            ESP_LOGW(BIOGAP_SEND_TAG, "Control frame transceive looked corrupted, retrying");
        }
    }

    ESP_LOGI(BIOGAP_SEND_TAG, "Control frame delivered, len=%u first byte=0x%02X", (unsigned)frame_len, tx_to_biogap_buf[0]);
    return ESP_OK;
}

/**
 * @brief Pulse DRDY, transmit the control frame, then (once delivery is
 * confirmed) allocate the bulk pre-queue buffers and hand off to the
 * streaming phase. Used for the GUI's whole config+start sequence, relayed
 * as exactly one frame while STATE_IDLE (see parse_gui_command(),
 * gui_utils.c) -- the ESP never distinguishes config bytes from the start
 * opcode within it.
 *
 * allocate_prequeue_resources() is deliberately called here, after delivery
 * is confirmed, rather than any earlier -- read_from_biogap_task_nrf_master_esp_slave_prequeue()
 * (biogap_read.c) waits on B_START_CMD_FWD_TO_BIOGAP before ever touching
 * the bulk pool or the queue-based SPI API, so there's no other task to race
 * with here either.
 *
 * @note Cross-firmware dependency: the NRF's process_esp_data() must poll
 * with a fixed SPI_FROM_BIOGAP_MAX_SIZE-byte dummy buffer (all 0xBB) --
 * matching this side's fixed transaction length exactly, per
 * send_first_start_command_to_biogap_master()'s doc comment. Also requires
 * ESP_PCKT_MAX_SIZE (NRF) == SPI_FROM_BIOGAP_MAX_SIZE (ESP, 250).
 */
esp_err_t propagate_first_start_command_to_biogap_master(const uint8_t *command, size_t cmd_len)
{
    esp_err_t ret = data_ready_pulse();
    if (ret != ESP_OK) {
        return ret;
    }

    ret = send_first_start_command_to_biogap_master(command, cmd_len);
    if (ret != ESP_OK) {
        return ret;
    }

    ret = allocate_prequeue_resources();
    if (ret != ESP_OK) {
        return ret;
    }

    xEventGroupClearBits(g_evt, B_STOP_CMD_FWD_TO_BIOGAP);
    node_state = STATE_STREAMING;
    xEventGroupSetBits(g_evt, B_START_CMD_FWD_TO_BIOGAP);
    return ESP_OK;
}


/** @brief Task: forwards GUI START commands to the NRF over SPI. */
void send_to_biogap_task_nrf_master_esp_slave(void *pv){

    esp_err_t ret; 
    send_to_biogap_task_handle = xTaskGetCurrentTaskHandle();
    bool first_time = true; 

    if (drdy_pulse_timer_handle == NULL) {
        esp_timer_create_args_t timer_args = {
            .callback = drdy_pulse_timer_callback,
            .arg = NULL,
            .dispatch_method = ESP_TIMER_TASK,
            .name = "drdy_pulse",
            .skip_unhandled_events = true,
        };

        ret = esp_timer_create(&timer_args, &drdy_pulse_timer_handle);
        if (ret != ESP_OK) {
            ESP_LOGE(BIOGAP_SEND_TAG, "Failed to create DRDY pulse timer: %s", esp_err_to_name(ret));
            vTaskDelete(NULL);
            return;
        }
    }

    /* Wait once for the NRF link to come up. After that, block on command events. */
    xEventGroupWaitBits(g_evt, B_BIOGAP_CONECTED, pdFALSE, pdFALSE, portMAX_DELAY);

    while(1){
        /* Sleep until a command or backpressure event arrives. */
        /* Wake on START, RINGBUFFER backpressure, or GUI STOP so this
         * task can process STOP handoff when GUI issues it. Use
         * pdFALSE (do not clear) so handlers can explicitly clear bits
         * after processing.
         */
        EventBits_t bits = xEventGroupWaitBits(g_evt,
                               B_START_CMD_RCV | B_RINGBUFFER_FULL | B_SPI_QUIESCED,
                               pdTRUE,
                               pdFALSE,
                               portMAX_DELAY);

        if (bits & B_START_CMD_RCV) {

            ESP_LOGI(BIOGAP_SEND_TAG, "Relaying GUI command to NRF, opcode=%d len=%u",
                    (unsigned)rx_gui_data_to_fwd[0], (unsigned)rx_gui_data_len);

            /* This task's whole job is relaying bytes -- it never decides
             * what an opcode means. rx_gui_data_to_fwd holds the GUI's whole
             * config+start sequence, accumulated across however many chunks
             * it took to arrive (see parse_gui_command(), gui_utils.c) and
             * relayed here as exactly one control frame. Once that one
             * relay is confirmed delivered, propagate_first_start_command_to_biogap_master()
             * arms the bulk pool and flips node_state to STREAMING itself --
             * no separate readiness signal from the NRF is needed. */
            ret = propagate_first_start_command_to_biogap_master(rx_gui_data_to_fwd, rx_gui_data_len);
            if (ret != ESP_OK) {
                ESP_LOGE(BIOGAP_SEND_TAG, "Failed to relay command to NRF, will retry: %s", esp_err_to_name(ret));
                vTaskDelay(pdMS_TO_TICKS(10));
                xEventGroupSetBits(g_evt, B_START_CMD_RCV);
            }
        }
        if (bits & B_RINGBUFFER_FULL) {
            ESP_LOGW(BIOGAP_SEND_TAG, "Ringbuffer backpressure signaled");
        }
    }
    vTaskDelete(NULL);
}
