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

#include "biogap.h"
#include "common.h"
#include "gui_task.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "driver/spi_slave.h"


bool send_start_command_to_biogap_master = false;
#define BIOGAP_SEND_TAG "biogap_send"
#define NRF_DRDY_PULSE_US 100

uint8_t tx_to_biogap_buf[4] __attribute__((aligned(4)));
uint8_t rx_from_biogap_buf[4] __attribute__((aligned(4)));

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

/** @brief Transmit a 4-byte control frame (header, opcode, 0x00, tailer) to the NRF. */
static esp_err_t send_first_start_command_to_biogap_master(uint8_t command)
{
    esp_err_t ret;

    tx_to_biogap_buf[0] = ESP_SPI_HEADER;
    tx_to_biogap_buf[1] = command;                   
    tx_to_biogap_buf[2] = 0x00;                     // nothing
    tx_to_biogap_buf[3] = ESP_SPI_TAILER;
    rx_from_biogap_buf[0] = 0x00;
    rx_from_biogap_buf[1] = 0x00;
    rx_from_biogap_buf[2] = 0x00;
    rx_from_biogap_buf[3] = 0x00;


    spi_slave_transaction_t t = {0};
    t.length = sizeof(tx_to_biogap_buf) * 8;  /* 4 bytes = 32 bits */
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

    ESP_LOGI(BIOGAP_SEND_TAG, "Transmitted 4-byte control frame: [0] 0x%02X, [1] 0x%02X, [2] 0x%02X, [3] 0x%02X",
             tx_to_biogap_buf[0], tx_to_biogap_buf[1], tx_to_biogap_buf[2], tx_to_biogap_buf[3]);

    return ESP_OK;
}

/** @brief Pulse DRDY then send the 4-byte control frame; used for every START. */
esp_err_t propagate_first_start_command_to_biogap_master(uint8_t command)
{
    esp_err_t ret = data_ready_pulse();
    if (ret != ESP_OK) {
        return ret;
    }
    return send_first_start_command_to_biogap_master(command);
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

        if ((bits & B_START_CMD_RCV) && !(xEventGroupGetBits(g_evt) & B_START_CMD_FWD_TO_BIOGAP)) {
            
            uint8_t start_command = rx_gui_data_to_fwd[0]; 
            ESP_LOGI(BIOGAP_SEND_TAG, "Received command from GUI, propagating to BIOGAP master start_command=%d", (unsigned)start_command);

            ret = propagate_first_start_command_to_biogap_master(start_command);
            if (ret == ESP_OK) {
                /* Only now -- control frame confirmed delivered -- allocate the
                 * DMA buffers for the upcoming streaming session. */
                esp_err_t alloc_ret = allocate_prequeue_resources();
                if (alloc_ret != ESP_OK) {
                    ESP_LOGE(BIOGAP_SEND_TAG, "Failed to allocate pre-queue resources after START, will retry: %s", esp_err_to_name(alloc_ret));
                    vTaskDelay(pdMS_TO_TICKS(10));
                    xEventGroupSetBits(g_evt, B_START_CMD_RCV);
                } else {
                    send_start_command_to_biogap_master = true;
                    xEventGroupSetBits(g_evt, B_START_CMD_FWD_TO_BIOGAP);
                    xEventGroupClearBits(g_evt, B_STOP_CMD_FWD_TO_BIOGAP);
                    xEventGroupClearBits(g_evt, B_START_CMD_RCV);
                    node_state = STATE_STREAMING;
                    ESP_LOGI(BIOGAP_SEND_TAG, "Start command propagated to BIOGAP master successfully");
                }
            }
            else {
                ESP_LOGE(BIOGAP_SEND_TAG, "Failed to propagate start command to BIOGAP master, will retry: %s", esp_err_to_name(ret));
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
