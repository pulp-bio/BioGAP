/*
 * ----------------------------------------------------------------------
 *
 * File: main.c
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

#include "esp_log.h"
#include "softap_main.h"
#include "gui_task.h"
#include "biogap.h"
#include "dummy_sensor_local.h"
#include "common.h"
#include "driver/gpio.h"
#include <stdio.h>
#include <string.h>

#define MAIN_TAG "[main.c]"

RingbufHandle_t biogap_ringbuf = NULL; 
TaskHandle_t sd_card_task_handle = NULL;
TaskHandle_t read_from_biogap_task_nrf_master_pq_esp_slave_handle = NULL;
TaskHandle_t send_to_biogap_task_nrf_master_esp_slave_handle = NULL;
EventGroupHandle_t g_evt;
uint8_t sd_writecounter = 0;
spi_device_handle_t nrf_spi_device = NULL;
volatile node_state_t node_state = STATE_DISCONNECTED;
int64_t start_time = 0;
/** @brief Boot sequence: GPIO/SPI/WiFi bring-up, then spawn the data-plane tasks. */
void app_main()
{
    esp_err_t ret= 0;

    // Initialize the debugging GPIO 
    gpio_config_t io_conf = {0};
    io_conf.pin_bit_mask = (1ULL << RTC_SCL);
    io_conf.mode = GPIO_MODE_OUTPUT;
    io_conf.pull_up_en = GPIO_PULLUP_ENABLE;            // Enable pull-up resistor (in any case this GPIO should pulled up hihg)
    io_conf.pull_down_en = GPIO_PULLDOWN_DISABLE;
    gpio_config(&io_conf);

    // set the RTC_SCL pin HIGH to enable debugging for now
    gpio_set_level(RTC_SCL, 1);

    // Logging policy controlled by compile-time user flag in common.h.
    #if ESP_ENABLE_INFO_LOGS
        esp_log_level_set("*", ESP_LOG_INFO);
        esp_log_level_set("gpio", ESP_LOG_WARN); // Keep GPIO dump lines muted even in INFO mode.
    #else
        esp_log_level_set("*", ESP_LOG_WARN);
    #endif

    ESP_LOGI(MAIN_TAG, "Starting app_main, initializing system...");
    // Initialize Event Group and clear all bits
    g_evt = xEventGroupCreate();
    xEventGroupClearBits(g_evt, B_WIFI_CONNECTED | B_BIOGAP_CONECTED | B_START_CMD_RCV | B_START_CMD_FWD_TO_BIOGAP | B_STOP_CMD_RCV_GUI | B_STOP_CMD_RCV_FORCED | B_STOP_CMD_FWD_TO_BIOGAP);
    
    

#if !ESP_LOCAL_DUMMY_SENSOR

    spi_bus_mutex = xSemaphoreCreateMutex();
    if (spi_bus_mutex == NULL) {
        ESP_LOGE(MAIN_TAG, "Failed to create SPI bus mutex");
        abort();
    }

    // SPI Handshake with NRF
    #if defined IS_ESP_SPI_SLAVE
        ESP_LOGI(MAIN_TAG, "ESP is configured as SPI SLAVE");
        // Initialize SPI GPIOs
        config_spi_nrf_master_esp_slave_pins();
        ESP_LOGI(MAIN_TAG, "NRF-ESP GPIO pins configured successfully");
        // Initialize the SPI BUS
        init_nrf_spi_master_esp_slave_bus();
        ESP_LOGI(MAIN_TAG, "NRF-ESP SPI bus initialized successfully");
    #endif


    uint8_t handshake_attempts = 0;
    while (handshake_attempts < 1) {
    //while(1){
        ret = initial_handshake_nrf_master_esp_slave_pq();
        if (ret == 0) {
            //ESP_LOGI(MAIN_TAG, "Initial handshake with NRF master successful");
            ESP_LOGI(MAIN_TAG, "handshake num %d successful", handshake_attempts);
            current_spi_mode = SPI_MODE_NRF;
            handshake_attempts+=1; 
            //break;
        } else {
            ESP_LOGE(MAIN_TAG, "Initial handshake failed, retrying in 1 second: %s", esp_err_to_name(ret));
            vTaskDelay(pdMS_TO_TICKS(1000));
        }
    }
#endif

    // Wi-Fi Initialization
    //ESP_LOGI(MAIN_TAG, "Before init Wifi free heap: %d, free DMA: %d", heap_caps_get_free_size(MALLOC_CAP_INTERNAL), heap_caps_get_free_size(MALLOC_CAP_DMA));
    
    vTaskDelay(pdMS_TO_TICKS(1000));
    if (wifi_init_softap() != ESP_OK) {
        ESP_LOGE(MAIN_TAG, "Failed to initialize WiFi");
        abort();
    }
    ESP_LOGI(MAIN_TAG, "After init Wifi free heap: %d, free DMA: %d", heap_caps_get_free_size(MALLOC_CAP_INTERNAL), heap_caps_get_free_size(MALLOC_CAP_DMA));
    
    xEventGroupSetBits(g_evt, B_WIFI_CONNECTED);




#if ESP_LOCAL_DUMMY_SENSOR
    // ESP-only dummy sensor test: bypasses SPI/BIOGAP entirely. Generates synthetic
    // dummy-sensor packets locally
     and streams them to BioGUI over the TCP connection,
    // to test the WiFi/GUI half of the system without any nRF/SPI hardware attached.
    biogap_ringbuf = xRingbufferCreate(RINGBUFF_SIZE, RINGBUF_TYPE_NOSPLIT);
    if (biogap_ringbuf == NULL) {
        ESP_LOGE(MAIN_TAG, "Failed to allocate memory for the BIOGAP ringbuffer");
        abort();
    }
    ESP_LOGI(MAIN_TAG, "BIOGAP ringbuffer created with size %d bytes", RINGBUFF_SIZE);

    ret = bind_to_gui();
    if (ret != ESP_OK) {
        ESP_LOGE(MAIN_TAG, "Failed to bind to GUI");
        abort();
    }
    xEventGroupSetBits(g_evt, B_GUI_SOCKET_BIND);
    node_state = STATE_IDLE;
    ESP_LOGI(MAIN_TAG, "WiFi initialized and bound to GUI successfully (local dummy sensor mode)");

    BaseType_t xr = xTaskCreate(rx_from_gui, "rx_from_gui", 4096, NULL, 3, NULL);
    if (xr != pdPASS) {
        ESP_LOGE(MAIN_TAG, "Failed to create rx_from_gui task (err=%d)", xr);
    } else {
        ESP_LOGI(MAIN_TAG, "Created rx_from_gui task");
    }

    xr = xTaskCreate(tx_to_gui, "tx_to_gui", 4096, NULL, 3, NULL);
    if (xr != pdPASS) {
        ESP_LOGE(MAIN_TAG, "Failed to create tx_to_gui task (err=%d)", xr);
    } else {
        ESP_LOGI(MAIN_TAG, "Created tx_to_gui task");
    }

    xr = xTaskCreate(dummy_sensor_local_task, "dummy_sensor_local", 4096, NULL, 2, NULL);
    if (xr != pdPASS) {
        ESP_LOGE(MAIN_TAG, "Failed to create dummy_sensor_local task (err=%d)", xr);
    } else {
        ESP_LOGI(MAIN_TAG, "Created dummy_sensor_local task");
    }
#else

    // Create the RingBuffer to allocate incoming data.
    biogap_ringbuf = xRingbufferCreate(RINGBUFF_SIZE, RINGBUF_TYPE_NOSPLIT);
    if (biogap_ringbuf == NULL){
      ESP_LOGE(MAIN_TAG, "Failed to allocate Memory for the BIOGAP ringbuffer");
      abort();
    }
    ESP_LOGI(MAIN_TAG, "BIOGAP Ringbuffer created with size %d bytes", RINGBUFF_SIZE);


    // Bind to the GUI socket. 
    ret = bind_to_gui();
    if (ret != ESP_OK) {
        ESP_LOGE(MAIN_TAG, "Failed to bind to GUI");
        abort();
    }

    xEventGroupSetBits(g_evt, B_GUI_SOCKET_BIND);
    node_state = STATE_IDLE;
    ESP_LOGI(MAIN_TAG, "WiFi initialized and bound to GUI successfully");
    // ========================== START ALL THE TASKS =============================

    /* Priorities: both biogap tasks (real SPI relay to/from the NRF,
     * including STOP handling) sit ABOVE the GUI-facing network tasks
     * below, not below them as before. ESP32-C6 is single-core with strict
     * priority-preemptive scheduling -- a lower-priority task gets zero CPU
     * while any higher-priority task is ready. tx_to_gui mostly blocks
     * (successful send()s, or an empty ringbuffer) so this normally doesn't
     * matter, but once the GUI's socket goes bad, send_all() fails fast
     * (no blocking) and tx_to_gui becomes a tight back-to-back loop for as
     * long as the ringbuffer keeps having items -- which, at the old
     * priorities, could starve read_from_biogap_task_nrf_master_esp_slave_prequeue
     * (including its STOP-quiesce poll) for as long as that lasted. */
    BaseType_t xr = xTaskCreate(read_from_biogap_task_nrf_master_esp_slave_prequeue, "read_biogap_task_nrf_master", 4096, NULL, 5, &read_from_biogap_task_nrf_master_pq_esp_slave_handle);
    if (xr != pdPASS) {
        ESP_LOGE(MAIN_TAG, "Failed to create read_from_biogap task (err=%d)", xr);
    } else {
        ESP_LOGI(MAIN_TAG, "Created read_from_biogap task");
    }

    xr = xTaskCreate(send_to_biogap_task_nrf_master_esp_slave, "send_to_biogap_task_nrf_master", 4096, NULL, 4, &send_to_biogap_task_nrf_master_esp_slave_handle);
    if (xr != pdPASS) {
        ESP_LOGE(MAIN_TAG, "Failed to create send_to_biogap task (err=%d)", xr);
    } else {
        ESP_LOGI(MAIN_TAG, "Created send_to_biogap task");
    }
    xr = xTaskCreate(rx_from_gui, "rx_from_gui", 4096, NULL, 3, NULL);
    if (xr != pdPASS) {
        ESP_LOGE(MAIN_TAG, "Failed to create rx_from_gui task (err=%d)", xr);
    } else {
        ESP_LOGI(MAIN_TAG, "Created rx_from_gui task");
    }

    xr = xTaskCreate(tx_to_gui, "tx_to_gui", 4096, NULL, 3, NULL);
    if (xr != pdPASS) {
        ESP_LOGE(MAIN_TAG, "Failed to create tx_to_gui task (err=%d)", xr);
    } else {
        ESP_LOGI(MAIN_TAG, "Created tx_to_gui task");
    }
#endif
    // Main Thread can now sleep and let the tasks do the work.

    while(1) {
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}


