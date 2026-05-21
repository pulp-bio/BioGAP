
#include "esp_log.h"
#include "sd_main.h"
#include "biogap.h"

#include "common.h"
#include "led_app.h"
#include "driver/gpio.h"
#include <stdio.h>
#include <string.h>
#define MAIN_TAG "[main.c]" 

RingbufHandle_t ringbuff = NULL; 
TaskHandle_t sd_card_task_handle = NULL;

TaskHandle_t read_from_biogap_task_nrf_master_pq_esp_slave_handle = NULL;
TaskHandle_t send_to_biogap_task_nrf_master_esp_slave_handle = NULL;
EventGroupHandle_t g_evt;
// temporary
uint8_t sd_writecounter = 0;
spi_device_handle_t nrf_spi_device = NULL;

void app_main()
{
    esp_err_t ret= 0;
    /* Initializations*/

    // Logging policy controlled by compile-time user flag in common.h.
#if ESP_ENABLE_INFO_LOGS
    esp_log_level_set("*", ESP_LOG_INFO);
    esp_log_level_set("gpio", ESP_LOG_WARN); // Keep GPIO dump lines muted even in INFO mode.
#else
    esp_log_level_set("*", ESP_LOG_WARN);
#endif
    
    // Initialize Event Group
    g_evt = xEventGroupCreate();
    xEventGroupClearBits(g_evt, B_EXPECTED_STREAMING | B_CONNECTED | B_NETWORK_CONGESTED);

    pin_mux_init();
    config_spi_nrf_master_esp_slave_drdy_pin(); 
    ESP_LOGI(MAIN_TAG, "Pin muxing and NRF DRDY pin configuration complete");

    // wait a bit
    vTaskDelay(pdMS_TO_TICKS(500));

    // Initialize SD card 
    #if ESP_ENABLE_SD_WRITE
        ESP_LOGI(MAIN_TAG, "Initializing SD Card, using SPI peripheral\n"); 
        ret= sd_card_init();              
        if (ret != ESP_OK) {
            ESP_LOGE(MAIN_TAG, "Failed to initialize SD card");
            // stay here forewer
            while(1) {
                vTaskDelay(pdMS_TO_TICKS(1000));
            }
        }
        ret = write_hello_word();
        if (ret!=ESP_OK){
            ESP_LOGE(MAIN_TAG, "Failed to write startup message to SD card");
            while(1) {
                vTaskDelay(pdMS_TO_TICKS(1000));
            }
        }
        
    #endif

    // Initialize the onboard RGB LED and run a short startup sequence.
    #if ENABLE_LED_STRIP
         ESP_LOGI(MAIN_TAG, "Configuring onboard RGB LED");
         configure_led();
    #endif
    // Switch back the MUX since the card initialization was completed 
    ESP_ERROR_CHECK(gpio_set_level(MUX_SEL, 0));

    switch_to_nrf_master_spi_mode(); 
    


    // Create the RingBuffer to allocate nodes data
    ringbuff = xRingbufferCreate(RINGBUFF_SIZE, RINGBUF_TYPE_NOSPLIT);
    if (ringbuff == NULL){
      ESP_LOGE(MAIN_TAG, "Failed to allocate Memory for the Ringbuffer");
      abort();
    }
    ESP_LOGI(MAIN_TAG, "Ringbuffer created with size %d bytes", RINGBUFF_SIZE);

    BaseType_t xr = xTaskCreate(read_from_biogap_task_nrf_master_esp_slave_prequeue, "read_biogap_task_nrf_master", 4096, NULL, 5, &read_from_biogap_task_nrf_master_pq_esp_slave_handle);
    xr = xTaskCreate(send_to_biogap_task_nrf_master_esp_slave, "send_to_biogap_task_nrf_master", 4096, NULL, 5, &send_to_biogap_task_nrf_master_esp_slave_handle);
    if (xr != pdPASS) {
        ESP_LOGE(MAIN_TAG, "Failed to create read_from_biogap task (err=%d)", xr);
    } else {
        ESP_LOGI(MAIN_TAG, "Created read_from_biogap task");
    }


    #if ESP_ENABLE_SD_WRITE
        BaseType_t xs = xTaskCreate(sd_card_task, "sd_card_task", 8192, NULL, 5, &sd_card_task_handle);
        if (xs != pdPASS) {
            ESP_LOGE(MAIN_TAG, "Failed to create sd_card_task (err=%d)", xs);
        } else {
            ESP_LOGI(MAIN_TAG, "Created sd_card_task");
        }
    #endif
    
    


}


