
#include "biogap.h"
#include "common.h"
#include "esp_log.h"
#include "esp_timer.h"

bool send_start_command_to_biogap_master = false;
#define BIOGAP_SEND_TAG "biogap_send"
#define NRF_DRDY_PULSE_US 100

uint8_t tx_to_biogap_buf[4] __attribute__((aligned(4)));
uint8_t rx_from_biogap_buf[4] __attribute__((aligned(4)));

static TaskHandle_t send_to_biogap_task_handle = NULL;
static esp_timer_handle_t drdy_pulse_timer_handle = NULL;

static void drdy_pulse_timer_callback(void *arg)
{
    gpio_set_level(NRF_DATA_READY_GPIO, 0);
    if (send_to_biogap_task_handle != NULL) {
        xTaskNotifyGive(send_to_biogap_task_handle);
    }
}

esp_err_t propagate_start_command_to_biogap_master(){
    tx_to_biogap_buf[0] = ESP_SPI_HEADER;
    tx_to_biogap_buf[1] = 249;                       // will be replaced by what is received from BIOGUI 
    tx_to_biogap_buf[2] = 0x00;                     // nothing
    tx_to_biogap_buf[3] = ESP_SPI_TAILER;
    rx_from_biogap_buf[0] = 0x00;
    rx_from_biogap_buf[1] = 0x00;
    rx_from_biogap_buf[2] = 0x00;
    rx_from_biogap_buf[3] = 0x00;


    spi_slave_transaction_t t = {0};
    t.length = 32;  /* 4 bytes = 32 bits */
    t.tx_buffer = tx_to_biogap_buf;
    t.rx_buffer = rx_from_biogap_buf;

    esp_err_t ret = spi_slave_transmit(SPI_HOST_DEVICE, &t, portMAX_DELAY);
    if(ret !=ESP_OK){
        ESP_LOGE(BIOGAP_SEND_TAG, "Failed to send start command to BIOGAP master: %s", esp_err_to_name(ret));
        return ESP_FAIL;
    }

    return ESP_OK;
}

void send_to_biogap_task_nrf_master_esp_slave(void *pv){

    // here we should check the current status (connected,ide etc). Keep it simple for now to verify connectivity 
    esp_err_t ret; 
    send_to_biogap_task_handle = xTaskGetCurrentTaskHandle();

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

    while(1){

        if (!handshake_pq_done){
            // Wait for handshake to complete 
            vTaskDelay(pdMS_TO_TICKS(100));
            continue;
        }

        if(handshake_pq_done && !send_start_command_to_biogap_master){
            // send start command to NRF master to enable the data transfer
            // first, set DRDY pin HIGH to signal NRF master that ESP is ready to receive data. 
            // Give a Pulse on DRDY to signal NRF master that ESP is ready to receive data. This will trigger the NRF master to start clocking data over SPI.

            gpio_set_level(NRF_DATA_READY_GPIO, 1);
            ret = esp_timer_start_once(drdy_pulse_timer_handle, NRF_DRDY_PULSE_US);
            if (ret != ESP_OK) {
                ESP_LOGE(BIOGAP_SEND_TAG, "Failed to start DRDY pulse timer: %s", esp_err_to_name(ret));
                break;
            }

            ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
            ret = propagate_start_command_to_biogap_master();
            if (ret == ESP_OK) {
                send_start_command_to_biogap_master = true;
                ESP_LOGI(BIOGAP_SEND_TAG, "Start command sent; send task exiting so read task can continue streaming");
                break;
            }
            else{
                ESP_LOGE(BIOGAP_SEND_TAG, "Failed to propagate start command to BIOGAP master, will retry: %s", esp_err_to_name(ret));
                break;
            }
        }
    }
    vTaskDelete(NULL);
}