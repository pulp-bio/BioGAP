#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "nvs_flash.h"
#include "esp_timer.h"
#include "freertos/queue.h"
#include "freertos/ringbuf.h"

#include "driver/gpio.h"            // will be used for the interrupt pin
#include "driver/gptimer.h"         // use for now to fake GPIO interrupt
#include "esp_intr_alloc.h"
#include "driver/spi_master.h"
#include "common.h"

/*Timer*/
void timer_init();
/*Initialization of Task to Read From Sensor*/

void read_from_biogap_task_nrf_master_esp_slave(void *pv); 

esp_err_t propagate_start_command_to_biogap_master();
void send_to_biogap_task_nrf_master_esp_slave(void *pv);

void read_from_biogap_task_nrf_master_esp_slave_prequeue(void *pv); 
void initial_handshake_nrf_master_esp_slave_pq();

extern TaskHandle_t read_from_biogap_task_nrf_master_pq_esp_slave_handle; 
extern TaskHandle_t send_to_biogap_task_nrf_master_esp_slave_handle; 

size_t prepare_buffer(uint8_t *buffer, uint32_t counter, uint16_t bytes_per_node);

#define ESP_SPI_HEADER 0x66             // Header byte for every ESP <--> NRF transaction, to verify correct data parsing
#define ESP_SPI_TAILER 0xBB             // Tailer byte for every ESP <--> NRF transaction, to verify correct data parsing