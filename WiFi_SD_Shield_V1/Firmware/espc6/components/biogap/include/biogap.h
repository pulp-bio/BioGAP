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
#include "gui_task.h"
#include "connectivity_commands.h"

/*Ringbuffer to store incoming data from BIOGAP*/
extern RingbufHandle_t biogap_ringbuf;


/*Timer*/
void timer_init();

/* Tasks to Read From Sensor*/
void read_from_biogap_task_nrf_master_esp_slave(void *pv); 
void send_to_biogap_task_nrf_master_esp_slave(void *pv);
extern TaskHandle_t read_from_biogap_task_nrf_master_pq_esp_slave_handle; 
extern TaskHandle_t send_to_biogap_task_nrf_master_esp_slave_handle; 
void read_from_biogap_task_nrf_master_esp_slave_prequeue(void *pv); 


/* Handshake and Initialization Functions */
int initial_handshake_nrf_master_esp_slave_pq();
esp_err_t propagate_start_command_to_biogap_master();


size_t prepare_buffer(uint8_t *buffer, uint32_t counter, uint16_t bytes_per_node);

esp_err_t allocate_prequeue_resources(void);
esp_err_t free_prequeue_resources(void);

/** @brief Re-arm QUEUE_COUNT idle-content transactions on the SPI slave bus,
 *  so it's never left unarmed between a STOP and the next START. */
esp_err_t rearm_idle_prequeue(void);

/** @brief Copy len bytes into biogap_ringbuf for tx_to_gui() to drain and send to BioGUI */
esp_err_t add_to_ringbuffer(const uint8_t *data, size_t len);
#define ESP_SPI_HEADER 0x66             // Header byte for every ESP <--> NRF transaction, to verify correct data parsing
#define ESP_SPI_TAILER 0xBB             // Tailer byte for every ESP <--> NRF transaction, to verify correct data parsing
#define NRF_STOP_ACK_MASK 0x80         // Bit mask for STOP ACK in the NRF's response header byte
/* Pre-queue configuration: keep this many transactions armed at all times */
#define QUEUE_COUNT 4

/* Persistent TX buffer shared with pre-queued SPI descriptors (allocated in reader) */
extern uint8_t *sendbuf_persistent;
extern uint8_t *rx_bufs[QUEUE_COUNT];           /* RX buffers for each pre-queued desc */
extern uint8_t *tx_bufs[QUEUE_COUNT];          /* TX buffers for each pre-queued desc */