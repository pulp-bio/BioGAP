// common.h
#pragma once

#include <stdint.h>
#include <stdbool.h>

// FreeRTOS / ESP-IDF
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "freertos/ringbuf.h"
#include "esp_timer.h"
// GPIO
#include "driver/gpio.h"
// SPI
#include "driver/spi_master.h"
#include "driver/spi_slave.h"


// ---------------------- Node definitons ---------------------
#define NODE_ID 3             // Unique node identifier. Must be changed manually for each node
static const uint8_t node_id = NODE_ID;

/* Handshake ping-pong markers to validate NRF-ESP communication */
#define HANDSHAKE_MARKER 0x5A
#define HANDSHAKE_RESPONSE_MARKER 0xA5


// -------------------- Task stack sizes --------------------
#define READ_FROM_BIOGAP_STACK_SIZE  4096



// -------------------- Packet format --------------------
#define PAYLOAD_SIZE                 1440    // application payload target for WiFi transmission, can be adjusted
#define HEADER_BYTES                 3
#define COUNTER_BYTES                4
#define TSF_BYTES                    8
#define NUM_SAMPLES_BYTES            2

#define PACKET_DEF_BYTES             (HEADER_BYTES + COUNTER_BYTES + TSF_BYTES + NUM_SAMPLES_BYTES)
#define SPACE_RESERVED_FOR_MASTER    3
#define CRC_BYTES                    0

#define EFFECTIVE_MAX_PAYLOAD        (PAYLOAD_SIZE - PACKET_DEF_BYTES - SPACE_RESERVED_FOR_MASTER - CRC_BYTES)

// NOTE: this is currently enforced by prepare_buffer(). Keep consistent with builder math.
#define EXPECTED_DATA_LEN            1436


// -------------------- Streaming / timing --------------------
#define NODE_FRAME_TAG               0x0A    // high nibble for node frame

// -------------------- Ringbuffer --------------------
#define RINGBUFF_SIZE                200000  // bytes
extern RingbufHandle_t ringbuff;

// -------------------- SD_CARD --------------------
#define SD_CARD_TRANSFER_SIZE         16000  // 16 KB at once 
#define MAX_SD_WRITES                10       
extern uint8_t sd_writecounter; 

// -------------------- Pin configurations --------------------
#define MUX_SEL       12                               // ESP pin to swith MUX between data read mode and SPI_ESP_CS_MUX (enable SPI transaction from mainboard)
void pin_mux_init();


// -------------------- Event bits --------------------
extern EventGroupHandle_t g_evt;

// These are for Wi-Fi Streaming
#define B_EXPECTED_STREAMING   (1U << 0)  // producer IRQ->task notifications enabled
#define B_CONNECTED            (1U << 1)  // TCP connected (socket usable)
#define B_CLEAN_PREV_TICKS     (1U << 2)  // one-shot: clear accumulated ticks/counters on start
#define B_STOP_CMD             (1U << 3)  // stop acquisition but keep connection
#define B_END_ACQUISITION      (1U << 4)  // shutdown
#define B_NETWORK_CONGESTED    (1U << 5)  // TX detected stall/backpressure
#define B_WRITING_TO_SD        (1U << 6)  // SD task currently owns SPI MUX path

#define MAX_RECONNECTION_ATTEMPTS  10

// -------------------- Integration mode flags --------------------
// Phase-1 bring-up defaults: verify NRF->ESP SPI transfer first, keep SD writes disabled.
#ifndef ESP_ENABLE_SD_WRITE
#define ESP_ENABLE_SD_WRITE          0
#endif

#ifndef ESP_DRDY_TOGGLE_TEST_ONLY
#define ESP_DRDY_TOGGLE_TEST_ONLY 1
#endif



// -------------------- NRF <-> ESP SPI pins --------------------
// ESP acts as SPI master while NRF is SPI slave.
#define NRF_SPI_MOSI_GPIO            7   // ESP MOSI  -> NRF P0.30 (NRF SPIS_MOSI)
#define NRF_SPI_MISO_GPIO            2   // ESP MISO  <- NRF P0.29 (NRF SPIS_MISO)
#define NRF_SPI_SCLK_GPIO            6   // ESP SCLK  -> NRF P1.07
#define NRF_SPI_CS_GPIO              18  // ESP CS    -> NRF P0.12
#define NRF_DATA_READY_GPIO          13  // ESP input <- NRF P0.04

// -------------------- SD Card SPI pins ------------------------
#define PIN_NUM_MISO  20                                // ESP MISO: received data from SD Card | SD CARD: mSD_DAT0 -> DAT0 
#define PIN_NUM_MOSI  18                                // ESP MOSI: writes data to SD Card | SD CARD: mSD_CMD ->CMD
#define PIN_NUM_CLK   19                                // ESP CLK: clock singal | SD CARD: mSD_CLK -> CLK
#define PIN_NUM_CS    23                                // ESP CS: chip select signal | SD CARD mSD_DAT3 -> CS (in spi mode)  

// Debug-only indication pin on ESP32-C6 (can be changed to match board LED).
#define ESP_DEBUG_LED_GPIO           8


// -------------------- Constants for NRF-ESP SPI transfer and packet building --------------------
#define SPI_FROM_BIOGAP_MAX_SIZE    1000   // max number of bytes in a single SPI transaction from BIOGAP (e.g. 410 if streaming from WULPUS)

#define EMULATE_EXG_DATA 0
#define EMULATE_US_DATA 1
#if (EMULATE_EXG_DATA == 1)
    #define NRF_EXG_PACKET_SIZE       211
    #define NRF_EXG_HEADER            0x55
    #define NRF_EXG_TAILER            0xAA
    #define ESP_EXG_HEADER            0x66
    #define ESP_EXG_TAILER            0xBB
#endif
#if (EMULATE_US_DATA == 1)
    #define NRF_EXG_PACKET_SIZE       811
    #define NRF_EXG_HEADER            0x55
    #define NRF_EXG_TAILER            0xAA
    #define ESP_EXG_HEADER            0x66
    #define ESP_EXG_TAILER            0xBB
#endif


#define NRF_SPI_CLOCK_HZ          4000000           // 4 MHz SPI clock for NRF-ESP transfer. 


//--------------------- SPI Transactions & Mode Switching --------
// Single SPI host constant used for both NRF and SD card modes (runtime switching)
#define SPI_HOST_DEVICE SPI2_HOST

// SPI bus ownership modes
typedef enum {
    SPI_MODE_IDLE = 0,
    SPI_MODE_NRF = 1,
    SPI_MODE_SD = 2
} spi_mode_t;

extern spi_device_handle_t nrf_spi_device;
extern spi_mode_t current_spi_mode;

esp_err_t biogap_read_hw_init(void);
esp_err_t switch_to_nrf_master_spi_mode(void);
esp_err_t switch_to_sd_spi_mode(void);
esp_err_t config_spi_nrf_master_esp_slave_drdy_pin(void);

extern bool handshake_pq_done;
extern bool send_start_command_to_biogap_master;
//---------------------- Logging tags ------------------------
// Set to 1 only when actively profiling SPI mode-switch overhead.
#ifndef ENABLE_SPI_PROFILE_LOGS
#define ENABLE_SPI_PROFILE_LOGS 0
#endif

#ifndef NOTIFY_USER_FOR_DRDY
#define NOTIFY_USER_FOR_DRDY 0
#endif

// 0: suppress INFO logs (default, faster runtime)
// 1: enable INFO logs for debugging/profiling
#ifndef ESP_ENABLE_INFO_LOGS
#define ESP_ENABLE_INFO_LOGS 1
#endif

#ifndef ENABLE_LED_STRIP
#define ENABLE_LED_STRIP 0
#endif