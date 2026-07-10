#include "common.h"
#include "esp_log.h"
#include "biogap.h"
#include "sd_main.h"

static const char *COMMON_TAG = "[common.c]: ";




// Track current SPI bus mode for safe mode switching
spi_mode_t current_spi_mode = SPI_MODE_IDLE;
SemaphoreHandle_t spi_bus_mutex = NULL;

/* Initialize ESP as SPI Slave, NRF will be the master */
esp_err_t init_nrf_spi_master_esp_slave_bus(void)
{   

    // Configuration for the SPI bus

    spi_bus_config_t buscfg = {
        .mosi_io_num = NRF_ESP_MOSI,
        .miso_io_num = NRF_ESP_MISO,
        .sclk_io_num = NRF_ESP_SCLK,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
        .data4_io_num = -1,
        .data5_io_num = -1,
        .data6_io_num = -1,
        .data7_io_num = -1,
        .max_transfer_sz = SPI_FROM_BIOGAP_MAX_SIZE,
        .flags = 0,
        .intr_flags = 0,
    };

    spi_slave_interface_config_t slvcfg = {
        .mode = 1,  // SPI Mode 1 required for DMA (CPOL=0, CPHA=1)
        .spics_io_num = NRF_ESP_CS,
        .queue_size = 10,            // sets how many transactions can be in the air. Must be higher for larger transaction intervals
        .flags = 0
        // .post_setup_cb = my_post_setup_cb,
        // .post_trans_cb = my_post_trans_cb
    };

    // Initialize SPI slave interface
    ESP_LOGW(COMMON_TAG, ">>> Initializing SPI SLAVE on host %d (DMA disabled, Mode 1)", SPI_HOST_DEVICE);
    ESP_LOGW(COMMON_TAG, ">>> MOSI=%d MISO=%d SCLK=%d CS=%d", NRF_ESP_MOSI, NRF_ESP_MISO, NRF_ESP_SCLK, NRF_ESP_CS);
    // DMA is now enabled 
    esp_err_t ret = spi_slave_initialize(SPI_HOST_DEVICE, &buscfg, &slvcfg, SPI_DMA_CH_AUTO);
    if (ret != ESP_OK) {
        ESP_LOGE(COMMON_TAG, ">>> FAILED to initialize NRF SPI slave: %s", esp_err_to_name(ret));
        return ret;
    }


    return ESP_OK;
}


esp_err_t config_spi_nrf_master_esp_slave_pins(void){
    esp_err_t ret;

    // Configure the Data Ready pin as output. 
    // Configure the Pin as Output
    gpio_config_t io_conf = {
        .pin_bit_mask = (1ULL << NRF_ESP_DATA_READY),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,  
        .pull_down_en = GPIO_PULLDOWN_ENABLE, 
        .intr_type = GPIO_INTR_DISABLE, 
    };

    ESP_ERROR_CHECK(gpio_reset_pin(NRF_ESP_DATA_READY));
    ESP_ERROR_CHECK(gpio_config(&io_conf));

    // Default state: LOW 
    ret = gpio_set_level(NRF_ESP_DATA_READY, 0);
    if (ret!=ESP_OK){
        ESP_LOGE(COMMON_TAG, "Failed to set NRF data-ready pin LOW: %s", esp_err_to_name(ret));
        return ret;
    }

    // Configure the DIR_CTRL pins
    gpio_config_t io_conf_dir_ctrl = {
        .pin_bit_mask = (1ULL << NRF_ESP_DRDY_DIR_CTRL),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,  
        .pull_down_en = GPIO_PULLDOWN_ENABLE, 
        .intr_type = GPIO_INTR_DISABLE, 
    };

    ESP_ERROR_CHECK(gpio_reset_pin(NRF_ESP_DRDY_DIR_CTRL));
    ESP_ERROR_CHECK(gpio_config(&io_conf_dir_ctrl));

    // LOW WHEN NRF IS MASTER, HIGH WHEN ESP IS MASTER.
    ret = gpio_set_level(NRF_ESP_DRDY_DIR_CTRL, 0);
    if (ret!=ESP_OK){
        ESP_LOGE(COMMON_TAG, "Failed to set NRF data-ready direction pin LOW: %s", esp_err_to_name(ret));
        return ret;
    }


    return ESP_OK;
}

// // For future use 
// esp_err_t config_spi_nrf_slave_esp_master_drdy_pin(void){
//     esp_err_t ret;
//     // Configure SPI data ready PIN
//     gpio_config_t drdy_cfg = {
//         .intr_type = GPIO_INTR_NEGEDGE,             // NRF will toggle DRDY pin LOW when data is ready, so trigger on falling edge
//         .mode = GPIO_MODE_INPUT,
//         .pin_bit_mask = (1ULL << NRF_DATA_READY_GPIO),
//         .pull_down_en = GPIO_PULLDOWN_DISABLE,      // no pull-down needed, NRF actively drives LOW when data is ready (original: GPIO_PULLDOWN_DISABLE)
//         .pull_up_en = GPIO_PULLUP_DISABLE,           // enable pull-up. NRF toggles the pin LOW when data is ready (oriignal: GPIO_PULLUP_ENABLE)
//     };
//     ESP_ERROR_CHECK(gpio_config(&drdy_cfg));
//     ret = gpio_install_isr_service(ESP_INTR_FLAG_IRAM);
//     if (ret != ESP_OK && ret != ESP_ERR_INVALID_STATE) {
//         ESP_ERROR_CHECK(ret);
//         return ret;
//     }
//     // Add ISR handler for NRF data ready interrupt. The handler will notify the read_from_biogap_task to read the data.
//     ESP_ERROR_CHECK(gpio_isr_handler_add(NRF_DATA_READY_GPIO, nrf_data_ready_isr, NULL));
//     ESP_LOGI(COMMON_TAG, "Configured NRF data-ready interrupt on GPIO %d", NRF_DATA_READY_GPIO);

//     return ret; 

// }

// esp_err_t biogap_read_nrf_master_esp_slave_hw_init(void)
// {
//     /* Function to Initialize SPI Master and DRDY pin for NRF to ESP Transaction*/
//     esp_err_t ret = ESP_OK;
//     #if ENABLE_SPI_PROFILE_LOGS
//         int64_t timer_start = esp_timer_get_time();
//     #endif
//     if (current_spi_mode != SPI_MODE_NRF) {
//         ret = init_nrf_spi_master_esp_slave_bus();
//         if (ret != ESP_OK) {
//             ESP_LOGE(COMMON_TAG, "Failed to initialize NRF SPI bus: %s", esp_err_to_name(ret));
//             return ret;
//         }
//         current_spi_mode = SPI_MODE_NRF;
//     } 
//     else {
//         ESP_LOGW(COMMON_TAG, "NRF SPI bus already initialized, skipping re-init");
//     }
//     #if ENABLE_SPI_PROFILE_LOGS
//         int64_t timer_stop = esp_timer_get_time();
//         ESP_LOGI(COMMON_TAG, "SPI bus initialization time: %lld us", (long long)(timer_stop - timer_start));
//     #endif


    
//     ret = config_spi_nrf_master_esp_slave_pins();
//     if (ret != ESP_OK) {
//         ESP_LOGE(COMMON_TAG, "Failed to configure NRF pins: %s", esp_err_to_name(ret));
//         return ret;
//     }


//     ESP_LOGI(COMMON_TAG,
//              "Initialized NRF SPI bus: MOSI=%d MISO=%d SCLK=%d CS=%d @ %d Hz (Mode: %d)",
//              SPI_NRF_MOSI,
//              SPI_NRF_MISO,
//              SPI_NRF_SCLK,
//              SPI_NRF_CS,
//              NRF_SPI_CLOCK_HZ,
//              current_spi_mode);
//     return ESP_OK;
// }




esp_err_t validate_command(uint8_t command) {

    switch (command) {
        case GET_DEVICE_SETTINGS:
            return ESP_OK; // No parameters to validate for this command
        case REQUEST_HARDWARE_VERSION:
            return ESP_OK; // No parameters to validate for this command
        case GET_BOARD_STATE:
            return ESP_OK; // No parameters to validate for this command
        case REQUEST_BATTERY_STATE:
            return ESP_OK; // No parameters to validate for this command
        case START_EEG_STREAMING:
            return ESP_OK; // No parameters to validate for this command
        case STOP_EEG_STREAMING:
            return ESP_OK; // No parameters to validate for this command
        case SET_BOARD_STATE:
            return ESP_OK; // No parameters to validate for this command
        case RESET_BOARD:
            return ESP_OK; // No parameters to validate for this command
        case ENTER_BOOTLOADERT_MODE:
            return ESP_OK; // No parameters to validate for this command
        case SET_TRIGGER_STATE:
            return ESP_OK; // No parameters to validate for this command
        case GO_TO_SLEEP:
            return ESP_OK; // No parameters to validate for this command
        case RESET_GAP9:
            return ESP_OK; // No parameters to validate for this command
        case START_MIC_STREAMING:
            return ESP_OK; // No parameters to validate for this command
        case STOP_MIC_STREAMING:
            return ESP_OK; // No parameters to validate for this command
        case REQUEST_AVAILABLE_SENSORS:
            return ESP_OK; // No parameters to validate for this command
        case REQUEST_FIRMWARE_VERSION:
            return ESP_OK; // No parameters to validate for this command
        case REQUEST_CONNECTING_STRING:
            return ESP_OK; // No parameters to validate for this command
        case START_STREAMING_ALL:
            return ESP_OK; // No parameters to validate for this command
        case STOP_STREAMING_ALL:
            return ESP_OK; // No parameters to validate for this command
        case START_IMU_STREAMING:
            return ESP_OK; // No parameters to validate for this command
        case STOP_IMU_STREAMING:
            return ESP_OK; // No parameters to validate for this command
        case START_EEG_MIC_STREAMING:
            return ESP_OK; // No parameters to validate for this command
        case STOP_EEG_MIC_STREAMING:
            return ESP_OK; // No parameters to validate for this command
        case SET_DEVICE_SETTINGS:
            return ESP_OK; // No parameters to validate for this command
        case START_EMG_STREAMING:
            return ESP_OK; // No parameters to validate for this command
        case STOP_EMG_STREAMING:
            return ESP_OK; // No parameters to validate for this command
        case START_DUMMY_STREAMING:
            return ESP_OK; // No parameters to validate for this command
        case STOP_DUMMY_STREAMING:
            return ESP_OK; // No parameters to validate for this command
        case ESP_STOP_COMMAND:
            return ESP_OK; // No parameters to validate for this command

        default:
            return ESP_ERR_INVALID_ARG; // Invalid command code
    }

}