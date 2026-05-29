#include "common.h"
#include "esp_log.h"
#include "biogap.h"
#include "sd_main.h"

static const char *COMMON_TAG = "[common.c]: ";




// Track current SPI bus mode for safe mode switching
spi_mode_t current_spi_mode = SPI_MODE_IDLE;

/* Initialize ESP as SPI Slave, NRF will be the master */
esp_err_t init_nrf_spi_master_esp_slave_bus(void)
{   

    // Configuration for the SPI bus

    spi_bus_config_t buscfg = {
        .mosi_io_num = NRF_SPI_MOSI_GPIO,
        .miso_io_num = NRF_SPI_MISO_GPIO,
        .sclk_io_num = NRF_SPI_SCLK_GPIO,
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
        .spics_io_num = NRF_SPI_CS_GPIO,
        .queue_size = 10,            // sets how many transactions can be in the air. Must be higher for larger transaction intervals
        .flags = 0
        // .post_setup_cb = my_post_setup_cb,
        // .post_trans_cb = my_post_trans_cb
    };

    // Initialize SPI slave interface
    ESP_LOGW(COMMON_TAG, ">>> Initializing SPI SLAVE on host %d (DMA disabled, Mode 1)", SPI_HOST_DEVICE);
    ESP_LOGW(COMMON_TAG, ">>> MOSI=%d MISO=%d SCLK=%d CS=%d", NRF_SPI_MOSI_GPIO, NRF_SPI_MISO_GPIO, NRF_SPI_SCLK_GPIO, NRF_SPI_CS_GPIO);
    // DMA is now enabled 
    esp_err_t ret = spi_slave_initialize(SPI_HOST_DEVICE, &buscfg, &slvcfg, SPI_DMA_CH_AUTO);
    if (ret != ESP_OK) {
        ESP_LOGE(COMMON_TAG, ">>> FAILED to initialize NRF SPI slave: %s", esp_err_to_name(ret));
        return ret;
    }


    return ESP_OK;
}

/** @brief Initialize the pin multiplexer. Used for Wi-Fi Shield Hardware version V0.
 * This configures the MUX_SEL pin as input+output with pull-up, and sets it HIGH by default to select the SD card SPI path. 
 * The pin can be toggled LOW to switch to the NRF SPI path when needed.
*/
void pin_mux_init(){
    // Configure as input+output so readback reflects the pad while we drive it.
    // Keep internal pull-up enabled so default idle state is HIGH.
    gpio_config_t io_conf = {
        .pin_bit_mask = (1ULL << MUX_SEL),
        // gpio_get_level() reads the input path, so input must be enabled.
        .mode = GPIO_MODE_INPUT_OUTPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };

    ESP_ERROR_CHECK(gpio_reset_pin(MUX_SEL));
    ESP_ERROR_CHECK(gpio_config(&io_conf));

    // Default state: HIGH -> to write to SD card
    ESP_ERROR_CHECK(gpio_set_level(MUX_SEL, 1));
}



esp_err_t config_spi_nrf_master_esp_slave_drdy_pin(void){
    esp_err_t ret;
    // Configure the Pin as Output
    gpio_config_t io_conf = {
        .pin_bit_mask = (1ULL << NRF_DATA_READY_GPIO),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,  
        .pull_down_en = GPIO_PULLDOWN_ENABLE, 
        .intr_type = GPIO_INTR_DISABLE, 
    };

    ESP_ERROR_CHECK(gpio_reset_pin(NRF_DATA_READY_GPIO));
    ESP_ERROR_CHECK(gpio_config(&io_conf));

    // Default state: LOW 
    ret = gpio_set_level(NRF_DATA_READY_GPIO, 0);
    if (ret!=ESP_OK){
        ESP_LOGE(COMMON_TAG, "Failed to set NRF data-ready pin LOW: %s", esp_err_to_name(ret));
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

esp_err_t biogap_read_nrf_master_esp_slave_hw_init(void)
{
    /* Function to Initialize SPI Master and DRDY pin for NRF to ESP Transaction*/
    esp_err_t ret = ESP_OK;
    #if ENABLE_SPI_PROFILE_LOGS
        int64_t timer_start = esp_timer_get_time();
    #endif
    if (current_spi_mode != SPI_MODE_NRF) {
        ret = init_nrf_spi_master_esp_slave_bus();
        if (ret != ESP_OK) {
            ESP_LOGE(COMMON_TAG, "Failed to initialize NRF SPI bus: %s", esp_err_to_name(ret));
            return ret;
        }
        current_spi_mode = SPI_MODE_NRF;
    } 
    else {
        ESP_LOGW(COMMON_TAG, "NRF SPI bus already initialized, skipping re-init");
    }
    #if ENABLE_SPI_PROFILE_LOGS
        int64_t timer_stop = esp_timer_get_time();
        ESP_LOGI(COMMON_TAG, "SPI bus initialization time: %lld us", (long long)(timer_stop - timer_start));
    #endif


    
    ret = config_spi_nrf_master_esp_slave_drdy_pin();
    if (ret != ESP_OK) {
        ESP_LOGE(COMMON_TAG, "Failed to configure NRF data-ready pin: %s", esp_err_to_name(ret));
        return ret;
    }


    ESP_LOGI(COMMON_TAG,
             "Initialized NRF SPI bus: MOSI=%d MISO=%d SCLK=%d CS=%d @ %d Hz (Mode: %d)",
             NRF_SPI_MOSI_GPIO,
             NRF_SPI_MISO_GPIO,
             NRF_SPI_SCLK_GPIO,
             NRF_SPI_CS_GPIO,
             NRF_SPI_CLOCK_HZ,
             current_spi_mode);
    return ESP_OK;}

// ============================================================================
// SPI Mode Switching: Runtime transitions between NRF and SD card modes. 
// NRF is SPI master
// ============================================================================

esp_err_t switch_to_nrf_master_spi_mode(void)
{
    esp_err_t ret;
    ESP_LOGI(COMMON_TAG, "Switching to NRF SPI mode (current mode: %d)", current_spi_mode);
    #if ENABLE_SPI_PROFILE_LOGS
        int64_t timer_start = esp_timer_get_time();
    #endif
    // If already in NRF mode and the device handle is valid, nothing to do.
    if (current_spi_mode == SPI_MODE_NRF && nrf_spi_device != NULL) {
        ESP_LOGW(COMMON_TAG, "Already in NRF mode, skipping switch");
        return ESP_OK;
    }
    
    // If in SD mode, properly deinitialize SD resources first
    if (current_spi_mode == SPI_MODE_SD) {
        
        // Unmount FAT filesystem first - this releases the SD device from the filesystem layer
        ret = ESP_OK;
        if (card != NULL) {
            ESP_LOGI(COMMON_TAG, "Unmounting FAT filesystem before switching SPI mode");
            ret = esp_vfs_fat_sdcard_unmount(mount_point, card);
            if (ret == ESP_OK) {
                card = NULL;
            } else if (ret == ESP_ERR_INVALID_ARG) {
                ESP_LOGW(COMMON_TAG, "Skipping stale FAT unmount state: %s", esp_err_to_name(ret));
                card = NULL;
                ret = ESP_OK;
            } else {
                ESP_LOGE(COMMON_TAG, "Failed to unmount FAT filesystem: %s", esp_err_to_name(ret));
                // Continue anyway - try to free the bus regardless
            }
        }
        
        // Now free the SPI bus - unmount should have released SDSPI device
        ret = spi_bus_free(SPI_HOST_DEVICE);
        if (ret != ESP_OK) {
            ESP_LOGE(COMMON_TAG, "Failed to free SD SPI bus: %s", esp_err_to_name(ret));
            return ret;
        }
    }
    
    // Set MUX to NRF mode (LOW = 0)
    ESP_ERROR_CHECK(gpio_set_level(MUX_SEL, 0));
    ESP_LOGI(COMMON_TAG, "MUX_SEL set to 0 (NRF mode)");
    
    gpio_reset_pin(NRF_SPI_MOSI_GPIO);
    gpio_reset_pin(NRF_SPI_MISO_GPIO);
    gpio_reset_pin(NRF_SPI_SCLK_GPIO);
    // Small delay to let signal settle
    esp_rom_delay_us(100);
    
    ret = init_nrf_spi_master_esp_slave_bus();
    if (ret != ESP_OK) {
        ESP_LOGE(COMMON_TAG, "Failed to initialize NRF SPI bus: %s", esp_err_to_name(ret));
        return ret;
    }

    // Clear any pending task notifications queued during mode switch
    if (read_from_biogap_task_nrf_master_pq_esp_slave_handle != NULL) {
        ulTaskNotifyTake(pdFALSE, 0);  // Non-blocking drain
    }

#if ENABLE_SPI_PROFILE_LOGS
    int64_t timer_stop = esp_timer_get_time();
    ESP_LOGI(COMMON_TAG, "NRF SPI bus initialization time: %lld us", (long long)(timer_stop - timer_start));
#endif
    current_spi_mode = SPI_MODE_NRF;
    ESP_LOGI(COMMON_TAG, "Successfully switched to NRF SPI mode");
    return ESP_OK;
}

esp_err_t switch_to_sd_spi_mode(void)
{
   
    ESP_LOGI(COMMON_TAG, "Switching to SD SPI mode (current mode: %d)", current_spi_mode);
    
    // If already in SD mode, nothing to do
    if (current_spi_mode == SPI_MODE_SD) {
        ESP_LOGW(COMMON_TAG, "Already in SD mode, skipping switch");
        return ESP_OK;
    }
#if ENABLE_SPI_PROFILE_LOGS
    int64_t timer_start = esp_timer_get_time();
#endif
    

    // If in NRF mode, deinitialize NRF bus first
    if (current_spi_mode == SPI_MODE_NRF) {
        // Disable DRDY interrupt during mode switch to prevent bouncing issues
        ESP_LOGI(COMMON_TAG, "Disabling DRDY interrupt for mode switch");
        gpio_intr_disable(NRF_DATA_READY_GPIO);
        
        //ESP_LOGI(COMMON_TAG, "Removing NRF device and freeing SPI bus before switching to SD");
        if (nrf_spi_device != NULL) {
            esp_err_t ret = spi_bus_remove_device(nrf_spi_device);
            if (ret != ESP_OK) {
                ESP_LOGE(COMMON_TAG, "Failed to remove NRF device: %s", esp_err_to_name(ret));
                return ret;
            }
            nrf_spi_device = NULL;
        }
        
        esp_err_t ret = spi_bus_free(SPI_HOST_DEVICE);
        if (ret != ESP_OK) {
            ESP_LOGE(COMMON_TAG, "Failed to free NRF SPI bus: %s", esp_err_to_name(ret));
            return ret;
        }
    }
    
    // Set MUX to SD mode (HIGH = 1) with interrupt disabled
    ESP_ERROR_CHECK(gpio_set_level(MUX_SEL, 1));
    ESP_LOGI(COMMON_TAG, "MUX_SEL set to 1 (SD mode)");
    
    // Small delay to let signal settle
    esp_rom_delay_us(100);
    
    ESP_LOGI(COMMON_TAG, "Initializing SD SPI bus with SD pin configuration");
    // Reinitialize bus for SD with SD pin configuration
    spi_bus_config_t bus_cfg = {
        .mosi_io_num = PIN_NUM_MOSI,
        .miso_io_num = PIN_NUM_MISO,
        .sclk_io_num = PIN_NUM_CLK,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
        .max_transfer_sz = 4000,
    };
    
    esp_err_t ret = spi_bus_initialize(SPI_HOST_DEVICE, &bus_cfg, SPI_DMA_CH_AUTO);
    if (ret != ESP_OK) {
        ESP_LOGE(COMMON_TAG, "Failed to reinitialize SD SPI bus: %s", esp_err_to_name(ret));
        return ret;
    }
    
    slot_config.gpio_cs = PIN_NUM_CS;
    slot_config.host_id = SPI_HOST_DEVICE;
    
    current_spi_mode = SPI_MODE_SD;
#if ENABLE_SPI_PROFILE_LOGS
    int64_t timer_stop = esp_timer_get_time();
    ESP_LOGI(COMMON_TAG, "SD SPI bus initialization time: %lld us", (long long)(timer_stop - timer_start));
#endif
    ESP_LOGI(COMMON_TAG, "Switched to SD SPI mode");
    return ESP_OK;
}