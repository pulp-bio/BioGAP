/* SD card and FAT filesystem example.
Adapted from ESP32-C6 example
*/

#include "sd_main.h"
#include "common.h"
#include "biogap.h"
#include "sdmmc_cmd.h"

static const char *SD_TAG = "[sd_card.c]";
sdmmc_host_t host = SDSPI_HOST_DEFAULT();
sdspi_device_config_t slot_config = SDSPI_DEVICE_CONFIG_DEFAULT();
sdmmc_card_t *card = NULL;
char mount_point[] = MOUNT_POINT;
static const char *file_name = MOUNT_POINT "/bla.txt";

// Temporary accumulation buffer for one SD flush chunk
static uint8_t sd_chunk_buffer[SD_CARD_TRANSFER_SIZE];


esp_err_t mounting_fat(void)
{
    

    esp_vfs_fat_sdmmc_mount_config_t mount_config = {
        .format_if_mount_failed = true,
        .max_files = 5,
        .allocation_unit_size = 16 * 1024
    };



    esp_err_t ret = esp_vfs_fat_sdspi_mount(
        mount_point,
        &host,
        &slot_config,
        &mount_config,
        &card
    );

    if (ret != ESP_OK) {
        if (ret == ESP_FAIL) {
            ESP_LOGE(SD_TAG, "Failed to mount filesystem");
        } else {
            ESP_LOGE(SD_TAG, "Failed to initialize the card (%s)", esp_err_to_name(ret));
        }
        return ret;
    }

    ESP_LOGI(SD_TAG, "Filesystem mounted");
    return ESP_OK;
}

static esp_err_t ensure_sd_ready(void)
{
    esp_err_t ret = ESP_OK;

    if (current_spi_mode != SPI_MODE_SD) {
        ret = switch_to_sd_spi_mode();
        if (ret != ESP_OK) {
            ESP_LOGE(SD_TAG, "Failed to switch to SD SPI mode: %s", esp_err_to_name(ret));
            return ret;
        }
    }

    if (card == NULL) {
        ret = mounting_fat();
        if (ret != ESP_OK) {
            ESP_LOGE(SD_TAG, "Failed to mount FAT filesystem: %s", esp_err_to_name(ret));
            return ret;
        }
    }

    return ESP_OK;
}

esp_err_t sd_card_init(){
    /* Function to initialize SD card SPI bus */
    esp_err_t ret;

    ret = switch_to_sd_spi_mode();
    if (ret != ESP_OK) {
        ESP_LOGE(SD_TAG, "Failed to initialize SD card SPI: %s", esp_err_to_name(ret));
        return ret;
    }
    //gpio_set_pull_mode(PIN_NUM_CS, GPIO_PULLUP_ONLY);


    ret = mounting_fat(); 
    if (ret != ESP_OK) {
        ESP_LOGE(SD_TAG, "Failed to mount FAT filesystem: %s", esp_err_to_name(ret));
        return ret;
    }
    else{
        ESP_LOGI(SD_TAG, "FAT correctly mounted");
    }
    ESP_LOGI(SD_TAG, "FAT correctly mounted");
    sdmmc_card_print_info(stdout, card);
    
    return ESP_OK;
}


esp_err_t sd_card_write_marker(const char *file_name, const char *marker)
{
    FILE *f = fopen(file_name, "ab");
    if (f == NULL) {
        ESP_LOGE(SD_TAG, "Failed to open file for append");
        return ESP_FAIL;
    }

    size_t len = strlen(marker);
    size_t written = fwrite(marker, 1, len, f);
    fflush(f);
    fclose(f);

    if (written != len) {
        ESP_LOGE(SD_TAG, "Failed to write marker");
        return ESP_FAIL;
    }

    return ESP_OK;
}

esp_err_t sd_card_write(const char *file_name, const uint8_t *data, size_t len)
{
    //ESP_LOGI(SD_TAG, "Writing to SD card, data length: %u", (unsigned)len);
    esp_err_t ret; 
    // First, set bit to notify that transaction from NRF is not possible while writing to SD card
    xEventGroupSetBits(g_evt, B_WRITING_TO_SD);
    
    // Ensure SD SPI mode is active and FAT is mounted before file I/O.
    ret = ensure_sd_ready();
    if (ret != ESP_OK) {
        ESP_LOGE(SD_TAG, "SD path is not ready for write: %s", esp_err_to_name(ret));
        xEventGroupClearBits(g_evt, B_WRITING_TO_SD);
        return ret;
    }

    // UNCOMMENT when physicall connecting SD card SD card 
    
    FILE *f = fopen(file_name, "ab");
    if (f == NULL) {
        ESP_LOGE(SD_TAG, "Failed to open file for append");
        xEventGroupClearBits(g_evt, B_WRITING_TO_SD);
        return ESP_FAIL;
    }

    size_t written = fwrite(data, 1, len, f);
    // print some data
    ESP_LOGI(SD_TAG, "First byte written to sd card is: 0x%02x", data[0]);
    fflush(f);
    fclose(f);

    if (written != len) {
        ESP_LOGE(SD_TAG, "fwrite wrote %u/%u bytes",
                 (unsigned)written,
                 (unsigned)len);
        xEventGroupClearBits(g_evt, B_WRITING_TO_SD);
        return ESP_FAIL;
    }
    ESP_LOGI(SD_TAG, "Wrote %u bytes to SD CARD, transfers done %u", (unsigned)written, sd_writecounter);
    

    // After writing, switch back to NRF SPI mode
    ESP_LOGI(SD_TAG, "Finished writing to SD card, switching back to NRF SPI mode");
    ret = switch_to_nrf_master_spi_mode();
    if (ret != ESP_OK) {
        ESP_LOGE(SD_TAG, "Failed to switch back to NRF SPI mode: %s", esp_err_to_name(ret));
    }
    
    xEventGroupClearBits(g_evt, B_WRITING_TO_SD);
    return ret;
}

void sd_card_deinit(){
    // All done, unmount partition and disable SPI peripheral
    esp_vfs_fat_sdcard_unmount(mount_point, card);
    ESP_LOGI(SD_TAG, "Card unmounted");
    //deinitialize the bus after all devices are removed
    spi_bus_free(host.slot);
}

void sd_card_task(void *pv)
{
    esp_err_t ret;

    static uint8_t pending_item[SPI_FROM_BIOGAP_MAX_SIZE + PACKET_DEF_BYTES];
    static size_t pending_item_size = 0;
    static bool has_pending_item = false;

    size_t collected_bytes = 0;

    // Start with a clean chunk buffer
    memset(sd_chunk_buffer, 0, sizeof(sd_chunk_buffer));

    ESP_LOGI(SD_TAG, "sd_card_task started");

    while (1) {

        // If a previous item did not fit, start the new chunk with it
        if (has_pending_item) {
            memcpy(&sd_chunk_buffer[0], pending_item, pending_item_size);
            collected_bytes = pending_item_size;
            has_pending_item = false;
            pending_item_size = 0;

            // ESP_LOGI(SD_TAG,
            //          "Inserted pending item: chunk[0]=0x%02X, collected_bytes=%u",
            //          sd_chunk_buffer[0],
            //          (unsigned)collected_bytes);
        }

        size_t item_size = 0;
        uint8_t *item = (uint8_t *)xRingbufferReceive(ringbuff, &item_size, pdMS_TO_TICKS(1));

        if (item == NULL) {
            vTaskDelay(pdMS_TO_TICKS(1));
            continue;
        }

        if (item_size == 0) {
            ESP_LOGW(SD_TAG, "Received zero-length item from ringbuffer");
            vRingbufferReturnItem(ringbuff, (void *)item);
            continue;
        }

        // ESP_LOGI(SD_TAG,
        //          "Received item from ringbuffer, size: %u, header: 0x%02X",
        //          (unsigned)item_size,
        //          item[0]);

        // Sanity check: one item must fit into one SD chunk
        if (item_size > SD_CARD_TRANSFER_SIZE) {
            ESP_LOGE(SD_TAG,
                     "Item too large for SD chunk: item_size=%u, max_chunk=%u",
                     (unsigned)item_size,
                     (unsigned)SD_CARD_TRANSFER_SIZE);
            vRingbufferReturnItem(ringbuff, (void *)item);
            continue;
        }

        // If next item does not fit, flush current chunk first
        if (collected_bytes + item_size > SD_CARD_TRANSFER_SIZE) {

            // Save this item for next chunk
            memcpy(pending_item, item, item_size);
            pending_item_size = item_size;
            has_pending_item = true;

            vRingbufferReturnItem(ringbuff, (void *)item);
            //ESP_LOGI(SD_TAG,"Flushing chunk: chunk[0]=0x%02X,chunk[-1]=0x%02X, len=%u",sd_chunk_buffer[0], sd_chunk_buffer[collected_bytes - 1],(unsigned)collected_bytes);

            gpio_set_level(MUX_SEL, 1);

            #if ESP_ENABLE_SD_WRITE
                ESP_LOGI(SD_TAG, "Flushing SD chunk to file: %s, size: %u bytes", file_name, (unsigned)collected_bytes);
                ret = sd_card_write(file_name, sd_chunk_buffer, collected_bytes);
            #endif

            if (ret!=ESP_OK){
                ESP_LOGE(SD_TAG, "Failed to write chunk to SD card");
                gpio_set_level(MUX_SEL, 0);
                break; 
            }
            gpio_set_level(MUX_SEL, 0);
            ret = ESP_OK; 
            if (ret != ESP_OK) {
                ESP_LOGE(SD_TAG,
                         "Failed to write %u bytes to SD card",
                         (unsigned)collected_bytes);
            } else {
                sd_writecounter++;
                // ESP_LOGI(SD_TAG,
                //          "Successfully wrote %u bytes to SD card, total writes: %u",
                //          (unsigned)collected_bytes,
                //          sd_writecounter);

                collected_bytes = 0;
                memset(sd_chunk_buffer, 0, sizeof(sd_chunk_buffer));
            }
        } 
        else {
            // Append item to current SD chunk
            size_t append_offset = collected_bytes;
            memcpy(&sd_chunk_buffer[append_offset], item, item_size);

            // ESP_LOGI(SD_TAG,
            //          "After append: chunk[0]=0x%02X, appended_header=0x%02X, append_offset=%u",
            //          sd_chunk_buffer[0],
            //          sd_chunk_buffer[append_offset],
            //          (unsigned)append_offset);

            collected_bytes += item_size;

            vRingbufferReturnItem(ringbuff, (void *)item);
        }

        // Stop condition for test
        if (sd_writecounter >= MAX_SD_WRITES) {
            ESP_LOGW(SD_TAG,
                     "Reached max SD writes (%u), stopping acquisition",
                     MAX_SD_WRITES);
            xEventGroupSetBits(g_evt, B_END_ACQUISITION);

            if (read_from_biogap_task_nrf_master_pq_esp_slave_handle != NULL) {
                ESP_LOGI(SD_TAG,
                         "Reached max SD writes, signaling end of acquisition to BIOGAP reading task");
                xTaskNotifyGive(read_from_biogap_task_nrf_master_pq_esp_slave_handle);
            }

            // Flush any remaining buffered data before stopping
            if (collected_bytes > 0) {
                ESP_LOGI(SD_TAG,
                         "Final flush before stop: chunk[0]=0x%02X, len=%u",
                         sd_chunk_buffer[0],
                         (unsigned)collected_bytes);

                gpio_set_level(MUX_SEL, 1);
                ret = sd_card_write(file_name, sd_chunk_buffer, collected_bytes);
                gpio_set_level(MUX_SEL, 0);

                if (ret != ESP_OK) {
                    ESP_LOGE(SD_TAG,
                             "Failed final write of %u bytes to SD card",
                             (unsigned)collected_bytes);
                } else {
                    sd_writecounter++;
                    ESP_LOGI(SD_TAG,
                             "Final write successful: %u bytes, total writes: %u",
                             (unsigned)collected_bytes,
                             sd_writecounter);
                }
            }

            sd_card_deinit();
            break;
        }
    }
    xEventGroupSetBits(g_evt, B_END_ACQUISITION);
    ESP_LOGI(SD_TAG, "sd_card_task finished");
    vTaskDelete(NULL);
}


esp_err_t write_hello_word(void){
    return write_debug_message("hello word\n");
}

esp_err_t write_debug_message(const char *msg)
{
    if (msg == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    esp_err_t ret = ensure_sd_ready();
    if (ret != ESP_OK) {
        ESP_LOGE(SD_TAG, "SD path is not ready for debug message write: %s", esp_err_to_name(ret));
        return ret;
    }

    FILE *f = fopen(file_name, "ab");
    if (f == NULL) {
        ESP_LOGE(SD_TAG, "Failed to open %s for debug message write", file_name);
        return ESP_FAIL;
    }

    size_t msg_len = strlen(msg);
    size_t written = fwrite(msg, 1, msg_len, f);
    fflush(f);
    fclose(f);

    if (written != msg_len) {
        ESP_LOGE(SD_TAG, "Debug message write incomplete: %u/%u", (unsigned)written, (unsigned)msg_len);
        return ESP_FAIL;
    }

    ESP_LOGI(SD_TAG, "Debug message written to %s", file_name);
    return ESP_OK;
}