#include <string.h>
#include <stdint.h>
#include "esp_err.h"
#include "esp_log.h"
#include <sys/unistd.h>
#include <sys/stat.h>
#include "esp_vfs_fat.h"
#include "common.h"  // For SPI mode switching functions



#define EXAMPLE_MAX_CHAR_SIZE    64


#define MOUNT_POINT "/sdcard"
extern char mount_point[];



#ifdef __cplusplus
extern "C" {
#endif

typedef struct{
    const char** names;
    const int* pins;
} pin_configuration_t;
void check_sd_card_pins(pin_configuration_t *config, const int pin_count);


extern sdmmc_host_t host;
extern spi_bus_config_t bus_cfg;
extern sdspi_device_config_t slot_config;
extern sdmmc_card_t *card;

esp_err_t sd_card_write(const char *file_name, const uint8_t *data, size_t len); 
esp_err_t sd_card_init();
esp_err_t write_hello_word();
esp_err_t write_debug_message(const char *msg);
void sd_card_task(void *pv);



#ifdef __cplusplus
}
#endif