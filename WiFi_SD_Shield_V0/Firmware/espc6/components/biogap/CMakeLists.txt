
file(GLOB_RECURSE COMMON_SOURCES
    "source/*.c"
)
idf_component_register(
    SRCS ${COMMON_SOURCES}
    INCLUDE_DIRS "include"
    REQUIRES esp_wifi esp_event nvs_flash freertos esp_timer common led_strip esp_driver_spi esp_mm 
)
