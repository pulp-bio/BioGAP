file(GLOB_RECURSE COMMON_SOURCES
    "source/*.c"
)

idf_component_register(
    SRCS ${COMMON_SOURCES}
    INCLUDE_DIRS "include"
    REQUIRES driver freertos esp_timer biogap sdcard_spi esp_driver_spi
)
