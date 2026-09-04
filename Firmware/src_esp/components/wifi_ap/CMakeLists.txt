file(GLOB_RECURSE WIFI_AP_SOURCES
    "source/*.c"
)

idf_component_register(
    SRCS ${WIFI_AP_SOURCES}
    INCLUDE_DIRS "include"
    REQUIRES esp_wifi esp_event driver common
)
