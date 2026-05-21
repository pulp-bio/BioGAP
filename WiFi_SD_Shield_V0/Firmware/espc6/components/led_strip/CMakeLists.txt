file(GLOB_RECURSE LED_STRIP_SOURCES
    "led_app.c"
    "source/*.c"
)

idf_component_register(
    SRCS ${LED_STRIP_SOURCES}
    INCLUDE_DIRS "include"
    REQUIRES driver
)
