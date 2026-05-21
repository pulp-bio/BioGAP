idf_component_register(SRCS "main.c"
                    INCLUDE_DIRS "."
                    REQUIRES sdcard_spi biogap common led_strip)

