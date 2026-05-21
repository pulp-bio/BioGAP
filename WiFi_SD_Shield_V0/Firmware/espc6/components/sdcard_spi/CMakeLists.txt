# The following lines of boilerplate have to be in your project's CMakeLists
# in this exact order for cmake to work correctly
# list(APPEND EXTRA_COMPONENT_DIRS "$ENV{IDF_PATH}/examples/storage/sd_card/sdmmc/components/sd_card")
# set(COMPONENTS main)

# include($ENV{IDF_PATH}/tools/cmake/project.cmake)
# project(sdspi)


file(GLOB_RECURSE SDC_CARD_SOURCES
    "source/*.c"
)
idf_component_register(
    SRCS ${SDC_CARD_SOURCES}
    INCLUDE_DIRS "include"
    REQUIRES fatfs esp_adc common biogap
    WHOLE_ARCHIVE
)


