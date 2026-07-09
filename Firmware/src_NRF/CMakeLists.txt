# Copyright (c) 2024 ETH Zurich
# SPDX-License-Identifier: Apache-2.0

cmake_minimum_required(VERSION 3.20.0)

# Enable shields
set(SHIELD "SENSEI_ExGShield SENSEI_PPGShield")

set(CONF_FILE prj.conf)


# Setup SENSEI SDK
set(SENSEI_SDK_ROOT $ENV{SENSEI_SDK_ROOT})
if(NOT SENSEI_SDK_ROOT)
    message(FATAL_ERROR "SENSEI_SDK_ROOT is not defined")
endif()
message(STATUS "SENSEI_SDK_ROOT = ${SENSEI_SDK_ROOT}")

# Also emit the value at build time so it's visible in build logs
add_custom_target(print_sensei_sdk_root ALL
    COMMAND ${CMAKE_COMMAND} -E echo "SENSEI_SDK_ROOT=${SENSEI_SDK_ROOT}"
    COMMENT "Printing SENSEI_SDK_ROOT at build time"
)

list(APPEND EXTRA_ZEPHYR_MODULES
    ${SENSEI_SDK_ROOT}/NRF
)

# Configure the project
find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(exg_stream)


target_sources(app PRIVATE
    # Entry point
    main.c

    # Core
    core/sync_streaming.c
    core/i2c_helpers.c
    core/board_sync.c

    # BSP
    bsp/pwr_bsp.c
    bsp/power/power.c
    bsp/system_status/system_status.c

    # AFE (Analog Front-End - ADS1298)
    afe/ads_appl.c
    afe/ads_spi_hw.c
    afe/ads_spi_comm.c
    afe/ads_spi_config.c
    afe/ads_spi_data.c

    # Sensors - IMU
    sensors/imu/imu_appl.c
    sensors/imu/lsm6dsv16bx_sensor.c
    sensors/imu/driver/lsm6dsv16bx_reg.c

    # Sensors - Microphone
    sensors/mic/mic_appl.c

    # Sensors - EEG
    sensors/eeg/eeg_appl.c

    # Sensors - EMG
    sensors/emg/emg_appl.c

    # BLE
    ble/ble_appl.c
    ble/bluetooth.c
)

# WULPUS ultrasound sensor (MSP430 bridge via SPI_B)
if(CONFIG_SENSOR_WULPUS)
    target_sources(app PRIVATE
        sensors/wulpus/wulpus_appl.c
    )
endif()

# Multi-PPG sensor subsystem (MAXM86161 × N via TCA9548A)
if(CONFIG_SENSOR_PPG_NEW)
    target_sources(app PRIVATE
        bsp/i2c_mux/i2c_mux.c
        sensors/ppg_new/maxm86161_i2c.c
        sensors/ppg_new/maxm86161.c
        sensors/ppg_new/ppg_new_appl.c
    )
endif()

target_include_directories(app PRIVATE
    .
    # Needed so maxm86161.c and maxm86161.h can use bare #include "maxm86161_i2c.h"
    # etc. without full path prefixes.
    sensors/ppg_new
)

# SDK power-thread policy (the SDK pwr sources build as part of the app
# target, so plain target definitions reach them):
# - keep the LED rail off in pwr_start(): the LED is unused on BioGAP and no
#   non-essential power domain may come up as a side effect.
# - keep the SDK long-press pwr_kill() disabled: the soft-reset button is
#   already handled by the app (any press -> PMIC factory-ship in main.c),
#   and both shutdown paths would race.
target_compile_definitions(app PRIVATE
    CONFIG_PWR_START_LED_POWER=0
    CONFIG_PWR_LONG_PRESS_KILL=0
)
