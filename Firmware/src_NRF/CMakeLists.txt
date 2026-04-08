# Copyright (c) 2024 ETH Zurich
# SPDX-License-Identifier: Apache-2.0

cmake_minimum_required(VERSION 3.20.0)

# Enable shields
set(SHIELD "SENSEI_ExGShield SENSEI_PPGShield")

set(CONF_FILE prj.conf)
set(OVERLAY_CONFIG "child_image/hci_ipc.conf")


# Setup SENSEI SDK
set(SENSEI_SDK_ROOT $ENV{SENSEI_SDK_ROOT})
if(NOT SENSEI_SDK_ROOT)
    message(FATAL_ERROR "SENSEI_SDK_ROOT is not defined")
endif()

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
    bsp/battery/battery.c
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
    sensors/imu/lis2duxs12_sensor.c
    sensors/imu/driver/lis2duxs12_reg.c

    # Sensors - Microphone
    sensors/mic/mic_appl.c

    # Sensors - PPG
    sensors/ppg/ppg_appl.c

    # Sensors - EEG
    sensors/eeg/eeg_appl.c

    # Sensors - EMG
    sensors/emg/emg_appl.c

    # BLE
    ble/ble_appl.c
    ble/bluetooth.c
)
target_include_directories(app PRIVATE
    .
)
