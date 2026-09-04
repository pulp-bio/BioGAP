# Copyright (c) 2024 ETH Zurich
# SPDX-License-Identifier: Apache-2.0

cmake_minimum_required(VERSION 3.20.0)

# mmWave radar shield (Infineon BGT60TR13C).
#
# The radar is enabled like any other sensor, with CONFIG_SENSOR_MMWAVE in
# prj.conf. It needs two things that Kconfig cannot provide, though, because
# shield and devicetree selection happen here -- before find_package(Zephyr)
# evaluates Kconfig: its shield definition (which declares the bgt60 node) and
# overlays/mmwave.overlay (which frees the radar's IRQ/RST pins by disabling the
# PDM microphone that shares them).
#
# So prj.conf is scanned for the option here and the two pieces are pulled in to
# match. That keeps a single switch in one obvious place instead of requiring a
# -D flag on the command line to be kept in sync with a Kconfig value.
file(READ "${CMAKE_CURRENT_SOURCE_DIR}/prj.conf" MMWAVE_PRJ_CONF_TEXT)
# Leading newline so the first line matches the same pattern as every other one.
# Requiring the newline is what makes "# CONFIG_SENSOR_MMWAVE=y" not count.
set(MMWAVE_PRJ_CONF_TEXT "\n${MMWAVE_PRJ_CONF_TEXT}")
set(MMWAVE_FROM_PRJ_CONF OFF)
if(MMWAVE_PRJ_CONF_TEXT MATCHES "\n[ \t]*CONFIG_SENSOR_MMWAVE[ \t]*=[ \t]*y")
    set(MMWAVE_FROM_PRJ_CONF ON)
endif()

# Escape hatch, mainly for CI: force the shield on without editing prj.conf.
# Deliberately a different variable from the scan result above, so this does not
# depend on how option() and a pre-existing normal variable interact (CMP0077).
option(MMWAVE_SHIELD "Force the SENSEI mmWave radar shield on" OFF)

# Radar-only image: the radar is driven through the Zephyr SPI subsystem and
# owns SPIM4, reproducing the standalone BGT60TR13C firmware. ExG acquisition
# does not work in this build (see overlays/mmwave_only.conf). Implies
# MMWAVE_SHIELD, and overrides CONFIG_SENSOR_EEG/EMG from prj.conf.
#
#   west build -- -DMMWAVE_ONLY=ON
option(MMWAVE_ONLY "Build a radar-only image (Zephyr SPI, no working ExG)" OFF)

# Enable shields
set(SHIELD "SENSEI_ExGShield SENSEI_PPGShield SENSEI_WiFi_SD_Shield_V1")

if(MMWAVE_FROM_PRJ_CONF OR MMWAVE_SHIELD OR MMWAVE_ONLY)
    string(APPEND SHIELD " SENSEI_mmWaveShield")
    list(APPEND EXTRA_DTC_OVERLAY_FILE "${CMAKE_CURRENT_SOURCE_DIR}/overlays/mmwave.overlay")

    # The ExG/PPG/WiFi shields stay in SHIELD even for a radar-only image: the
    # unconditionally built afe/ and bsp/power sources reference their
    # devicetree nodes, so dropping them would break the build rather than just
    # disable ExG. Kconfig is what turns ExG off.
    if(MMWAVE_ONLY)
        list(APPEND EXTRA_CONF_FILE "${CMAKE_CURRENT_SOURCE_DIR}/overlays/mmwave_only.conf")
        message(STATUS "mmWave radar-only image (Zephyr SPI; ExG disabled)")
    else()
        list(APPEND EXTRA_CONF_FILE "${CMAKE_CURRENT_SOURCE_DIR}/overlays/mmwave.conf")
        message(STATUS "mmWave radar enabled (shared SPI_A with ExG; PDM microphone disabled)")
    endif()
endif()

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

list(APPEND EXTRA_ZEPHYR_MODULES "${SENSEI_SDK_ROOT}/NRF")

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
    core/command_dispatcher.c

    # BSP
    bsp/pwr_bsp.c
    bsp/power/power.c
    bsp/system_status/system_status.c

    # SPI (shared SPI_A bus -- ADS1298 and WiFi/SD shield both sit on it)
    spi/spi_a.c

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

    # Sensors - EEG
    sensors/eeg/eeg_appl.c

    # Sensors - EMG
    sensors/emg/emg_appl.c

    # BLE
    ble/ble_appl.c
    ble/bluetooth.c
)

# Sensors - Microphone. mic_appl.c resolves the PDM device with
# DEVICE_DT_GET(), so it only links when the devicetree node is enabled. The
# mmWave build gives the microphone's CLK/DATA pins (P0.04/P0.12) to the radar
# and disables &pdm0, and gets the reject-at-runtime variant instead, which
# keeps the command dispatcher free of conditional compilation.
if(CONFIG_AUDIO_DMIC)
    target_sources(app PRIVATE
        sensors/mic/mic_appl.c
    )
else()
    target_sources(app PRIVATE
        sensors/mic/mic_unavailable.c
    )
endif()

# WULPUS ultrasound sensor (MSP430 bridge via SPI_B)
if(CONFIG_SENSOR_WULPUS)
    target_sources(app PRIVATE
        sensors/wulpus/wulpus_appl.c
    )
endif()

# mmWave radar (BGT60TR13C). Shares SPI_A (spi/spi_a.c, unconditional above)
# with the ADS1298 and the WiFi/SD shield.
if(CONFIG_SENSOR_MMWAVE)
    target_sources(app PRIVATE
        sensors/mmWave/mmWave_appl.c
        sensors/mmWave/driver/xensiv_bgt60trxx.c
    )

    # Two interchangeable SPI transports behind the same mmWave_spi.h API:
    # the Zephyr SPI subsystem (radar owns SPIM4, reproduces the standalone
    # reference firmware) or the raw nrfx SPI_A layer (radar shares the bus
    # with the ADS1298 and the WiFi/SD shield).
    if(CONFIG_MMWAVE_ZEPHYR_SPI)
        target_sources(app PRIVATE sensors/mmWave/mmWave_spi_zephyr.c)
    else()
        target_sources(app PRIVATE sensors/mmWave/mmWave_spi.c)
    endif()
endif()

# Diagnostic: PMIC-noise injection sweep (see bsp/pmic_noise_test.c)
if(CONFIG_PMIC_NOISE_TEST)
    target_sources(app PRIVATE
        bsp/pmic_noise_test.c
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

# WiFi/SD shield (ESP32-C6): alternative to BLE for streaming sensor data.
# Shares SPI_A (spi/spi_a.c) with the ADS1298. The command dispatcher itself
# (core/command_dispatcher.c) is unconditional above, since BLE routes
# through it too.
if(CONFIG_WI_FI)
    target_sources(app PRIVATE
        wifi_sd_shield/wifi_sd_shield_appl.c
        wifi_sd_shield/wifi_sd_shield_inits.c
        wifi_sd_shield/wifi_sd_spi_functions.c
    )
endif()

# Dummy sensor (synthetic data, no hardware required): sends over BLE or
# the WiFi shield depending on CONFIG_WI_FI, handled internally by the file.
if(CONFIG_DUMMY_SENSOR)
    target_sources(app PRIVATE
        sensors/dummy_sensor/dummy_sensor_appl.c
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
