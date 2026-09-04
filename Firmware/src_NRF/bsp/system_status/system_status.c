/*
 * ----------------------------------------------------------------------
 *
 * File: system_status.c
 *
 * Last edited: 05.01.2026
 *
 * Copyright (C) 2026, ETH Zurich and University of Bologna.
 *
 * ----------------------------------------------------------------------
 * SPDX-License-Identifier: Apache-2.0
 */

#include "bsp/system_status/system_status.h"
#include "sensors/imu/imu_appl.h"
#include "core/connectivity_commands.h"
#include "ble/bluetooth.h"
#include "core/common.h"
#include "afe/ads_defs.h"
#include "pwr/pwr.h"
#include <stdbool.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#if defined(CONFIG_WI_FI)
#include "wifi_sd_shield/wifi_sd_shield_appl.h"
#endif

LOG_MODULE_REGISTER(system_status, LOG_LEVEL_INF);

/* IMU die temperature, cached so status requests hit the (shared) I2C bus
 * at most once per TEMP_CACHE_MS. Returns true if the value is valid. */
#define TEMP_CACHE_MS 2000

static bool get_temperature(float *out) {
    static float cached;
    static bool valid = false;
    static int64_t last_read_ms = -TEMP_CACHE_MS;

    if (k_uptime_get() - last_read_ms >= TEMP_CACHE_MS) {
        valid = (imu_read_temperature(&cached) == 0);
        last_read_ms = k_uptime_get();
    }
    if (valid) {
        *out = cached;
    }
    return valid;
}

static void put_u16_le(uint8_t *p, uint16_t v) {
    p[0] = (uint8_t)(v & 0xFF);
    p[1] = (uint8_t)(v >> 8);
}

int system_status_init(void) {
    LOG_INF("System Status aggregator initialized");
    return 0;
}

int system_status_build_packet(uint8_t *buffer, size_t buf_size, size_t *out_len) {
    if (buf_size < 7) {
        return -1; // Buffer too small for standard 7-byte status packet
    }

    // All battery/charger values come from the SDK power-thread cache:
    struct pwr_status pwr;
    pwr_get_status(&pwr);

    float temp_celsius = 0.0f;
    bool temp_valid = get_temperature(&temp_celsius);

    /* Layout as parsed by the legacy BioWolf GUI (RefreshRTDeviceInfo in
     * FXMLDocumentController.java): [1] charging flag, [2] current in mA,
     * [3] power in mW, [4] SoC in %, [5] voltage in 0.1 V units (the GUI
     * renders 41 as "4.1v"), [6] temperature in degC. The original BioWolf
     * sourced these from a BQ27441 fuel gauge; here they come from the
     * MAX77654 telemetry. */
    uint16_t current_mA = pwr.batt_dmA / 10;
    uint16_t power_mW = pwr.chgin_present ? pwr.chgin_power_mW : pwr.batt_power_mW;

    buffer[0] = REQUEST_BATTERY_STATE;
    buffer[1] = pwr.charging ? 1 : 0;
    buffer[2] = (uint8_t)MIN(current_mA, 255);
    buffer[3] = (uint8_t)MIN(power_mW, 255);
    buffer[4] = pwr.batt_perc;
    buffer[5] = (uint8_t)((pwr.batt_mV + 50) / 100); // deci-volts: 4120 mV -> 41 -> "4.1v"
    buffer[6] = temp_valid ? (uint8_t)temp_celsius : 0;

    *out_len = 7;
    return 0;
}

int system_status_build_system_status_packet(uint8_t *buffer, size_t buf_size, size_t *out_len) {
    if (buf_size < SYSTEM_STATUS_PACKET_LEN) {
        return -1;
    }

    struct pwr_status pwr;
    pwr_get_status(&pwr);

    float temp_celsius = 0.0f;
    bool temp_valid = get_temperature(&temp_celsius);

    uint8_t flags = 0;
    flags |= pwr.chgin_present ? SYSTEM_STATUS_FLAG_CHGIN_PRESENT : 0;
    flags |= pwr.charging ? SYSTEM_STATUS_FLAG_CHARGING : 0;
    flags |= pwr.chg_fault ? SYSTEM_STATUS_FLAG_CHG_FAULT : 0;
    flags |= pwr.thermal_alarm ? SYSTEM_STATUS_FLAG_THERMAL_ALARM : 0;
    flags |= temp_valid ? SYSTEM_STATUS_FLAG_TEMP_VALID : 0;
    /* The power thread reports 0 mV when the BATT pin reads implausibly
     * high, i.e. no battery is attached (a measured battery is never 0): */
    flags |= (pwr.batt_mV > 0) ? SYSTEM_STATUS_FLAG_BATT_PRESENT : 0;

    buffer[0] = REQUEST_SYSTEM_STATUS;
    buffer[1] = flags;
    buffer[2] = pwr.chg_details;
    buffer[3] = pwr.batt_perc;
    put_u16_le(&buffer[4], pwr.batt_mV);
    put_u16_le(&buffer[6], pwr.vsys_mV);
    put_u16_le(&buffer[8], pwr.chgin_mV);
    put_u16_le(&buffer[10], pwr.chgin_dmA);
    put_u16_le(&buffer[12], pwr.batt_dmA);
    put_u16_le(&buffer[14], pwr.chgin_power_mW);
    put_u16_le(&buffer[16], pwr.batt_power_mW);
    put_u16_le(&buffer[18], (uint16_t)(int16_t)(temp_valid ? temp_celsius * 100.0f : 0.0f));
    buffer[20] = BLE_PCK_TAILER;

    *out_len = SYSTEM_STATUS_PACKET_LEN;
    return 0;
}

/*==============================================================================
 * Device Information Functions
 *============================================================================*/

void system_status_send_hardware_version(void) {
    uint8_t data[4];
    data[0] = REQUEST_HARDWARE_VERSION;
    data[1] = HARDWARE_VERSION;
    data[2] = HARDWARE_REVISION;
    data[3] = BLE_PCK_TAILER;
#if defined(CONFIG_WI_FI)
    add_data_to_esp_send_buffer(data, sizeof(data));
#else
    send_data_ble(data, sizeof(data));
#endif
}

void system_status_send_firmware_version(void) {
    uint8_t data[4];
    data[0] = REQUEST_FIRMWARE_VERSION;
    data[1] = FIRMWARE_VERSION;
    data[2] = FIRMWARE_REVISION;
    data[3] = BLE_PCK_TAILER;
#if defined(CONFIG_WI_FI)
    add_data_to_esp_send_buffer(data, sizeof(data));
#else
    send_data_ble(data, sizeof(data));
#endif
}

void system_status_send_available_sensors(void) {
    uint8_t data[4];
    data[0] = REQUEST_AVAILABLE_SENSORS;
    data[1] = true;
    data[2] = 0;
    data[3] = BLE_PCK_TAILER;
#if defined(CONFIG_WI_FI)
    add_data_to_esp_send_buffer(data, sizeof(data));
#else
    send_data_ble(data, sizeof(data));
#endif
}

void system_status_send_device_settings(void) {
    // Not implemented yet
}

void system_status_send_ready(void) {
    uint8_t ready[5] = {'B', 'W', 'F', '1', '6'};
#if defined(CONFIG_WI_FI)
    add_data_to_esp_send_buffer(ready, sizeof(ready));
#else
    send_data_ble(ready, sizeof(ready));
#endif
}

/*==============================================================================
 * Board State Management
 *============================================================================*/

static int8_t board_current_state = STATE_STREAMING_NORDIC;

void system_status_set_board_state(int8_t state) {
    board_current_state = state;
}

int8_t system_status_get_board_state(void) {
    return board_current_state;
}
