/*
 * ----------------------------------------------------------------------
 *
 * File: battery.c
 *
 * Last edited: 02.01.2026
 *
 * Copyright (C) 2026, ETH Zurich and University of Bologna.
 *
 * ----------------------------------------------------------------------
 * SPDX-License-Identifier: Apache-2.0
 */

#include "bsp/battery/battery.h"
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>

#include "pwr/pwr_common.h"
#include "pwr/pwr.h"
#include "max77654.h"
#include "afe/ads_appl.h"

LOG_MODULE_REGISTER(battery_bsp, LOG_LEVEL_INF);

/* Define stack sizes and priorities */
#define BAT_UPDATE_STACK_SIZE    1024
#define BAT_UPDATE_PRIORITY      6

static uint32_t current_soc = 0;
static uint32_t current_bat_mv = 0;
static uint32_t current_power_mw = 0;
static bool is_charging = false;
static const char *current_power_source = NULL;

static void battery_update_thread(void *arg1, void *arg2, void *arg3)
{
    LOG_INF("Battery update thread started");
    k_sleep(K_MSEC(2000));

    while (1) {
        k_sleep(K_MSEC(5000));
        // Only update if not currently reading ADS data to avoid I2C/SPI interference if shared
        if (ads_get_function() != ADS_READ) {
            battery_update_status();
        }
    }
}

K_THREAD_DEFINE(battery_update_tid, BAT_UPDATE_STACK_SIZE, battery_update_thread, 
                NULL, NULL, NULL, BAT_UPDATE_PRIORITY, 0, 0);

int battery_init(void) {
    LOG_INF("Battery BSP initialized");
    return 0;
}

int battery_update_status(void) {
    // Get battery percentage and battery voltage
    current_soc = pwr_bat_perc();
    current_bat_mv = pwr_bat_mV();

    // Retrieve PMIC status
    struct max77654_stat stat;
    if (max77654_get_stat(&pmic_h, &stat) != E_MAX77654_SUCCESS) {
        LOG_ERR("Failed to get PMIC status");
        return -1;
    }

    if (stat.chgin_status != MAX77654_CHGIN_DTLS_UVLO) {
        current_power_source = "USB/External";
        is_charging = true;
        uint32_t chgin_v = 0, chgin_i = 0;
        if (max77654_measure(&pmic_h, MAX77654_CHGIN_V, &chgin_v) != E_MAX77654_SUCCESS) {
            chgin_v = 0;
        }
        if (max77654_measure(&pmic_h, MAX77654_CHGIN_I, &chgin_i) != E_MAX77654_SUCCESS) {
            chgin_i = 0;
        }
        current_power_mw = (chgin_v * chgin_i) / 1000;
    } else {
        current_power_source = "Battery";
        is_charging = false;
        uint32_t vsys_v = 0, batt_i = 0;
        if (max77654_measure(&pmic_h, MAX77654_VSYS, &vsys_v) != E_MAX77654_SUCCESS) {
            vsys_v = 0;
        }
        if (max77654_measure(&pmic_h, MAX77654_BATT_I_8MA2, &batt_i) != E_MAX77654_SUCCESS) {
            batt_i = 0;
        }
        current_power_mw = (vsys_v * batt_i) / 1000;
    }

    return 0;
}

int battery_get_status(battery_status_t *status) {
    if (!status) return -1;
    status->soc_percent = (uint8_t)current_soc;
    status->voltage_mv = (uint16_t)current_bat_mv;
    status->is_charging = is_charging;
    status->power_mw = (uint16_t)current_power_mw;
    status->power_source = current_power_source;
    return 0;
}
