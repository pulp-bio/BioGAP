/*
 * ----------------------------------------------------------------------
 *
 * File: power.c
 *
 * Last edited: 05.01.2026
 *
 * Copyright (C) 2026, ETH Zurich and University of Bologna.
 *
 * ----------------------------------------------------------------------
 * SPDX-License-Identifier: Apache-2.0
 */

#include "bsp/power/power.h"
#include <zephyr/drivers/gpio.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/util.h>

#include "pwr/pwr_common.h"
#include "max77654.h"

LOG_MODULE_REGISTER(power_bsp, LOG_LEVEL_INF);

#define GPIO_NODE_ads1298_pwr DT_NODELABEL(gpio_ads1298_pwr)
static const struct gpio_dt_spec gpio_p0_31_ads1298_pwr = GPIO_DT_SPEC_GET(GPIO_NODE_ads1298_pwr, gpios);

int power_init(void) {
    // Enable ADS1298 power GPIO
    if (!device_is_ready(gpio_p0_31_ads1298_pwr.port)) {
        LOG_ERR("ADS1298 power GPIO port not ready");
        return -1;
    }
    
    if (gpio_pin_configure_dt(&gpio_p0_31_ads1298_pwr, GPIO_OUTPUT_INACTIVE) < 0) {
        LOG_ERR("ADS pwr GPIO init error");
        return -1;
    }
    
    LOG_INF("Power BSP initialized");
    return 0;
}

int power_ads_off(void) {
    struct max77654_conf *pmic_conf = &pmic_h.conf;

    // Disable ADS1298 power
    if (gpio_pin_set_dt(&gpio_p0_31_ads1298_pwr, 0) < 0) {
        LOG_ERR("ADS1298 power GPIO clear error");
        return -1;
    }

    // Switch off VA1
    pmic_conf->ldo_conf[1].mode = MAX77654_LDO_MODE_LDO;
    pmic_conf->ldo_conf[1].active_discharge = true;
    pmic_conf->ldo_conf[1].en = MAX77654_REG_OFF;
    pmic_conf->ldo_conf[1].output_voltage_mV = 0;

    max77654_config(&pmic_h);

    return 0;
}

int power_ads_on_unipolar(void) {
    struct max77654_conf *pmic_conf = &pmic_h.conf;

    // Enable ADS1298 power
    if (gpio_pin_set_dt(&gpio_p0_31_ads1298_pwr, 1) < 0) {
        LOG_ERR("ADS1298 power GPIO set error");
        return -1;
    }

    // Switch on VA1, set to 3.0V for unipolar configuration
    pmic_conf->ldo_conf[1].mode = MAX77654_LDO_MODE_LDO;
    pmic_conf->ldo_conf[1].active_discharge = false;
    pmic_conf->ldo_conf[1].en = MAX77654_REG_ON;
    pmic_conf->ldo_conf[1].output_voltage_mV = 3000;

    max77654_config(&pmic_h);

    return 0;
}

int power_ads_on_bipolar(void) {
    struct max77654_conf *pmic_conf = &pmic_h.conf;

    // Enable ADS1298 power
    if (gpio_pin_set_dt(&gpio_p0_31_ads1298_pwr, 1) < 0) {
        LOG_ERR("ADS1298 power GPIO set error");
        return -1;
    }

    // First, disable VA1 and SBB1
    pmic_conf->ldo_conf[1].mode = MAX77654_LDO_MODE_LDO;
    pmic_conf->ldo_conf[1].active_discharge = true;
    pmic_conf->ldo_conf[1].en = MAX77654_REG_OFF;
    pmic_conf->ldo_conf[1].output_voltage_mV = 1500;

    pmic_conf->sbb_conf[1].mode = MAX77654_SBB_MODE_BUCKBOOST;
    pmic_conf->sbb_conf[1].peak_current = MAX77654_SBB_PEAK_CURRENT_1A;
    pmic_conf->sbb_conf[1].active_discharge = true;
    pmic_conf->sbb_conf[1].en = MAX77654_REG_OFF;
    pmic_conf->sbb_conf[1].output_voltage_mV = 1800;

    max77654_config(&pmic_h);

    // Then enable VA1 and SBB1 for bipolar operation
    pmic_conf->ldo_conf[1].mode = MAX77654_LDO_MODE_LDO;
    pmic_conf->ldo_conf[1].active_discharge = false;
    pmic_conf->ldo_conf[1].en = MAX77654_REG_ON;
    pmic_conf->ldo_conf[1].output_voltage_mV = 1500;

    pmic_conf->sbb_conf[1].mode = MAX77654_SBB_MODE_BUCKBOOST;
    pmic_conf->sbb_conf[1].peak_current = MAX77654_SBB_PEAK_CURRENT_1A;
    pmic_conf->sbb_conf[1].active_discharge = false;
    pmic_conf->sbb_conf[1].en = MAX77654_REG_ON;
    pmic_conf->sbb_conf[1].output_voltage_mV = 2700;

    max77654_config(&pmic_h);

    return 0;
}

int power_exg_on(void) {
#if defined(CONFIG_SENSOR_EEG) && !defined(CONFIG_SENSOR_EMG)
    LOG_INF("EEG sensor - powering on unipolar configuration");
    return power_ads_on_unipolar();
#elif defined(CONFIG_SENSOR_EMG) && !defined(CONFIG_SENSOR_EEG)
    LOG_INF("EMG sensor - powering on bipolar configuration");
    return power_ads_on_bipolar();
#else
    LOG_ERR("No EXG sensor enabled");
    return -EINVAL;
#endif
}
