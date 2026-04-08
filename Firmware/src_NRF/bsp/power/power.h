/*
 * ----------------------------------------------------------------------
 *
 * File: power.h
 *
 * Last edited: 05.01.2026
 *
 * Copyright (C) 2026, ETH Zurich and University of Bologna.
 *
 * ----------------------------------------------------------------------
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef POWER_BSP_H_
#define POWER_BSP_H_

/**
 * @brief Initialize the power subsystem.
 * 
 * Initializes GPIO pins for ADS1298 power control.
 * 
 * @return 0 on success, negative on error.
 */
int power_init(void);

/**
 * @brief Turn on ADS1298 power in unipolar configuration.
 * 
 * Enables ADS1298 power GPIO and configures PMIC LDO for 3.0V.
 * 
 * @return 0 on success, negative on error.
 */
int power_ads_on_unipolar(void);

/**
 * @brief Turn on ADS1298 power in bipolar configuration.
 * 
 * Enables ADS1298 power GPIO and configures PMIC for bipolar operation.
 * 
 * @return 0 on success, negative on error.
 */
int power_ads_on_bipolar(void);

/**
 * @brief Turn off ADS1298 power.
 * 
 * Disables ADS1298 power GPIO and turns off PMIC LDO.
 * 
 * @return 0 on success, negative on error.
 */
int power_ads_off(void);

/**
 * @brief Power on EXG sensor based on Kconfig selection.
 * 
 * Calls the appropriate power function based on the enabled sensor:
 * - EEG (CONFIG_SENSOR_EEG): power_ads_on_unipolar()
 * - EMG (CONFIG_SENSOR_EMG): power_ads_on_bipolar()
 * 
 * @return 0 on success, -EINVAL if no sensor is enabled
 */
int power_exg_on(void);

#endif /* POWER_BSP_H_ */
