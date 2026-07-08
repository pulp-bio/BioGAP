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
 * @brief ADS1298 analog supply mode, selected at runtime per stream start.
 */
typedef enum {
    EXG_MODE_UNIPOLAR,  /**< EEG: VA1 = 3.0 V (AVSS = GND) */
    EXG_MODE_BIPOLAR,   /**< EMG: VA1 = 1.5 V + VD1 = 2.7 V (AVSS = -1.5 V on shield) */
} exg_mode_t;

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
 * @brief Power on the ADS1298 rails for the requested mode.
 *
 * Selected at runtime by the GUI start command:
 * - EXG_MODE_UNIPOLAR (START_EEG_STREAMING): power_ads_on_unipolar()
 * - EXG_MODE_BIPOLAR  (START_EMG_STREAMING): power_ads_on_bipolar()
 *
 * @param mode Analog supply mode.
 * @return 0 on success, -EINVAL on unknown mode, negative on error.
 */
int power_exg_on(exg_mode_t mode);

#endif /* POWER_BSP_H_ */
