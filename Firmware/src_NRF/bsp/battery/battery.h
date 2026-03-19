/*
 * ----------------------------------------------------------------------
 *
 * File: battery.h
 *
 * Last edited: 02.01.2026
 *
 * Copyright (C) 2026, ETH Zurich and University of Bologna.
 *
 * ----------------------------------------------------------------------
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef BATTERY_BSP_H_
#define BATTERY_BSP_H_

#include <stdbool.h>
#include <stdint.h>

typedef struct {
    uint8_t soc_percent;      // State of charge (0-100)
    uint16_t voltage_mv;      // Battery voltage in millivolts
    bool is_charging;         // True if external power connected
    uint16_t power_mw;        // Current power consumption
    const char *power_source; // Power source string
} battery_status_t;

/**
 * @brief Initialize the battery monitoring subsystem.
 * @return 0 on success, negative on error.
 */
int battery_init(void);

/**
 * @brief Get the current battery status.
 * @param status Pointer to status struct to populate.
 * @return 0 on success, negative on error.
 */
int battery_get_status(battery_status_t *status);

/**
 * @brief Update the battery status by reading from hardware.
 * @return 0 on success, negative on error.
 */
int battery_update_status(void);

#endif /* BATTERY_BSP_H_ */
