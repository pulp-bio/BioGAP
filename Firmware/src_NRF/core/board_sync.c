/*
 * ----------------------------------------------------------------------
 *
 * File: board_sync.c
 *
 * Copyright (C) 2025, ETH Zurich and University of Bologna
 *
 * ----------------------------------------------------------------------
 * SPDX-License-Identifier: Apache-2.0
 *
 * Licensed under the Apache License, Version 2.0 (the License); you may
 * not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an AS IS BASIS, WITHOUT
 * WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

/**
 * @file board_sync.c
 * @brief Inter-Board Hardware Synchronization Implementation
 */

#include "core/board_sync.h"

#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(board_sync, LOG_LEVEL_INF);

/*==============================================================================
 * Device Tree References
 *============================================================================*/

#if DT_NODE_EXISTS(DT_NODELABEL(gpio_board_sync))
#define SYNC_PIN_NODE DT_NODELABEL(gpio_board_sync)
static const struct gpio_dt_spec sync_pin = GPIO_DT_SPEC_GET(SYNC_PIN_NODE, gpios);
#define BOARD_SYNC_HW_AVAILABLE 1
#else
#define BOARD_SYNC_HW_AVAILABLE 0
#endif

/*==============================================================================
 * Module Variables
 *============================================================================*/

#if BOARD_SYNC_HW_AVAILABLE
static struct gpio_callback sync_cb_data;
#endif

static K_SEM_DEFINE(sync_sem, 0, 1);
static uint8_t sync_pulse_count = 0;

/*==============================================================================
 * GPIO Callback (SECONDARY only)
 *============================================================================*/

#if BOARD_SYNC_HW_AVAILABLE && defined(CONFIG_BOARD_SYNC_ROLE_SECONDARY)
static void sync_gpio_callback(const struct device *dev,
                               struct gpio_callback *cb,
                               uint32_t pins) {
    sync_pulse_count++;
    k_sem_give(&sync_sem);
    LOG_DBG("Sync pulse received (%d)", sync_pulse_count);
}
#endif

/*==============================================================================
 * Public Functions
 *============================================================================*/

int board_sync_init(void) {
#if !BOARD_SYNC_HW_AVAILABLE
    LOG_WRN("Board sync GPIO not defined in device tree");
    return -ENODEV;
#else
    if (!device_is_ready(sync_pin.port)) {
        LOG_ERR("Sync GPIO port not ready");
        return -ENODEV;
    }

#if defined(CONFIG_BOARD_SYNC_ROLE_PRIMARY)
    /* PRIMARY: Configure as output, initially low */
    int ret = gpio_pin_configure_dt(&sync_pin, GPIO_OUTPUT_INACTIVE);
    if (ret < 0) {
        LOG_ERR("Failed to configure sync pin as output: %d", ret);
        return ret;
    }
    LOG_INF("Board sync: PRIMARY (Board ID %d)", CONFIG_BOARD_ID);

#elif defined(CONFIG_BOARD_SYNC_ROLE_SECONDARY)
    /* SECONDARY: Configure as input with interrupt on rising edge */
    int ret = gpio_pin_configure_dt(&sync_pin, GPIO_INPUT);
    if (ret < 0) {
        LOG_ERR("Failed to configure sync pin as input: %d", ret);
        return ret;
    }

    ret = gpio_pin_interrupt_configure_dt(&sync_pin, GPIO_INT_EDGE_TO_ACTIVE);
    if (ret < 0) {
        LOG_ERR("Failed to configure sync interrupt: %d", ret);
        return ret;
    }

    gpio_init_callback(&sync_cb_data, sync_gpio_callback, BIT(sync_pin.pin));
    ret = gpio_add_callback(sync_pin.port, &sync_cb_data);
    if (ret < 0) {
        LOG_ERR("Failed to add sync callback: %d", ret);
        return ret;
    }
    LOG_INF("Board sync: SECONDARY (Board ID %d)", CONFIG_BOARD_ID);

#else
    /* STANDALONE: No sync */
    LOG_INF("Board sync: STANDALONE (Board ID %d)", CONFIG_BOARD_ID);
#endif

    return 0;
#endif /* BOARD_SYNC_HW_AVAILABLE */
}

void board_sync_assert(void) {
#if BOARD_SYNC_HW_AVAILABLE && defined(CONFIG_BOARD_SYNC_ROLE_PRIMARY)
    gpio_pin_set_dt(&sync_pin, 1);
    sync_pulse_count++;
    LOG_INF("Sync asserted (pulse %d)", sync_pulse_count);
#endif
}

void board_sync_deassert(void) {
#if BOARD_SYNC_HW_AVAILABLE && defined(CONFIG_BOARD_SYNC_ROLE_PRIMARY)
    gpio_pin_set_dt(&sync_pin, 0);
    LOG_DBG("Sync deasserted");
#endif
}

void board_sync_pulse(void) {
#if BOARD_SYNC_HW_AVAILABLE && defined(CONFIG_BOARD_SYNC_ROLE_PRIMARY)
    gpio_pin_set_dt(&sync_pin, 1);
    k_busy_wait(10); /* 10µs pulse */
    gpio_pin_set_dt(&sync_pin, 0);
    sync_pulse_count++;
    LOG_DBG("Sync pulse %d", sync_pulse_count);
#endif
}

int board_sync_wait(uint32_t timeout_ms) {
#if BOARD_SYNC_HW_AVAILABLE && defined(CONFIG_BOARD_SYNC_ROLE_SECONDARY)
    k_timeout_t timeout;
    if (timeout_ms == 0) {
        timeout = K_FOREVER;
    } else {
        timeout = K_MSEC(timeout_ms);
    }
    int ret = k_sem_take(&sync_sem, timeout);
    if (ret == -EAGAIN) {
        LOG_ERR("Inter-board sync timeout");
        return -ETIMEDOUT;
    }
    LOG_INF("Inter-board sync received (pulse %d)", sync_pulse_count);
    return 0;
#else
    /* PRIMARY and STANDALONE don't wait */
    return 0;
#endif
}

bool board_sync_is_primary(void) {
#if defined(CONFIG_BOARD_SYNC_ROLE_PRIMARY)
    return true;
#else
    return false;
#endif
}

uint8_t board_sync_get_pulse_count(void) {
    return sync_pulse_count;
}

uint8_t board_sync_get_board_id(void) {
#if defined(CONFIG_BOARD_ID)
    return CONFIG_BOARD_ID;
#else
    return 0;
#endif
}
