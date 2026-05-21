/*
 * SPDX-FileCopyrightText: 2022-2023 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Apache-2.0
 */
#pragma once

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
    LED_PIXEL_FORMAT_GRB,
    LED_PIXEL_FORMAT_GRBW,
    LED_PIXEL_FORMAT_INVALID
} led_pixel_format_t;

typedef enum {
    LED_MODEL_WS2812,
    LED_MODEL_SK6812,
    LED_MODEL_INVALID
} led_model_t;

typedef struct led_strip_t *led_strip_handle_t;

typedef struct {
    int strip_gpio_num;
    uint32_t max_leds;
    led_pixel_format_t led_pixel_format;
    led_model_t led_model;

    struct {
        uint32_t invert_out: 1;
    } flags;
} led_strip_config_t;

#ifdef __cplusplus
}
#endif
