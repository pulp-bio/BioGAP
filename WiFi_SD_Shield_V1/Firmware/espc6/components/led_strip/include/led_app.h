#ifndef LED_APP_H
#define LED_APP_H

#include "esp_log.h"
#include "led_strip.h"
#include "led_strip_types.h"
#include "led_strip_rmt.h"

#define BLINK_GPIO 8
#define CONFIG_BLINK_LED_STRIP 1
#define CONFIG_BLINK_LED_STRIP_BACKEND_RMT 1

extern uint8_t s_led_state;
extern led_strip_handle_t led_strip;

void blink_led(void);
void configure_led(void);
void led_signaling(uint8_t num_blinks);

#endif // LED_APP_H
