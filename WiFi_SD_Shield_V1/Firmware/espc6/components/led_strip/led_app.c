#include "led_app.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

uint8_t s_led_state = 0;
led_strip_handle_t led_strip = NULL;
const char *LED_TAG = "led_app";

void led_signaling(uint8_t num_blinks)
{
    for (int i = 0; i < num_blinks; i++) {
        blink_led();
        vTaskDelay(pdMS_TO_TICKS(200));
    }

    led_strip_clear(led_strip);
}

#ifdef CONFIG_BLINK_LED_STRIP

void configure_led(void)
{
    ESP_LOGI(LED_TAG, "Configured to blink addressable LED");

    led_strip_config_t strip_config = {
        .strip_gpio_num = BLINK_GPIO,
        .max_leds = 1,
    };

#if CONFIG_BLINK_LED_STRIP_BACKEND_RMT
    led_strip_rmt_config_t rmt_config = {
        .resolution_hz = 10 * 1000 * 1000,
        .flags.with_dma = false,
    };
    ESP_ERROR_CHECK(led_strip_new_rmt_device(&strip_config, &rmt_config, &led_strip));
#else
#error "unsupported LED strip backend"
#endif

    led_strip_clear(led_strip);
}

void blink_led(void)
{
    s_led_state = !s_led_state;

    if (s_led_state) {
        led_strip_set_pixel(led_strip, 0, 16, 16, 16);
        led_strip_refresh(led_strip);
    } else {
        led_strip_clear(led_strip);
    }
}

#else
#error "unsupported LED type"
#endif
