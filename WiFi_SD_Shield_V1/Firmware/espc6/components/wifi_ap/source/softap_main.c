#include "softap_main.h"
#include "esp_log.h"
#include "esp_wifi.h"
#include "esp_mac.h"
#include "nvs_flash.h"

static const char *TAG = "softap_main.c";

static void wifi_station_event_handler(void* arg, esp_event_base_t event_base,
                                    int32_t event_id, void* event_data)
{
    if (event_id == WIFI_EVENT_AP_STACONNECTED) {
        wifi_event_ap_staconnected_t* event = (wifi_event_ap_staconnected_t*) event_data;
        ESP_LOGI(TAG, "station " MACSTR " join, AID=%d", MAC2STR(event->mac), (int)event->aid);
    } 
    else if (event_id == WIFI_EVENT_AP_STADISCONNECTED) {
        wifi_event_ap_stadisconnected_t* event = (wifi_event_ap_stadisconnected_t*) event_data;
        ESP_LOGI(TAG, "station " MACSTR " leave, AID=%d, reason=%d", MAC2STR(event->mac), (int)event->aid, (int)event->reason);
    }
}

esp_err_t wifi_init_softap(void)
{
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    if (ret != ESP_OK) {
        return ret;
    }

    //vTaskDelay(pdMS_TO_TICKS(1000));
    ESP_LOGI(TAG, "Nvs OKAY"); 

    ret = esp_netif_init();
    if (ret != ESP_OK) {
        return ret;
    }
    //vTaskDelay(pdMS_TO_TICKS(1000));
    ESP_LOGI(TAG, "esp_netif_init OKAY"); 

    ret = esp_event_loop_create_default();
    if (ret != ESP_OK) {
        return ret;
    }
    //vTaskDelay(pdMS_TO_TICKS(1000));
    ESP_LOGI(TAG, "esp_event_loop_create_default OKAY");

    esp_netif_create_default_wifi_ap();
    //vTaskDelay(pdMS_TO_TICKS(1000));
    ESP_LOGI(TAG, "esp_netif_create_default_wifi_ap OKAY");

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ret = esp_wifi_init(&cfg);
    if (ret != ESP_OK) {
        return ret;
    }
    //vTaskDelay(pdMS_TO_TICKS(1000));
    ESP_LOGI(TAG, "esp_wifi_init OKAY");

    ret = esp_event_handler_instance_register(WIFI_EVENT, ESP_EVENT_ANY_ID, &wifi_station_event_handler, NULL, NULL);
    if (ret != ESP_OK) {
        return ret;
    }
    // Set up WiFi configuration for soft-AP
    wifi_config_t wifi_config = {
        .ap = {
            .ssid = EXAMPLE_ESP_WIFI_SSID,
            .ssid_len = (uint8_t)strlen(EXAMPLE_ESP_WIFI_SSID),
            .channel = EXAMPLE_ESP_WIFI_CHANNEL,
            .password = EXAMPLE_ESP_WIFI_PASS,
            .max_connection = 1,
#ifdef CONFIG_ESP_WIFI_SOFTAP_SAE_SUPPORT
            .authmode = WIFI_AUTH_WPA3_PSK,
            .sae_pwe_h2e = WPA3_SAE_PWE_BOTH,
#else
            .authmode = WIFI_AUTH_WPA2_PSK,
#endif
            .pmf_cfg = {
                    .required = true,
            },
        },
    };
    if (strlen(EXAMPLE_ESP_WIFI_PASS) == 0) {
        wifi_config.ap.authmode = WIFI_AUTH_OPEN;
    }

    ret = esp_wifi_set_mode(WIFI_MODE_AP);
    if (ret != ESP_OK) {
        return ret;
    }
    //vTaskDelay(pdMS_TO_TICKS(1000));
    ESP_LOGI(TAG, "esp_wifi_set_mode(WIFI_MODE_AP); OKAY");
    ret = esp_wifi_set_config(WIFI_IF_AP, &wifi_config);
    if (ret != ESP_OK) {
        return ret;
    }
    //vTaskDelay(pdMS_TO_TICKS(1000));
    ESP_LOGI(TAG, "esp_wifi_set_config(WIFI_IF_AP, &wifi_config); OKAY");
    gpio_set_level(RTC_SCL, 0);
    ret = esp_wifi_start();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "esp_wifi_start failed: %s", esp_err_to_name(ret));
        return ret;
    }
    gpio_set_level(RTC_SCL, 1);
    ESP_LOGI(TAG, "wifi_init_softap finished. SSID:%s password:%s channel:%d", EXAMPLE_ESP_WIFI_SSID, EXAMPLE_ESP_WIFI_PASS, (int)EXAMPLE_ESP_WIFI_CHANNEL);
    return ESP_OK;
}

void tcp_server_task(void *pvParameters)
{
    ESP_LOGI("TCP", "Ready to accept connections...");
    for (;;) {
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}
