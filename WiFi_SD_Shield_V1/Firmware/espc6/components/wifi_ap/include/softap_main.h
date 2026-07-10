#include <string.h>
#include <sys/param.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_system.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_log.h"
#include "esp_netif.h"
#include "lwip/err.h"
#include "lwip/sockets.h"
#include "lwip/sys.h"
#include <lwip/netdb.h>
#include "common.h"
#include "connectivity_commands.h"

#ifndef SOFTAP_MAIN_H
#define SOFTAP_MAIN_H

// Defines to set-up WiFi
#define EXAMPLE_ESP_WIFI_SSID      "bioserver"
#define EXAMPLE_ESP_WIFI_PASS      "biogapwifi"
#define EXAMPLE_ESP_WIFI_CHANNEL   1
// ====== Protocol / ports ======
#define PORT_LAPTOP              4444
#define PORT_ESP_NODE            3333

/** @brief Function to initialize WiFi in soft-AP mode */
esp_err_t wifi_init_softap(void); 

/** @brief Global socket for GUI connection */
extern int          gui_sock;   // single GUI connection socket or -1
extern bool         gui_connected; // flag to indicate if a GUI is connected

#endif // SOFTAP_MAIN_H