/*
 * ----------------------------------------------------------------------
 *
 * File: softap_main.h
 *
 * Last edited: 17.07.2026
 *
 * Copyright (c) 2026 ETH Zurich and University of Bologna
 *
 * Authors:
 * - Giusy Spacone (gspacone@iis.ee.ethz.ch), ETH Zurich
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