/*
 * ----------------------------------------------------------------------
 *
 * File: shield_config.h
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

#ifndef SHIELD_CONFIG_H
#define SHIELD_CONFIG_H


// ---------------------- AP State Machine ---------------------
#define IS_WBAN  0


// ---------------------- SELECT IF ESP is SPI MASTER OR SLAVE ---------------------
#define IS_ESP_SPI_SLAVE  1

// ----------------------- ESP-only dummy sensor generator -----------------------
// When set, the ESP32 generates synthetic dummy-sensor packets locally (identical
// wire format to Firmware/src_NRF/sensors/dummy_sensor/dummy_sensor_appl.c) and
// streams them straight to BioGUI, bypassing SPI/BIOGAP entirely. For testing the
// WiFi/GUI half of the system in isolation, with no nRF/SPI hardware attached.
#define ESP_LOCAL_DUMMY_SENSOR       0

#endif // SHIELD_CONFIG_H