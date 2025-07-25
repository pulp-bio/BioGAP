/*
 * ----------------------------------------------------------------------
 *
 * File: i2c_helpers.c
 *
 * Last edited: 24.03.2025
 *
 * Copyright (C) 2025, ETH Zurich and University of Bologna.
 *
 * Authors:
 * - Philip Wiese (wiesep@iis.ee.ethz.ch), ETH Zurich
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

#include "i2c_helpers.h"

#include <zephyr/device.h>
#include <zephyr/kernel.h>

#include <zephyr/drivers/i2c.h>

#include <zephyr/logging/log.h>
#include <zephyr/logging/log_ctrl.h>

LOG_MODULE_REGISTER(sensors, LOG_LEVEL_INF);

// static const struct device *const i2c_a = DEVICE_DT_GET(DT_ALIAS(i2ca));
static const struct device *const i2c_b = DEVICE_DT_GET(DT_ALIAS(i2cb));

int32_t i2c_write_reg(void *handle, uint8_t reg, const uint8_t *bufp, uint16_t len) {
  i2c_ctx_t *ctx = (i2c_ctx_t *)handle;
  LOG_DBG("[0x%02X] Address 0x%02X:", ctx->i2c_addr, reg);
  LOG_HEXDUMP_DBG(bufp, len, "I2C TX");
  return i2c_burst_write(ctx->i2c_handle, ctx->i2c_addr, reg, bufp, len);
}

int32_t i2c_read_reg(void *handle, uint8_t reg, uint8_t *bufp, uint16_t len) {
  i2c_ctx_t *ctx = (i2c_ctx_t *)handle;
  int error = i2c_burst_read(ctx->i2c_handle, ctx->i2c_addr, reg, bufp, len);
  LOG_DBG("[0x%02X] Address 0x%02X:", ctx->i2c_addr, reg);
  LOG_HEXDUMP_DBG(bufp, len, "I2C RX");
  return error;
}

int8_t sensirion_i2c_hal_read(uint8_t address, uint8_t *data, uint16_t count) {
  return i2c_read(i2c_b, data, count, address);
}

int8_t sensirion_i2c_hal_write(uint8_t address, const uint8_t *data, uint16_t count) {
  return i2c_write(i2c_b, data, count, address);
}

void sensirion_i2c_hal_sleep_usec(uint32_t useconds) {
  int32_t remaining = useconds;
  while (remaining > 0) {
    remaining = k_usleep(remaining);
  }
}

void resole_address_to_name(uint8_t address, char *name) {
  switch (address) {
  case 0x0A:
    strcpy(name, "GAP9 (RISC-V SoC)");
    break;
  case 0x19:
    strcpy(name, "LIS2DUXS12 (Accelerometer)");
    break;
  case 0x24:
    strcpy(name, "HM0360 (Camera)");
    break;
  case 0x48:
    strcpy(name, "MAX77654 (PMIC)");
    break;
  case 0x53:
    strcpy(name, "IS31FL3194 (LED Driver)");
    break;
  case 0x6A:
    strcpy(name, "ISM330DHCX (IMU)");
    break;
  case 0x76:
    strcpy(name, "BME680 (Environmental Sensor)");
    break;
  case 0x20:
    strcpy(name, "PCA6416A (GPIO Expander)");
    break;
  case 0x29:
    strcpy(name, "BH1730FVC (Light Sensor)");
    break;
  case 0x42:
    strcpy(name, "MAX-M10S (GNSS)");
    break;
  case 0x59:
    strcpy(name, "SGP41 (VOC Sensor)");
    break;
  case 0x5C:
    strcpy(name, "ILPS28QSW (Pressure Sensor)");
    break;
  case 0x62:
    strcpy(name, "SCD41 (CO2 Sensor)");
    break;
  case 0x74:
    strcpy(name, "AS7331 (UV Sensor)");
    break;
  default:
    strcpy(name, "Unknown");
    break;
  }
}

void i2c_scan(const struct device *dev) {
  uint8_t buf[1];
  uint8_t peripherals[MAX_PERIPHERALS];
  char name[32];

  uint8_t found_nb = 0;
  for (uint8_t i = 0; i < MAX_PERIPHERALS; i++) {
    peripherals[i] = i2c_write(dev, buf, 1, i);

    if (peripherals[i] == 0) {
      found_nb++;
    }
  }
  if (found_nb) {
    LOG_INF(" - Number of Peripherals               : %d", found_nb);
    for (uint8_t i = 0; i < MAX_PERIPHERALS; i++) {
      if (peripherals[i] == 0) {
        resole_address_to_name(i, name);
        LOG_INF(" - Device @ 0x%02X                       : %s", i, name);
      }
    }
  } else {
    LOG_WRN("* Interface %s: No peripheral found\r\n", dev->name);
  }
}