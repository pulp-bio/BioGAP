/*
 * ----------------------------------------------------------------------
 *
 * File: i2c_helpers.h
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

#ifndef I2C_HELPERS_H
#define I2C_HELPERS_H

#include <stdint.h>

// With addresses on 7 bits, we can have 128 peripherals maximum, per interface.
// See I2C documentation for more details.
#define MAX_PERIPHERALS 128

typedef struct {
  const struct device *i2c_handle;
  uint8_t i2c_addr;
} i2c_ctx_t;

int32_t i2c_write_reg(void *handle, uint8_t reg, const uint8_t *bufp, uint16_t len);
int32_t i2c_read_reg(void *handle, uint8_t reg, uint8_t *bufp, uint16_t len);

int8_t sensirion_i2c_hal_read(uint8_t address, uint8_t *data, uint16_t count);

int8_t sensirion_i2c_hal_write(uint8_t address, const uint8_t *data, uint16_t count);

void sensirion_i2c_hal_sleep_usec(uint32_t useconds);

void resole_address_to_name(uint8_t address, char *name);

void i2c_scan(const struct device *dev);

#endif /* I2C_HELPERS_H */