/*
 * ----------------------------------------------------------------------
 *
 * File: pwr_bsp.c
 *
 * Last edited: 19.06.2024
 *
 * Copyright (C) 2024, ETH Zurich and University of Bologna.
 *
 * Authors:
 * - Philip Wiese (wiesep@iis.ee.ethz.ch), ETH Zurich
 * - Sebastian Frey (sefrey@iis.ee.ethz.ch), ETH Zurich
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

#include "bsp/pwr_bsp.h"
#include "bsp/power/power.h"
#include "pwr/pwr_common.h"

#include <zephyr/drivers/gpio.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>

#include "pwr/pwr.h"

#include "max77654.h"
#include "afe/ads_appl.h"


#include "core/i2c_helpers.h"

// ======== Defines/Variables ======================================================================

LOG_MODULE_REGISTER(pwr_bsp, LOG_LEVEL_INF);

static const struct device *pwr_i2c = DEVICE_DT_GET(DT_ALIAS(i2ca));

// Store soft reset flag
bool flag_isr_soft_reset = 0;

/* GAP9 I2C bus control GPIO */
#define GPIO_NODE_gap9_i2c_ctrl DT_NODELABEL(gpio_gap9_i2c_ctrl)
static const struct gpio_dt_spec gpio_p0_6_gap9_i2c_ctrl = GPIO_DT_SPEC_GET(GPIO_NODE_gap9_i2c_ctrl, gpios);

/* Soft reset button GPIO */
#define BUTTON_SOFT_RST_INT_NODE DT_NODELABEL(gpio_soft_rst)
static const struct gpio_dt_spec soft_rst_int_gpio = GPIO_DT_SPEC_GET(BUTTON_SOFT_RST_INT_NODE, gpios);
static struct gpio_callback soft_rst_cb_data;

void soft_rst_irq_callback(const struct device *dev, struct gpio_callback *cb, uint32_t pins)
{
  flag_isr_soft_reset = 1;
}

int pwr_bsp_init() {
  // Configure soft reset button
  if (!device_is_ready(soft_rst_int_gpio.port)) {
      LOG_ERR("Soft reset GPIO port not ready");
      return -1;
  }
  if (gpio_pin_configure_dt(&soft_rst_int_gpio, GPIO_INPUT) < 0) {
    LOG_ERR("Soft reset GPIO init error");
    return -1;
  }
  if (gpio_pin_interrupt_configure_dt(&soft_rst_int_gpio, GPIO_INT_EDGE_TO_ACTIVE) < 0) {
      LOG_ERR("Failed to configure interrupt on GPIO pin %d (error)", soft_rst_int_gpio.pin);
      return -1;
  }
  gpio_init_callback(&soft_rst_cb_data, soft_rst_irq_callback, BIT(soft_rst_int_gpio.pin));
  gpio_add_callback(soft_rst_int_gpio.port, &soft_rst_cb_data);

  // Configure GAP9 I2C bus control pin
  if (gpio_pin_configure_dt(&gpio_p0_6_gap9_i2c_ctrl, GPIO_OUTPUT_INACTIVE) < 0) {
    LOG_ERR("GAP9 I2C GPIO init error");
    return -1;
  }

  if (!device_is_ready(pwr_i2c)) {
    LOG_ERR("PWR I2C not ready!");
    return -1;
  }

  // Initialize power subsystem
  if (power_init() < 0) {
    LOG_ERR("Power subsystem init error");
    return -1;
  }

  return 0;
}

int pwr_charge_enable()
{
    struct max77654_conf *pmic_conf = &pmic_h.conf;
    
    pmic_conf->charger_enabled = true;           // Enable charger
    pmic_conf->preq_i = MAX77654_I_PQ_20PERCENT; // Pre-qual current ~5mA (20% of 22.5mA)
    pmic_conf->chgin_i_lim = MAX77654_ICHGIN_LIM_0A285; // Input current limit 285mA
    pmic_conf->fast_chg_cc = MAX77654_CHG_CC_90MA; // Fast charge current 22.5mA
    pmic_conf->fast_chg_cv = MAX77654_CHG_CV_4V2; // Charge voltage 4.2V
    pmic_conf->termnation_i = MAX77654_I_TERM_10PERCENT; // Term current ~2.25mA
    pmic_conf->topoff_t = MAX77654_T_TOPOFF_30MIN; // Top-off timer 30min
    pmic_conf->t_fast_chg = MAX77654_T_FAST_CHG_5H; // Fast charge timer 5hrs
    pmic_conf->thm_en = false;                  // Disable thermal monitoring
    
    pmic_conf->vsys_regulation = MAX77654_VSYS_4V7; // System voltage regulation 4.1V
    pmic_conf->USB_suspend = false;             // Don't suspend USB charging

    return max77654_config(&pmic_h);
}

int pwr_bsp_start() {
  // Configure PMIC
  struct max77654_conf *pmic_conf = &pmic_h.conf;

  pmic_conf->sbb_conf[0].mode = MAX77654_SBB_MODE_BUCKBOOST;
  pmic_conf->sbb_conf[0].peak_current = MAX77654_SBB_PEAK_CURRENT_1A;
  pmic_conf->sbb_conf[0].active_discharge = false;
  pmic_conf->sbb_conf[0].en = MAX77654_REG_ON;
  pmic_conf->sbb_conf[0].output_voltage_mV = 3300;

  pmic_conf->sbb_conf[1].mode = MAX77654_SBB_MODE_BUCKBOOST;
  pmic_conf->sbb_conf[1].peak_current = MAX77654_SBB_PEAK_CURRENT_1A;
  pmic_conf->sbb_conf[1].active_discharge = false;
  pmic_conf->sbb_conf[1].en = MAX77654_REG_ON;
  pmic_conf->sbb_conf[1].output_voltage_mV = 2800;

  pmic_conf->sbb_conf[2].mode = MAX77654_SBB_MODE_BUCKBOOST;
  pmic_conf->sbb_conf[2].peak_current = MAX77654_SBB_PEAK_CURRENT_1A;
  pmic_conf->sbb_conf[2].active_discharge = false;
  pmic_conf->sbb_conf[2].en = MAX77654_REG_ON;
  pmic_conf->sbb_conf[2].output_voltage_mV = 1200;

  pmic_conf->ldo_conf[0].mode = MAX77654_LDO_MODE_LDO;
  pmic_conf->ldo_conf[0].active_discharge = false;
  pmic_conf->ldo_conf[0].en = MAX77654_REG_ON;
  pmic_conf->ldo_conf[0].output_voltage_mV = 3300;

  max77654_config(&pmic_h);

  // Power up GAP9 and connect to I2C bus
  gap9_pwr(true);
  LOG_INF("GAP9 powered up");

  if (gpio_pin_set_dt(&gpio_p0_6_gap9_i2c_ctrl, 1) < 0) {
    LOG_ERR("GAP9 I2C GPIO configuration error");
    return -1;
  }
  LOG_INF("GAP9 connected to I2C bus");

  return 0;
}