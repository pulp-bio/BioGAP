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
#include "pwr/pwr_common.h"

#include <zephyr/drivers/gpio.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>

#include "pwr/pwr.h"

#include "max77654.h"


#include "i2c_helpers.h"

// ======== Defines/Variables ======================================================================

LOG_MODULE_REGISTER(pwr_bsp, LOG_LEVEL_INF);


/* Define stack sizes and priorities */
#define BAT_UPDATE_STACK_SIZE    1024
#define BAT_UPDATE_PRIORITY      6


static const struct device *pwr_i2c = DEVICE_DT_GET(DT_ALIAS(i2ca));

// Store battery information
uint32_t soc = 0, bat_mv = 0, power_mw = 0;
bool charging = false;
const char *power_source = NULL;



/* The devicetree node identifier */
#define GPIO_NODE_scd41_pwr DT_NODELABEL(gpio_scd41_pwr)
#define GPIO_NODE_sgp41_pwr DT_NODELABEL(gpio_sgp41_pwr)

#define GPIO_NODE_i2c_sgp41_en DT_NODELABEL(gpio_ext_i2c_sgp41_en)
#define GPIO_NODE_i2c_as7331_en DT_NODELABEL(gpio_ext_i2c_as7331_en)
#define GPIO_NODE_i2c_scd41_en DT_NODELABEL(gpio_ext_i2c_scd41_en)
#define GPIO_NODE_hm0360_clk_en DT_NODELABEL(gpio_ext_hm0360_clk_en)

#define GPIO_NODE_gap9_i2c_ctrl DT_NODELABEL(gpio_gap9_i2c_ctrl)

#define GPIO_NODE_ads1298_pwr DT_NODELABEL(gpio_ads1298_pwr)


static const struct gpio_dt_spec gpio_p0_6_gap9_i2c_ctrl = GPIO_DT_SPEC_GET(GPIO_NODE_gap9_i2c_ctrl, gpios);

static const struct gpio_dt_spec gpio_p0_31_ads1298_pwr = GPIO_DT_SPEC_GET(GPIO_NODE_ads1298_pwr, gpios);




/**
 * @brief Battery update thread
 *
 * This thread chills around and updates the battery status every 5 seconds.
 *
 * @param arg1 Unused
 * @param arg2 Unused
 * @param arg3 Unused
 */
void battery_update_thread(void *arg1, void *arg2, void *arg3)
{

  k_sleep(K_MSEC(2000)); // Wait for 2 seconds before starting the thread

  while (1) {
    k_sleep(K_MSEC(5000));
    int ret = pwr_get_full_status(&soc, &bat_mv, &power_mw, &charging, &power_source);
  }

}
// ======== Functions ==============================================================================

int pwr_bsp_init() {
  
  // Configure GAP9 I2C bus control pin
  if (gpio_pin_configure_dt(&gpio_p0_6_gap9_i2c_ctrl, GPIO_OUTPUT_INACTIVE) < 0) {
    LOG_ERR("GAP9 I2C GPIO init error");
    return 0;
  }

  if (!device_is_ready(pwr_i2c)) {
    LOG_ERR("PWR I2C not ready!");
    return -1;
  }

  // Enable ADS1298 power
  if (!device_is_ready(gpio_p0_31_ads1298_pwr.port)) {
      LOG_ERR("ADS1298 power GPIO port not ready");
      return -1;
  }
  // Configure GAP9 I2C bus control pin
  if (gpio_pin_configure_dt(&gpio_p0_31_ads1298_pwr, GPIO_OUTPUT_INACTIVE) < 0) {
    LOG_ERR("ADS pwr GPIO init error");
    return 0;
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


int pwr_ads_off() {
  // Configure PMIC
  struct max77654_conf *pmic_conf = &pmic_h.conf;


  // Disable ADS1298 power
  if (gpio_pin_set_dt(&gpio_p0_31_ads1298_pwr, 0) < 0) {
      LOG_ERR("ADS1298 power GPIO clear error");
      return -1;
  }

  // Switch on VA1, set to 3.0V for unipolar configuration
  pmic_conf->ldo_conf[1].mode = MAX77654_LDO_MODE_LDO;
  pmic_conf->ldo_conf[1].active_discharge = true;
  pmic_conf->ldo_conf[1].en = MAX77654_REG_OFF;
  pmic_conf->ldo_conf[1].output_voltage_mV = 0;

  max77654_config(&pmic_h);

  return 0;
}

int pwr_ads_on_unipolar() {
  // Configure PMIC
  struct max77654_conf *pmic_conf = &pmic_h.conf;


  // Enable ADS1298 power
  if (gpio_pin_set_dt(&gpio_p0_31_ads1298_pwr, 1) < 0) {  // Set pin high to enable
      LOG_ERR("ADS1298 power GPIO set error");
      return -1;
  }

  // Switch on VA1, set to 3.0V for unipolar configuration
  pmic_conf->ldo_conf[1].mode = MAX77654_LDO_MODE_LDO;
  pmic_conf->ldo_conf[1].active_discharge = false;
  pmic_conf->ldo_conf[1].en = MAX77654_REG_ON;
  pmic_conf->ldo_conf[1].output_voltage_mV = 3000;

  max77654_config(&pmic_h);

  return 0;
}



int pwr_ads_on_bipolar() {
  // Configure PMIC
  struct max77654_conf *pmic_conf = &pmic_h.conf;


  

  // Enable ADS1298 power
  if (gpio_pin_set_dt(&gpio_p0_31_ads1298_pwr, 1) < 0) {  // Set pin high to enable
      LOG_ERR("ADS1298 power GPIO set error");
      return -1;
  }

/*
  // Disable ADS1298 power
  if (gpio_pin_set_dt(&gpio_p0_31_ads1298_pwr, 0) < 0) {
      LOG_ERR("ADS1298 power GPIO clear error");
      return -1;
  }

*/
  // Switch on VA1, set to 3.0V for unipolar configuration
  pmic_conf->ldo_conf[1].mode = MAX77654_LDO_MODE_LDO;
  pmic_conf->ldo_conf[1].active_discharge = true;
  pmic_conf->ldo_conf[1].en = MAX77654_REG_OFF;
  pmic_conf->ldo_conf[1].output_voltage_mV = 1500;


  pmic_conf->sbb_conf[1].mode = MAX77654_SBB_MODE_BUCKBOOST;
  pmic_conf->sbb_conf[1].peak_current = MAX77654_SBB_PEAK_CURRENT_1A;
  pmic_conf->sbb_conf[1].active_discharge = true;
  pmic_conf->sbb_conf[1].en = MAX77654_REG_OFF;
  pmic_conf->sbb_conf[1].output_voltage_mV = 1800;

  max77654_config(&pmic_h);

  // Switch on VA1, set to 3.0V for unipolar configuration
  pmic_conf->ldo_conf[1].mode = MAX77654_LDO_MODE_LDO;
  pmic_conf->ldo_conf[1].active_discharge = false;
  pmic_conf->ldo_conf[1].en = MAX77654_REG_ON;
  pmic_conf->ldo_conf[1].output_voltage_mV = 1500;


  pmic_conf->sbb_conf[1].mode = MAX77654_SBB_MODE_BUCKBOOST;
  pmic_conf->sbb_conf[1].peak_current = MAX77654_SBB_PEAK_CURRENT_1A;
  pmic_conf->sbb_conf[1].active_discharge = false;
  pmic_conf->sbb_conf[1].en = MAX77654_REG_ON;
  pmic_conf->sbb_conf[1].output_voltage_mV = 2700;

  max77654_config(&pmic_h);


  return 0;
}



int pwr_update_battery_status() {

  int ret = pwr_get_full_status(&soc, &bat_mv, &power_mw, &charging, &power_source);
  
  return ret;
}
// pwr_get_full_status() now returns:
//   soc         : Battery state of charge in percent (0-100)
//   bat_mv      : Battery voltage (in mV)
//   power_mw    : Total system power consumed (in mW)
//   charging    : True if external power (USB/charger) is present
//   power_source: A string: either "USB/External" or "Battery"
int pwr_get_full_status(uint32_t *soc, uint32_t *bat_mv, uint32_t *power_mw, bool *charging, const char **power_source)
{
    if (!soc || !bat_mv || !power_mw || !charging || !power_source) {
        return -1;
    }

    // Get battery percentage and battery voltage as before.
    *soc = pwr_bat_perc();
    *bat_mv = pwr_bat_mV();

    // Retrieve PMIC status (see thread_pwr.c :contentReference[oaicite:2]{index=2}&#8203;:contentReference[oaicite:3]{index=3}).
    struct max77654_stat stat;
    if (max77654_get_stat(&pmic_h, &stat) != E_MAX77654_SUCCESS) {
        return -1;
    }

    // Decide the power source based on the CHGIN status.
    // (If chgin_status is not equal to the under-voltage threshold, external power is present.)
    if (stat.chgin_status != MAX77654_CHGIN_DTLS_UVLO) {
        *power_source = "USB/External";
        *charging = true;
        // Read charger input voltage and current.
        uint32_t chgin_v = 0, chgin_i = 0;
        if (max77654_measure(&pmic_h, MAX77654_CHGIN_V, &chgin_v) != E_MAX77654_SUCCESS) {
            chgin_v = 0;
        }
        if (max77654_measure(&pmic_h, MAX77654_CHGIN_I, &chgin_i) != E_MAX77654_SUCCESS) {
            chgin_i = 0;
        }
        // Compute total power consumed (in mW): (voltage in mV * current in mA) / 1000.
        *power_mw = (chgin_v * chgin_i) / 1000;
    } else {
        *power_source = "Battery";
        *charging = false;
        // When not externally powered, use the system voltage (VSYS) and the battery discharge current.
        uint32_t vsys_v = 0, batt_i = 0;
        if (max77654_measure(&pmic_h, MAX77654_VSYS, &vsys_v) != E_MAX77654_SUCCESS) {
            vsys_v = 0;
        }
        if (max77654_measure(&pmic_h, MAX77654_BATT_I_8MA2, &batt_i) != E_MAX77654_SUCCESS) {
            batt_i = 0;
        }
        *power_mw = (vsys_v * batt_i) / 1000;
    }

    return 0;
}


// Get Battery State-of-Charge (in percent)
// Returns the last updated SOC value.
uint32_t bsp_get_battery_soc(void)
{
    // Optionally force an update:
    //pwr_update_battery_status();
    return soc;
}

// Get Battery Voltage (in millivolts)
// Returns the last updated battery voltage.
uint32_t bsp_get_battery_voltage(void)
{
    //pwr_update_battery_status();
    return bat_mv;
}

// Get Total System Power (in mW)
// When externally powered, this computes (CHGIN_V * CHGIN_I)/1000,
// otherwise (VSYS * battery_discharge_current)/1000.
uint32_t bsp_get_total_power_mw(void)
{
    //pwr_update_battery_status();
    return power_mw;
}

// Get Charging Status
// Returns 1 if external (USB/charger) power is active (i.e. battery is charging),
// or 0 if running off battery.
uint8_t bsp_is_charging(void)
{
    //pwr_update_battery_status();
    return charging ? 1 : 0;
}

// Get Power Source as a string
// Returns "USB/External" if external power is present, "Battery" otherwise.
const char* bsp_get_power_source(void)
{
    //pwr_update_battery_status();
    return power_source;
}


//K_THREAD_DEFINE(battery_update_tid, BAT_UPDATE_STACK_SIZE, battery_update_thread, NULL, NULL, NULL, BAT_UPDATE_PRIORITY, 0, 0);