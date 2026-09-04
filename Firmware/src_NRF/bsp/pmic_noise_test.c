/*
 * ----------------------------------------------------------------------
 *
 * File: pmic_noise_test.c
 *
 * Diagnostic build (CONFIG_PMIC_NOISE_TEST): locate which PMIC activity
 * injects noise into the ExG signal path.
 *
 * Background: with the SDK power thread enabled, short noise bursts were
 * visible in EEG/EMG every 20 s (= the telemetry cycle). The SIMO rails
 * share one inductor, so PMIC-side load transients can ripple into the ADS
 * analog supplies - but which activity exactly (I2C traffic, AMUX buffer,
 * SAADC, IMON monitor, charger toggling) is unknown.
 *
 * Method: while ExG streams, the production telemetry is gated off
 * (pwr_set_measurement_gate in main.c), so this thread is the ONLY source
 * of PMIC activity. It runs a fixed schedule of isolated activities:
 *
 *   t=0            ExG stream start (detected)
 *   0     - 30 s   baseline, no PMIC activity
 *   then per phase i = 1..8:  10 s quiet gap + 30 s activity every 3 s
 *     phase 1:  40 -  70 s  I2C status reads only (no AMUX/ADC)
 *     phase 2:  80 - 110 s  AMUX switch only (BATT_V on/off, no SAADC)
 *     phase 3: 120 - 150 s  SAADC conversion only (AMUX off)
 *     phase 4: 160 - 190 s  full BATT_V measurement (AMUX + SAADC)
 *     phase 5: 200 - 230 s  full VSYS measurement (AMUX + SAADC)
 *     phase 6: 240 - 270 s  discharge-current measurement (IMON monitor)
 *     phase 7: 280 - 310 s  charger pause/resume (350 ms, as in production)
 *     phase 8: 320 - 350 s  full production-like telemetry cycle
 *   sweep repeats while streaming continues.
 *
 * Every phase transition and repetition is logged with the time since
 * stream start; the ExG packets carry µs timestamps on the same clock, so
 * noise bursts in the recording can be matched to phases either via the
 * console log or purely from the recording using the schedule above.
 *
 * Copyright (C) 2026, ETH Zurich and University of Bologna.
 *
 * ----------------------------------------------------------------------
 * SPDX-License-Identifier: Apache-2.0
 */

#include "afe/ads_appl.h"
#include "bsp/pmic_bsp.h"
#include "pwr/pwr_common.h"

#include "max77654.h"
#include "max77654_reg.h"

#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(pmic_noise_test, LOG_LEVEL_INF);

#define PHASE_BASELINE_MS 30000
#define PHASE_GAP_MS 10000
#define PHASE_ACTIVE_MS 30000
#define REP_INTERVAL_MS 3000
#define REPS_PER_PHASE (PHASE_ACTIVE_MS / REP_INTERVAL_MS)

static int64_t stream_t0;

static bool streaming(void) { return ads_get_function() == ADS_READ; }

static uint32_t t_ms(void) { return (uint32_t)(k_uptime_get() - stream_t0); }

/* Sleep in small steps; returns false if streaming stopped meanwhile. */
static bool wait_while_streaming(int32_t ms) {
  while (ms > 0) {
    if (!streaming()) {
      return false;
    }
    int32_t step = MIN(ms, 200);
    k_msleep(step);
    ms -= step;
  }
  return streaming();
}

/* ======== Isolated activities (all run while holding pwr_mutex) ========= */

// Phase 1: pure I2C register reads, nothing inside the PMIC changes state
// except the interrupt-flag clear-on-read (as in the production cycle):
static int act_i2c_status_only(void) {
  struct max77654_int flags;
  struct max77654_stat stat;
  int err = 0;
  if (max77654_get_int_flags(&pmic_h, &flags) != E_MAX77654_SUCCESS) {
    err = -1;
  }
  if (max77654_get_stat(&pmic_h, &stat) != E_MAX77654_SUCCESS) {
    err = -1;
  }
  return err;
}

// Phase 2: switch the AMUX monitor buffer on (BATT_V) and off again, but
// never sample it - isolates the PMIC-internal buffer/bias load transient:
static int act_amux_only(void) {
  uint8_t v = MAX77654__REG_CNFG_CHG_I__FIELD_MUX_SEL__CONST_BATT_V;
  int err = 0;
  if (pmic_bsp_write_regs(MAX77654__REG_CNFG_CHG_I, 1, &v) != E_MAX77654_SUCCESS) {
    err = -1;
  }
  k_msleep(2); /* on-time comparable to a real measurement's settle */
  v = MAX77654__REG_CNFG_CHG_I__FIELD_MUX_SEL__CONST_DISABLED;
  if (pmic_bsp_write_regs(MAX77654__REG_CNFG_CHG_I, 1, &v) != E_MAX77654_SUCCESS) {
    err = -1;
  }
  return err;
}

// Phase 3: one SAADC conversion on P0.07 with the AMUX kept off - isolates
// the nRF-side ADC activity (sampling, charge injection, supply blip):
static int act_saadc_only(void) {
  struct max77654_adc_reading r;
  return (pmic_bsp_adc_read(&r) == E_MAX77654_SUCCESS) ? 0 : -1;
}

// Phases 4/5: complete measurements as production performs them:
static int act_measure_batt_v(void) {
  uint32_t v;
  return (max77654_measure(&pmic_h, MAX77654_BATT_V, &v) == E_MAX77654_SUCCESS) ? 0 : -1;
}

static int act_measure_vsys(void) {
  uint32_t v;
  return (max77654_measure(&pmic_h, MAX77654_VSYS, &v) == E_MAX77654_SUCCESS) ? 0 : -1;
}

// Phase 6: discharge-current measurement - additionally exercises the
// IMON discharge monitor (sense path + scale selection):
static int act_measure_discharge(void) {
  uint32_t v;
  return (max77654_measure(&pmic_h, MAX77654_BATT_I_103MA4, &v) == E_MAX77654_SUCCESS) ? 0 : -1;
}

// Phase 7: charger pause/resume exactly as the rest-voltage sampling does
// (only has an electrical effect while a charger is attached):
static int act_charger_pause(void) {
  int err = 0;
  if (max77654_set_charger_enabled(&pmic_h, false) != E_MAX77654_SUCCESS) {
    err = -1;
  }
  k_msleep(350);
  if (max77654_set_charger_enabled(&pmic_h, true) != E_MAX77654_SUCCESS) {
    err = -1;
  }
  return err;
}

// Phase 8: approximation of one full production telemetry cycle:
static int act_full_cycle(void) {
  int err = act_i2c_status_only();
  err |= act_measure_vsys();
  for (int i = 0; i < 3; i++) {
    err |= act_measure_batt_v();
    k_msleep(1);
  }
  return err;
}

struct test_phase {
  const char *name;
  int (*act)(void);
};

static const struct test_phase phases[] = {
    {"I2C status reads only (no AMUX/ADC)", act_i2c_status_only},
    {"AMUX switch only (BATT_V on/off, no SAADC)", act_amux_only},
    {"SAADC conversion only (AMUX off)", act_saadc_only},
    {"full BATT_V measurement (AMUX+SAADC)", act_measure_batt_v},
    {"full VSYS measurement (AMUX+SAADC)", act_measure_vsys},
    {"discharge-current measurement (IMON)", act_measure_discharge},
    {"charger pause/resume (350 ms)", act_charger_pause},
    {"full production-like cycle", act_full_cycle},
};

/* Run one phase; returns false if streaming stopped. */
static bool run_phase(uint32_t idx) {
  LOG_INF("t=%6u ms: phase %u GAP (quiet %u ms)", t_ms(), idx + 1, PHASE_GAP_MS);
  if (!wait_while_streaming(PHASE_GAP_MS)) {
    return false;
  }

  LOG_INF("t=%6u ms: phase %u ACTIVE: %s (every %u ms)", t_ms(), idx + 1, phases[idx].name, REP_INTERVAL_MS);

  for (uint32_t rep = 0; rep < REPS_PER_PHASE; rep++) {
    if (!streaming()) {
      return false;
    }

    if (k_mutex_lock(&pwr_mutex, K_MSEC(400)) == 0) {
      int err = phases[idx].act();
      k_mutex_unlock(&pwr_mutex);
      LOG_INF("t=%6u ms: phase %u rep %2u%s", t_ms(), idx + 1, rep + 1, err ? " ERROR" : "");
    } else {
      LOG_WRN("t=%6u ms: phase %u rep %2u skipped (pwr mutex busy)", t_ms(), idx + 1, rep + 1);
    }

    if (!wait_while_streaming(REP_INTERVAL_MS)) {
      return false;
    }
  }
  return true;
}

static void pmic_noise_test_thread(void *a, void *b, void *c) {
  LOG_WRN("PMIC NOISE TEST build - not for production use!");
  LOG_INF("Schedule per sweep after ExG stream start: 30 s baseline, then 8 phases of 10 s gap + 30 s activity");

  while (1) {
    while (!streaming()) {
      k_msleep(200);
    }

    stream_t0 = k_uptime_get();
    LOG_INF("=== ExG streaming detected, sweep starts (t=0, uptime %lld ms) ===", stream_t0);
    LOG_INF("t=     0 ms: BASELINE (quiet %u ms)", PHASE_BASELINE_MS);

    bool ok = wait_while_streaming(PHASE_BASELINE_MS);

    for (uint32_t i = 0; ok && i < ARRAY_SIZE(phases); i++) {
      ok = run_phase(i);
    }

    if (ok) {
      LOG_INF("=== t=%6u ms: sweep complete, repeating ===", t_ms());
    } else {
      LOG_INF("=== ExG streaming stopped, test paused ===");
    }
  }
}

K_THREAD_DEFINE(pmic_noise_test_tid, 2048, pmic_noise_test_thread, NULL, NULL, NULL, 10, 0, 0);
