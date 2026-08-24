/*
 * ----------------------------------------------------------------------
 *
 * File: spi_a.c
 *
 * Copyright (C) 2026, ETH Zurich
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

/**
 * @file spi_a.c
 * @brief Shared SPI_A bus (nrfx SPIM4) initialization and transfer dispatch
 */

#include "spi/spi_a.h"

#include <zephyr/logging/log.h>
#include "zephyr/irq.h"

LOG_MODULE_REGISTER(spi_a, LOG_LEVEL_INF);

nrfx_spim_t spi_a_inst = NRFX_SPIM_INSTANCE(SPI_A_INST_IDX);
K_MUTEX_DEFINE(spi_a_mutex);

/** @brief Owner of the in-flight transfer, set by spi_a_begin_transfer().
 *  Written from ISR context (spi_a_event_handler) and polled from thread
 *  context (spi_a_current_owner(), spi_a_transfer_in_flight()), so must be
 *  volatile. */
static volatile spi_a_owner_t current_owner = SPI_A_OWNER_NONE;

/** @brief Diagnostic-only: which step of a SPI_A transaction is currently in
 *  progress, tracked separately per subsystem (ads_spi_comm.c/ads_spi_hw.c
 *  vs wifi_sd_spi_functions.c) since ADS reads happen far more often than
 *  WiFi sends -- a single shared variable would almost always show a recent
 *  ADS value regardless of which side actually stalled. Purely
 *  informational -- read by the periodic status heartbeat
 *  (wifi_sd_shield_appl.c). See spi_a_checkpoint_t. */
static volatile uint8_t ads_checkpoint = SPI_A_CP_NONE;
static volatile uint8_t wifi_checkpoint = SPI_A_CP_NONE;

void spi_a_set_ads_checkpoint(uint8_t cp) {
  ads_checkpoint = cp;
}

uint8_t spi_a_get_ads_checkpoint(void) {
  return ads_checkpoint;
}

void spi_a_set_wifi_checkpoint(uint8_t cp) {
  wifi_checkpoint = cp;
}

uint8_t spi_a_get_wifi_checkpoint(void) {
  return wifi_checkpoint;
}

/** @brief CS line to auto-deassert on completion, or NULL if the owner
 *  manages its own CS deassertion */
static const struct gpio_dt_spec *current_cs = NULL;

/** @brief Set once nrfx_spim_init() has succeeded, so init_spi_a_bus() is
 *  safe to call from every shield that sits on SPI_A regardless of
 *  init order, without re-initializing an already-live peripheral. */
static bool spi_a_initialized = false;

/* The completion dispatcher exists only when this module owns the peripheral.
 * With CONFIG_MMWAVE_ZEPHYR_SPI the Zephyr SPI driver owns SPIM4 and installs
 * its own handler, so ours would be dead code. */
#if !defined(CONFIG_MMWAVE_ZEPHYR_SPI)

/* Each consumer owns its own completion handling (CS deassertion for
 * multi-CS owners, data processing, semaphore signalling, ...); this
 * module only routes the single nrfx completion event to the right one. */
extern void ads_spim_transfer_complete(void);
#if defined(CONFIG_WI_FI)
extern void wifi_sd_spim_transfer_complete(void);
#endif
#if defined(CONFIG_SENSOR_MMWAVE)
extern void mmwave_spim_transfer_complete(void);
#endif

/** @brief Set by ads_spi_data.c; true once the in-flight ADS transfer is
 *  the current A+B pair's last one, so this handler knows when it's safe
 *  to release current_owner back to SPI_A_OWNER_NONE. */
extern volatile bool ads_a_and_b_done;

static void spi_a_event_handler(nrfx_spim_evt_t const *p_event, void *p_context) {
  ARG_UNUSED(p_context);
  if (p_event->type != NRFX_SPIM_EVENT_DONE) {
    return;
  }

  if (current_cs != NULL) {
    gpio_pin_set_dt(current_cs, 0); // deassert
    current_cs = NULL;
  }

  switch (current_owner) {
    case SPI_A_OWNER_ADS:
      ads_spim_transfer_complete();
      break;
    case SPI_A_OWNER_WIFI_SD:
#if defined(CONFIG_WI_FI)
      wifi_sd_spim_transfer_complete();
#else
      LOG_WRN("SPI_A transfer completed for WiFi/SD owner, but CONFIG_WI_FI=n");
#endif
      break;
    case SPI_A_OWNER_MMWAVE:
#if defined(CONFIG_SENSOR_MMWAVE)
      mmwave_spim_transfer_complete();
#else
      LOG_WRN("SPI_A transfer completed for mmWave owner, but CONFIG_SENSOR_MMWAVE=n");
#endif
      break;
    default:
      LOG_WRN("SPI_A transfer completed with no registered owner");
      break;
  }
  // A and B are read as a back-to-back pair once both DRDYs fire (see
  // process_ads_data()); keep the bus pinned to ADS between the two so
  // WiFi/SD can't grab it in the gap, and only release once the pair
  // is actually done.
  if (current_owner == SPI_A_OWNER_ADS && !ads_a_and_b_done) {
    current_owner = SPI_A_OWNER_ADS;
  } else {
    current_owner = SPI_A_OWNER_NONE;
  }
}

/** @brief (Re)configure and start spi_a_inst at the given frequency. Caller
 *  handles mutex/uninit as needed. */
static int spi_a_init_at(uint32_t frequency_hz) {
  nrfx_spim_config_t config =
      NRFX_SPIM_DEFAULT_CONFIG(SPI_A_SCK_PIN, SPI_A_MOSI_PIN, SPI_A_MISO_PIN, NRF_SPIM_PIN_NOT_CONNECTED);
  config.frequency = frequency_hz;
  config.mode = SPI_A_INIT_MODE;
  config.bit_order = NRF_SPIM_BIT_ORDER_MSB_FIRST;
  config.irq_priority = SPI_A_INT_PRIO;

  nrfx_err_t status = nrfx_spim_init(&spi_a_inst, &config, spi_a_event_handler, NULL);
  if (status != NRFX_SUCCESS) {
    LOG_ERR("SPI_A (SPIM%d) init failed: %d", SPI_A_INST_IDX, status);
    return -1;
  }
  return 0;
}

#endif /* !CONFIG_MMWAVE_ZEPHYR_SPI */

#if defined(CONFIG_MMWAVE_ZEPHYR_SPI)

int init_spi_a_bus(void) {
  /* Zephyr's SPI driver owns SPIM4 in this build (see CONFIG_MMWAVE_ZEPHYR_SPI):
   * it runs its own nrfx_spim_init() with its own event handler, so claiming the
   * instance here as well would fight it. Report success so callers proceed --
   * the consequence is that ADS1298 transfers do not work in a radar-only
   * image, which is the documented trade-off of that option. */
  if (!spi_a_initialized) {
    spi_a_initialized = true;
    LOG_INF("SPI_A left to the Zephyr SPI driver (radar-only build); "
            "ADS1298 transfers are not available");
  }
  return 0;
}

#else /* raw nrfx ownership of SPI_A */

int init_spi_a_bus(void) {
  if (spi_a_initialized) {
    return 0;
  }

#if defined(__ZEPHYR__)
  IRQ_CONNECT(NRFX_IRQ_NUMBER_GET(NRF_SPIM_INST_GET(SPI_A_INST_IDX)), SPI_A_INT_PRIO,
              NRFX_SPIM_INST_HANDLER_GET(SPI_A_INST_IDX), 0, 0);
#endif

  if (spi_a_init_at(SPI_A_INIT_FREQ) != 0) {
    return -1;
  }

  spi_a_initialized = true;
  LOG_INF("SPI_A (SPIM%d) initialized: SCK=P0.%02d MOSI=P0.%02d MISO=P0.%02d", SPI_A_INST_IDX, SPI_A_SCK_PIN,
          SPI_A_MOSI_PIN, SPI_A_MISO_PIN);
  return 0;
}

#endif /* CONFIG_MMWAVE_ZEPHYR_SPI */

#if defined(CONFIG_MMWAVE_ZEPHYR_SPI)
int spi_a_set_frequency(uint32_t frequency_hz) {
  /* Zephyr's SPI driver owns SPIM4 in this build and takes its clock from the
   * devicetree, so there is nothing here to retune. spi_a_init_at() does not
   * even exist. Reported as success so ExG/ESP callers stay unconditional. */
  ARG_UNUSED(frequency_hz);
  return 0;
}
#else
int spi_a_set_frequency(uint32_t frequency_hz) {
  k_mutex_lock(&spi_a_mutex, K_FOREVER);
  nrfx_spim_uninit(&spi_a_inst);
  int ret = spi_a_init_at(frequency_hz);
  k_mutex_unlock(&spi_a_mutex);
  return ret;
}
#endif

void spi_a_begin_transfer(spi_a_owner_t owner, const struct gpio_dt_spec *cs) {
  current_owner = owner;
  current_cs = cs;
  if (cs != NULL) {
    gpio_pin_set_dt(cs, 1); // assert
  }
}

spi_a_owner_t spi_a_current_owner(void) {
  return current_owner;
}

bool spi_a_transfer_in_flight(void) {
  return spi_a_current_owner() != SPI_A_OWNER_NONE;
}

void spi_a_reconfigure(nrf_spim_mode_t mode, nrf_spim_frequency_t frequency) {
  nrf_spim_configure(spi_a_inst.p_reg, mode, NRF_SPIM_BIT_ORDER_MSB_FIRST);
  nrf_spim_frequency_set(spi_a_inst.p_reg, frequency);
  /* Same barrier nrfy_spim_periph_configure() ends with: make sure the CONFIG
   * and FREQUENCY writes have left the write buffer before the caller starts a
   * transfer, otherwise the first bytes could still be clocked with the
   * previous owner's mode. */
  nrf_barrier_w();
}

void spi_a_save_config(spi_a_config_t *out) {
  out->config = spi_a_inst.p_reg->CONFIG;
  out->frequency = spi_a_inst.p_reg->FREQUENCY;
}

void spi_a_restore_config(const spi_a_config_t *cfg) {
  /* Written back raw rather than through nrf_spim_configure(), so bit order is
   * preserved along with mode and clock whatever the previous owner had set. */
  spi_a_inst.p_reg->CONFIG = cfg->config;
  spi_a_inst.p_reg->FREQUENCY = cfg->frequency;
  nrf_barrier_w();
}
