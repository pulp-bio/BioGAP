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
 *  context (spi_a_current_owner()), so must be volatile. */
static volatile spi_a_owner_t current_owner = SPI_A_OWNER_NONE;

/** @brief CS line to auto-deassert on completion, or NULL if the owner
 *  manages its own CS deassertion */
static const struct gpio_dt_spec *current_cs = NULL;

/** @brief Set once nrfx_spim_init() has succeeded, so init_spi_a_bus() is
 *  safe to call from every shield that sits on SPI_A regardless of
 *  init order, without re-initializing an already-live peripheral. */
static bool spi_a_initialized = false;

/* Each consumer owns its own completion handling (CS deassertion for
 * multi-CS owners, data processing, semaphore signalling, ...); this
 * module only routes the single nrfx completion event to the right one. */
extern void ads_spim_transfer_complete(void);
#if defined(CONFIG_WI_FI)
extern void wifi_sd_spim_transfer_complete(void);
#endif

/** @brief Set by ads_spi_data.c; true once the in-flight ADS transfer is
 *  the cycle's last one, so this handler knows when it's safe to release
 *  current_owner back to SPI_A_OWNER_NONE. */
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
    default:
      LOG_WRN("SPI_A transfer completed with no registered owner");
      break;
  }
  // Make sure that ADS has completed transfers both from A and B
  if ((current_owner == SPI_A_OWNER_ADS) & (!ads_a_and_b_done)){
    current_owner = SPI_A_OWNER_ADS;
  }
  else{
    // No Owner, others can take the SPI bus 
    current_owner = SPI_A_OWNER_NONE;
  }
}

int init_spi_a_bus(void) {
  nrfx_err_t status;

  if (spi_a_initialized) {
    return 0;
  }

#if defined(__ZEPHYR__)
  IRQ_CONNECT(NRFX_IRQ_NUMBER_GET(NRF_SPIM_INST_GET(SPI_A_INST_IDX)), SPI_A_INT_PRIO,
              NRFX_SPIM_INST_HANDLER_GET(SPI_A_INST_IDX), 0, 0);
#endif

  nrfx_spim_config_t config =
      NRFX_SPIM_DEFAULT_CONFIG(SPI_A_SCK_PIN, SPI_A_MOSI_PIN, SPI_A_MISO_PIN, NRF_SPIM_PIN_NOT_CONNECTED);
  config.frequency = NRFX_MHZ_TO_HZ(4);
  config.mode = NRF_SPIM_MODE_1;
  config.bit_order = NRF_SPIM_BIT_ORDER_MSB_FIRST;
  config.irq_priority = SPI_A_INT_PRIO;

  status = nrfx_spim_init(&spi_a_inst, &config, spi_a_event_handler, NULL);
  if (status != NRFX_SUCCESS) {
    LOG_ERR("SPI_A (SPIM%d) init failed: %d", SPI_A_INST_IDX, status);
    return -1;
  }

  spi_a_initialized = true;
  LOG_INF("SPI_A (SPIM%d) initialized: SCK=P0.%02d MOSI=P0.%02d MISO=P0.%02d", SPI_A_INST_IDX, SPI_A_SCK_PIN,
          SPI_A_MOSI_PIN, SPI_A_MISO_PIN);
  return 0;
}

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
