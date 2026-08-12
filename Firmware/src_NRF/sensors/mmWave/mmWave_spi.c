/*
 * ----------------------------------------------------------------------
 *
 * File: mmWave_spi.c
 *
 * Copyright (C) 2026, ETH Zurich
 *
 * Authors:
 * - Benjamin Löliger, ETH Zurich
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
 * @file mmWave_spi.c
 * @brief BGT60TR13C SPI transport over the shared SPI_A bus
 */

#include "sensors/mmWave/mmWave_spi.h"

#include <zephyr/drivers/gpio.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <nrfx_spim.h>

#include "sensors/mmWave/mmWave_config.h"
#include "sensors/mmWave/driver/xensiv_bgt60trxx.h"
/* Declares the platform callbacks implemented at the bottom of this file, so
 * the compiler checks them against the signatures the driver expects. */
#include "sensors/mmWave/driver/xensiv_bgt60trxx_platform.h"
#include "spi/spi_a.h"

LOG_MODULE_REGISTER(mmWave_spi, LOG_LEVEL_INF);

/*==============================================================================
 * Private Definitions
 *============================================================================*/

/** @brief SPI mode of the BGT60TR13C (CPOL=0, CPHA=0) */
#define MMWAVE_SPI_MODE NRF_SPIM_MODE_0

/** @brief Default SPI clock for radar transfers, from Kconfig */
#if CONFIG_MMWAVE_SPI_FREQ_MHZ == 1
#define MMWAVE_SPI_FREQ_DEFAULT NRF_SPIM_FREQ_1M
#elif CONFIG_MMWAVE_SPI_FREQ_MHZ == 2
#define MMWAVE_SPI_FREQ_DEFAULT NRF_SPIM_FREQ_2M
#elif CONFIG_MMWAVE_SPI_FREQ_MHZ == 4
#define MMWAVE_SPI_FREQ_DEFAULT NRF_SPIM_FREQ_4M
#else
#define MMWAVE_SPI_FREQ_DEFAULT NRF_SPIM_FREQ_8M
#endif

/** @brief Timeout for a single SPI transfer to complete */
#define MMWAVE_SPI_XFER_TIMEOUT_MS 100

/** @brief How long to wait for another consumer's transfer to finish before
 *  rewriting the bus configuration (ADS1298 transfers take tens of us) */
#define MMWAVE_BUS_IDLE_TIMEOUT_US 500

/* The configured clock must not exceed what the shield overlay advertises. */
BUILD_ASSERT(CONFIG_MMWAVE_SPI_FREQ_MHZ * 1000000 <=
                 DT_PROP(DT_NODELABEL(bgt60), spi_max_frequency),
             "CONFIG_MMWAVE_SPI_FREQ_MHZ exceeds the bgt60 spi-max-frequency");

/*==============================================================================
 * Private Variables
 *============================================================================*/

/** @brief Chip select, driven in software because SPI_A is shared */
static const struct gpio_dt_spec cs_pin =
    GPIO_DT_SPEC_GET(DT_NODELABEL(bgt60), manual_cs_gpios);

/** @brief Signalled from the shared SPI_A ISR when our transfer completes */
static K_SEM_DEFINE(mmwave_spi_done, 0, 1);

/** @brief True while this module holds spi_a_mutex (i.e. CS is asserted).
 *
 * xensiv_bgt60trxx_hard_reset() deasserts CS without a matching assert, so the
 * unlock path has to tolerate being called without holding the lock. */
static bool bus_held;

/** @brief Last FIFO burst in the radar's native 12-bit packing */
static uint8_t packed_frame[MMWAVE_FRAME_SIZE_BYTES_PACKED] __aligned(4);

/** @brief Backing object for the opaque interface handle (see mmWave_spi.h) */
static uint8_t iface_token;
void *const mmwave_spi_iface = &iface_token;

/** @brief SPI clock currently used for radar transfers */
static nrf_spim_frequency_t mmwave_spi_freq = MMWAVE_SPI_FREQ_DEFAULT;

/* Peripheral state captured at the instant the last transfer completed, while
 * the radar still owned the bus. Sampling it later is useless: the chip-select
 * callback has by then restored the ADS1298's mode and clock, so a log taken
 * after the fact reports the wrong configuration. RXD/TXD AMOUNT are what
 * actually distinguish "bytes moved but MISO read low" (a power or wiring
 * problem at the shield) from "no bytes moved" (a driver problem). */
static uint32_t last_enable;
static uint32_t last_config;
static uint32_t last_frequency;
static uint32_t last_rx_amount;
static uint32_t last_tx_amount;

/*==============================================================================
 * Private Functions
 *============================================================================*/

/**
 * @brief Spin until no transfer is in flight on SPI_A
 *
 * Holding spi_a_mutex is not enough: the ADS1298 driver releases it as soon as
 * it has queued its async transfer, so one may still be running. Rewriting the
 * peripheral's mode and clock under a live transfer would corrupt it, so both
 * the switch to the radar's configuration and the restore of the default wait
 * here first. ADS transfers are tens of microseconds, so this normally does not
 * spin at all.
 */
static void wait_for_bus_idle(void) {
  for (int i = 0; i < MMWAVE_BUS_IDLE_TIMEOUT_US && spi_a_transfer_in_flight();
       i++) {
    k_busy_wait(1);
  }

  if (spi_a_transfer_in_flight()) {
    LOG_WRN("SPI_A still busy after %d us; reconfiguring anyway",
            MMWAVE_BUS_IDLE_TIMEOUT_US);
  }
}

/**
 * @brief Run one nrfx transfer on SPI_A and wait for it to finish
 *
 * The caller must hold spi_a_mutex (guaranteed by the CS callback) and must
 * have switched the bus to the radar's configuration.
 *
 * @param p_xfer nrfx transfer descriptor
 *
 * @return XENSIV_BGT60TRXX_STATUS_OK on success,
 *         XENSIV_BGT60TRXX_STATUS_COM_ERROR on failure or timeout
 */
static int32_t mmwave_spi_xfer(const nrfx_spim_xfer_desc_t *p_xfer) {
  k_sem_reset(&mmwave_spi_done);

  /* CS is already asserted by the driver's cs_set callback, so pass NULL and
   * keep it asserted across the whole transaction. */
  spi_a_begin_transfer(SPI_A_OWNER_MMWAVE, NULL);

  nrfx_err_t status = nrfx_spim_xfer(&spi_a_inst, p_xfer, 0);
  if (status != NRFX_SUCCESS) {
    LOG_ERR("nrfx_spim_xfer failed: 0x%08x", (unsigned int)status);
    return XENSIV_BGT60TRXX_STATUS_COM_ERROR;
  }

  if (k_sem_take(&mmwave_spi_done, K_MSEC(MMWAVE_SPI_XFER_TIMEOUT_MS)) != 0) {
    LOG_ERR("SPI transfer timed out");
    return XENSIV_BGT60TRXX_STATUS_COM_ERROR;
  }

  /* Snapshot while we still hold the bus in the radar's configuration. */
  NRF_SPIM_Type *reg = spi_a_inst.p_reg;
  last_enable = reg->ENABLE;
  last_config = reg->CONFIG;
  last_frequency = reg->FREQUENCY;
  last_rx_amount = reg->RXD.AMOUNT;
  last_tx_amount = reg->TXD.AMOUNT;

  return XENSIV_BGT60TRXX_STATUS_OK;
}

/*==============================================================================
 * Public Functions
 *============================================================================*/

int mmwave_spi_init(void) {
  if (!gpio_is_ready_dt(&cs_pin)) {
    LOG_ERR("mmWave CS GPIO not ready");
    return -ENODEV;
  }

  /* Idle level = deasserted. The pin is flagged active-low in the devicetree,
   * so GPIO_OUTPUT_ACTIVE drives it physically low, which keeps the radar's
   * inputs at 0 V while its power rail is still off. */
  int ret = gpio_pin_configure_dt(&cs_pin, GPIO_OUTPUT_ACTIVE);
  if (ret < 0) {
    LOG_ERR("Failed to configure mmWave CS GPIO: %d", ret);
    return ret;
  }

  ret = init_spi_a_bus();
  if (ret != 0) {
    LOG_ERR("Failed to bring up SPI_A: %d", ret);
    return ret;
  }

  LOG_INF("mmWave SPI transport ready (SPI_A, mode 0, 8 MHz)");
  return 0;
}

void mmwave_spi_set_clock_mhz(uint8_t freq_mhz) {
  switch (freq_mhz) {
  case 1: mmwave_spi_freq = NRF_SPIM_FREQ_1M; break;
  case 2: mmwave_spi_freq = NRF_SPIM_FREQ_2M; break;
  case 4: mmwave_spi_freq = NRF_SPIM_FREQ_4M; break;
  case 8: mmwave_spi_freq = NRF_SPIM_FREQ_8M; break;
  default:
    LOG_WRN("Unsupported radar SPI clock %u MHz - keeping current setting",
            freq_mhz);
    return;
  }
  LOG_INF("Radar SPI clock set to %u MHz", freq_mhz);
}

void mmwave_spi_log_bus_state(void) {
  NRF_SPIM_Type *reg = spi_a_inst.p_reg;

  /* Current (idle) state. After a radar transaction the chip-select callback
   * has restored the ADS1298's settings, so mode 1 / 4 MHz here is expected and
   * says nothing about how the radar's transfers ran. PSEL should read 8/9/10. */
  LOG_INF("SPI_A now (restored): ENABLE=%u FREQUENCY=0x%08x CONFIG=0x%08x "
          "SCK=%u MOSI=%u MISO=%u",
          (unsigned int)reg->ENABLE, (unsigned int)reg->FREQUENCY,
          (unsigned int)reg->CONFIG, (unsigned int)reg->PSEL.SCK,
          (unsigned int)reg->PSEL.MOSI, (unsigned int)reg->PSEL.MISO);

  /* State at the end of the last radar transfer, which is the one that matters.
   *   ENABLE should be 7 (SPIM enabled); 0 means no transfer ran.
   *   CONFIG should be 0 (mode 0, MSB first); 2 means the bus was still in the
   *     ADS1298's mode 1, i.e. the reconfigure did not take effect.
   *   FREQUENCY 0x80000000 = 8 MHz, 0x40000000 = 4 MHz.
   *   RX/TX AMOUNT are the byte counts EasyDMA actually moved: 4 and 4 for a
   *     register access. Non-zero AMOUNT with an all-zero result means the bus
   *     ran correctly and the radar simply did not drive MISO -- look at the
   *     shield's supply and the enable pin, not at the firmware. */
  LOG_INF("SPI_A at last radar xfer: ENABLE=%u FREQUENCY=0x%08x CONFIG=0x%08x "
          "RXD.AMOUNT=%u TXD.AMOUNT=%u",
          (unsigned int)last_enable, (unsigned int)last_frequency,
          (unsigned int)last_config, (unsigned int)last_rx_amount,
          (unsigned int)last_tx_amount);
}

void mmwave_spi_park_cs(void) { gpio_pin_set_raw(cs_pin.port, cs_pin.pin, 0); }

void mmwave_spim_transfer_complete(void) { k_sem_give(&mmwave_spi_done); }

const uint8_t *mmwave_spi_get_packed_frame(void) { return packed_frame; }

/*==============================================================================
 * xensiv_bgt60trxx platform callbacks
 *============================================================================*/

void xensiv_bgt60trxx_platform_spi_cs_set(const void *iface, bool val) {
  ARG_UNUSED(iface);

  /* The driver calls this with val=false to select the device and val=true to
   * release it, and the pin is written raw so the polarity matches the net
   * name (CS_n): low = selected. */
  if (!val) {
    k_mutex_lock(&spi_a_mutex, K_FOREVER);
    bus_held = true;

#if !defined(CONFIG_MMWAVE_SPI_STATIC_MODE)
    wait_for_bus_idle();
    spi_a_reconfigure(MMWAVE_SPI_MODE, mmwave_spi_freq);
#endif
    gpio_pin_set_raw(cs_pin.port, cs_pin.pin, 0);
  } else {
    gpio_pin_set_raw(cs_pin.port, cs_pin.pin, 1);
    if (bus_held) {
#if !defined(CONFIG_MMWAVE_SPI_STATIC_MODE)
      /* Also wait here: after a transfer timeout in mmwave_spi_xfer() ours may
       * still be running, and restoring the configuration under it would
       * corrupt the tail of the transfer. */
      wait_for_bus_idle();
      spi_a_restore_default_config();
#endif
      bus_held = false;
      k_mutex_unlock(&spi_a_mutex);
    }
  }
}

int32_t xensiv_bgt60trxx_platform_spi_transfer(void *iface, uint8_t *tx_data,
                                              uint8_t *rx_data, uint32_t len) {
  ARG_UNUSED(iface);

  nrfx_spim_xfer_desc_t xfer = {
      .p_tx_buffer = tx_data,
      .tx_length = (tx_data != NULL) ? len : 0,
      .p_rx_buffer = rx_data,
      .rx_length = (rx_data != NULL) ? len : 0,
  };

  return mmwave_spi_xfer(&xfer);
}

int32_t xensiv_bgt60trxx_platform_spi_fifo_read(void *iface, uint16_t *rx_data,
                                                uint32_t len) {
  ARG_UNUSED(iface);

  /* The FIFO delivers 12-bit samples, two packed into every three bytes. */
  uint32_t byte_len = (len * 3U) / 2U;

  if (byte_len > sizeof(packed_frame)) {
    LOG_ERR("FIFO burst of %u bytes exceeds the %u-byte frame buffer", byte_len,
            (uint32_t)sizeof(packed_frame));
    return XENSIV_BGT60TRXX_STATUS_COM_ERROR;
  }

  nrfx_spim_xfer_desc_t xfer = {
      .p_tx_buffer = NULL,
      .tx_length = 0,
      .p_rx_buffer = packed_frame,
      .rx_length = byte_len,
  };

  int32_t status = mmwave_spi_xfer(&xfer);
  if (status != XENSIV_BGT60TRXX_STATUS_OK) {
    return status;
  }

  /* Unpack for the driver (and for test-mode validation); the packed form is
   * kept in packed_frame for the BLE path. */
  uint32_t byte_idx = 0;
  for (uint32_t i = 0; i < len; i += 2) {
    rx_data[i] = (uint16_t)((packed_frame[byte_idx] << 4) |
                            (packed_frame[byte_idx + 1] >> 4));

    if ((i + 1U) < len) {
      rx_data[i + 1] = (uint16_t)(((packed_frame[byte_idx + 1] & 0x0F) << 8) |
                                  packed_frame[byte_idx + 2]);
    }
    byte_idx += 3U;
  }

  return XENSIV_BGT60TRXX_STATUS_OK;
}
