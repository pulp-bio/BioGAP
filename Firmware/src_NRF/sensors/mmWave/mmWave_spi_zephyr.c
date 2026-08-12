/*
 * ----------------------------------------------------------------------
 *
 * File: mmWave_spi_zephyr.c
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
 * @file mmWave_spi_zephyr.c
 * @brief BGT60TR13C SPI transport using the Zephyr SPI subsystem
 *
 * Alternative to mmWave_spi.c, selected by CONFIG_MMWAVE_ZEPHYR_SPI. It is a
 * deliberate, close reproduction of the transport from the standalone
 * BGT60TR13C firmware that is known to work on this hardware: Zephyr's SPI
 * driver owns SPIM4 outright, the radar is the only device on the bus, and the
 * bus is configured once from the devicetree rather than switched per
 * transaction.
 *
 * The trade-off is exclusivity. Zephyr's driver and BioGAP's raw nrfx SPI_A
 * layer cannot both own SPIM4, so a build using this transport has no working
 * ADS1298 (ExG) -- init_spi_a_bus() deliberately does nothing. Use it to bring
 * the radar up and to acquire radar-only data; mmWave_spi.c remains the path
 * for sharing the bus with ExG.
 *
 * The API is identical to mmWave_spi.h so sensors/mmWave/mmWave_appl.c is
 * unchanged between the two transports.
 */

#include "sensors/mmWave/mmWave_spi.h"

#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/spi.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>

#include "sensors/mmWave/mmWave_config.h"
#include "sensors/mmWave/driver/xensiv_bgt60trxx.h"
/* Declares the platform callbacks implemented at the bottom of this file, so
 * the compiler checks them against the signatures the driver expects. */
#include "sensors/mmWave/driver/xensiv_bgt60trxx_platform.h"

LOG_MODULE_REGISTER(mmWave_spi, LOG_LEVEL_INF);

/*==============================================================================
 * Private Definitions
 *============================================================================*/

/*
 * Mode 0 (CPOL=0, CPHA=0), 8-bit words, MSB first. Neither SPI_MODE_CPOL nor
 * SPI_MODE_CPHA is set, which is what the reference implementation used. The
 * clock comes from the node's spi-max-frequency in the shield overlay.
 */
#define MMWAVE_SPI_OPERATION (SPI_WORD_SET(8) | SPI_TRANSFER_MSB)

/*==============================================================================
 * Private Variables
 *============================================================================*/

/** @brief Bus handle built from the devicetree node */
static const struct spi_dt_spec spi_cfg =
    SPI_DT_SPEC_GET(DT_NODELABEL(bgt60), MMWAVE_SPI_OPERATION, 0);

/** @brief Chip select, driven in software: the controller has no cs-gpios, and
 *  the radar's protocol needs CS held across multi-transfer transactions. */
static const struct gpio_dt_spec cs_pin =
    GPIO_DT_SPEC_GET(DT_NODELABEL(bgt60), manual_cs_gpios);

/** @brief Last FIFO burst in the radar's native 12-bit packing */
static uint8_t packed_frame[MMWAVE_FRAME_SIZE_BYTES_PACKED] __aligned(4);

/* The Infineon driver stores this and hands it back to every platform callback.
 * Here it really is the bus handle, matching the reference implementation. */
void *const mmwave_spi_iface = (void *)&spi_cfg;

/*==============================================================================
 * Public Functions
 *============================================================================*/

int mmwave_spi_init(void) {
  if (!gpio_is_ready_dt(&cs_pin)) {
    LOG_ERR("mmWave CS GPIO not ready");
    return -ENODEV;
  }

  /* Idle level = deasserted. The pin is flagged active-low in the devicetree,
   * so GPIO_OUTPUT_ACTIVE drives it physically low, keeping the radar's inputs
   * at 0 V while its power rail is still off. */
  int ret = gpio_pin_configure_dt(&cs_pin, GPIO_OUTPUT_ACTIVE);
  if (ret < 0) {
    LOG_ERR("Failed to configure mmWave CS GPIO: %d", ret);
    return ret;
  }

  if (!spi_is_ready_dt(&spi_cfg)) {
    LOG_ERR("SPI bus not ready - is CONFIG_SPI=y and &spi_a enabled?");
    return -ENODEV;
  }

  LOG_INF("mmWave SPI transport ready (Zephyr SPI, mode 0, %u Hz, exclusive bus)",
          spi_cfg.config.frequency);
  return 0;
}

void mmwave_spi_set_clock_mhz(uint8_t freq_mhz) {
  /* The Zephyr transport takes its clock from the node's spi-max-frequency, so
   * there is nothing to override at runtime. */
  ARG_UNUSED(freq_mhz);
  LOG_WRN("Radar SPI clock is fixed by the devicetree in this build (%u Hz)",
          spi_cfg.config.frequency);
}

void mmwave_spi_log_bus_state(void) {
  LOG_INF("SPI_A (Zephyr driver): frequency=%u Hz operation=0x%08x slave=%u",
          spi_cfg.config.frequency, (unsigned int)spi_cfg.config.operation,
          (unsigned int)spi_cfg.config.slave);
}

void mmwave_spi_park_cs(void) { gpio_pin_set_raw(cs_pin.port, cs_pin.pin, 0); }

/* Only reached through the shared nrfx SPI_A interrupt dispatcher, which this
 * transport never enrols in. Defined so spi/spi_a.c still links. */
void mmwave_spim_transfer_complete(void) {}

const uint8_t *mmwave_spi_get_packed_frame(void) { return packed_frame; }

/*==============================================================================
 * xensiv_bgt60trxx platform callbacks
 *============================================================================*/

void xensiv_bgt60trxx_platform_spi_cs_set(const void *iface, bool val) {
  ARG_UNUSED(iface);

  /* The driver passes the raw level it wants on the pin: false selects the
   * device, true releases it. Written raw so the polarity matches the net name
   * (CS_n) rather than the devicetree's logical sense. */
  gpio_pin_set_raw(cs_pin.port, cs_pin.pin, val ? 1 : 0);
}

int32_t xensiv_bgt60trxx_platform_spi_transfer(void *iface, uint8_t *tx_data,
                                              uint8_t *rx_data, uint32_t len) {
  const struct spi_dt_spec *spi = (const struct spi_dt_spec *)iface;

  struct spi_buf tx_buf = {.buf = tx_data, .len = tx_data ? len : 0};
  struct spi_buf_set tx_set = {.buffers = &tx_buf, .count = tx_data ? 1 : 0};

  struct spi_buf rx_buf = {.buf = rx_data, .len = rx_data ? len : 0};
  struct spi_buf_set rx_set = {.buffers = &rx_buf, .count = rx_data ? 1 : 0};

  int err = spi_transceive_dt(spi, tx_data ? &tx_set : NULL,
                              rx_data ? &rx_set : NULL);

  return (err == 0) ? XENSIV_BGT60TRXX_STATUS_OK
                    : XENSIV_BGT60TRXX_STATUS_COM_ERROR;
}

int32_t xensiv_bgt60trxx_platform_spi_fifo_read(void *iface, uint16_t *rx_data,
                                                uint32_t len) {
  const struct spi_dt_spec *spi = (const struct spi_dt_spec *)iface;

  /* The FIFO delivers 12-bit samples, two packed into every three bytes. */
  uint32_t byte_len = (len * 3U) / 2U;

  if (byte_len > sizeof(packed_frame)) {
    LOG_ERR("FIFO burst of %u bytes exceeds the %u-byte frame buffer", byte_len,
            (uint32_t)sizeof(packed_frame));
    return XENSIV_BGT60TRXX_STATUS_COM_ERROR;
  }

  /* Receive-only: Zephyr clocks the bus and drives MOSI with the controller's
   * overrun character, which the nRF SPIM takes from the bus node's
   * overrun-character property (0xFF by default). The radar requires MOSI high
   * throughout a FIFO burst, so that default is what makes this correct. */
  struct spi_buf rx_buf = {.buf = packed_frame, .len = byte_len};
  struct spi_buf_set rx_set = {.buffers = &rx_buf, .count = 1};

  if (spi_read_dt(spi, &rx_set) != 0) {
    return XENSIV_BGT60TRXX_STATUS_COM_ERROR;
  }

  /* Unpack for the driver (and for test-mode validation); the packed form stays
   * in packed_frame for the BLE path. */
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
