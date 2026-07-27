/*
 * ----------------------------------------------------------------------
 *
 * File: spi_a.h
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
 * @file spi_a.h
 * @brief Shared SPI_A bus (nrfx SPIM4) initialization and transfer dispatch
 *
 * SPI_A is the BioGAP mainboard's high-speed SPI bus (SCK=P0.08, MOSI=P0.09,
 * MISO=P0.10, driven by nrfx SPIM instance 4). It is physically shared by
 * every device that sits on it -- currently the two ADS1298 AFEs (ExG
 * shield) and the ESP32-C6 / SD card (WiFi/SD shield). Each device gets its
 * own software-toggled CS GPIO; there is no hardware-managed CS on this
 * bus, since more than one device shares it.
 *
 * The nrfx SPIM driver only allows ONE event handler per peripheral
 * instance, but every consumer needs to know when *its* transfer
 * completed. This module resolves that with a single shared interrupt
 * handler that dispatches to whichever consumer kicked off the
 * in-flight transfer (tracked via spi_a_begin_transfer()). Since the bus
 * is physically serial and every consumer is expected to hold spi_a_mutex
 * for the duration of its transfer, at most one transfer -- and therefore
 * one owner -- is ever in flight at a time.
 */

#ifndef SPI_A_H
#define SPI_A_H

#include <zephyr/kernel.h>
#include <zephyr/drivers/gpio.h>
#include <nrfx_spim.h>

/** @brief SPIM instance index for SPI_A (must match spi_a_default pinctrl
 *  in the sensei-sdk board DTS: nrf5340_senseiv1_cpuapp.dts) */
#define SPI_A_INST_IDX 4

/** @brief SCK pin number for SPI_A (SPI_A_CLK = P0.08) */
#define SPI_A_SCK_PIN 8

/** @brief MOSI pin number for SPI_A (SPI_A_MOSI = P0.09) */
#define SPI_A_MOSI_PIN 9

/** @brief MISO pin number for SPI_A (SPI_A_MISO = P0.10) */
#define SPI_A_MISO_PIN 10

/** @brief SPI_A interrupt priority level */
#define SPI_A_INT_PRIO 1

/**
 * @brief SCLK frequency safe for ADS1298 command decode
 *
 * The ADS1298 needs 4 tCLK periods (tSDECODE) to internally decode each
 * command byte (RESET, SDATAC, RREG, WREG, ...); sending a multi-byte
 * command faster than that -- as a single burst DMA transfer does -- can
 * outrun the decoder. Only applies to actual command bytes, not to reading
 * already-converted samples in RDATAC mode (see SPI_A_ADS_STREAMING_FREQ_HZ).
 */
#define SPI_A_ADS_CMD_SAFE_FREQ_HZ NRFX_MHZ_TO_HZ(4)

/**
 * @brief SCLK frequency for steady-state ADS1298 data streaming
 *
 * SPI rate when reading data from the ADS1298.
 * Safe once the ADS1298 is in RDATAC mode
 * Maxmimum value is 8 Mhz (theoretical 15 MHz, but NRF supports only 8 or 16 MHz)
 */
#define SPI_A_ADS_STREAMING_FREQ_HZ NRFX_MHZ_TO_HZ(4)

/**
 * @brief SCLK frequency for WiFi/SD (ESP32-C6) transfers
 *
 * The ESP side (as slave) can handle 40 MHz
 *
 * If this differs from SPI_A_ADS_STREAMING_FREQ_HZ, spi_master_transceive()
 * (wifi_sd_spi_functions.c) switches SPI_A to this rate for the duration of
 * each ESP transfer and back afterward; if they're equal, it skips both
 * switches. That's a plain runtime check, not #if -- NRFX_MHZ_TO_HZ()'s
 * expansion isn't guaranteed valid in a preprocessor constant expression.
 */
#define SPI_A_ESP_STREAMING_FREQ_HZ NRFX_MHZ_TO_HZ(8)

/** @brief nrfx SPIM driver instance shared by every device on SPI_A */
extern nrfx_spim_t spi_a_inst;

/**
 * @brief Serializes access to SPI_A across all consumers (ADS, WiFi/SD, ...)
 *
 * @note The ADS1298 driver releases this mutex right after kicking off an
 * async nrfx_spim_xfer(), rather than holding it until the transfer completes. 
 * This leaves a window where another consumer (ESP) could start a
 * transfer before the ADS one physically finishes. 
 */
extern struct k_mutex spi_a_mutex;

/**
 * @brief Identifies which consumer owns the in-flight SPI_A transfer
 *
 * Set via spi_a_begin_transfer() immediately before issuing an
 * nrfx_spim_xfer() on spi_a_inst, so the shared interrupt handler routes
 * the completion event to the right consumer.
 */
typedef enum {
  SPI_A_OWNER_NONE,
  SPI_A_OWNER_ADS,
  SPI_A_OWNER_WIFI_SD,
} spi_a_owner_t;

/**
 * @brief Bring up the shared SPI_A (SPIM4) peripheral
 *
 * Configures SPI Mode 1 (CPOL=0, CPHA=1), 4 MHz, MSB first, with no
 * hardware-managed CS (every device's CS is a plain GPIO toggled by its
 * own driver). Call exactly once at boot, regardless of which shields
 * (ExG, WiFi/SD, ...) are actually enabled -- each shield's own init
 * function should call this rather than initializing SPIM4 itself.
 *
 * @return 0 on success, -1 on nrfx init failure
 */
int init_spi_a_bus(void);

/**
 * @brief Declare the owner and CS line for the next SPI_A transfer
 *
 * Must be called with spi_a_mutex held, immediately before asserting CS
 * and issuing nrfx_spim_xfer() on spi_a_inst. Asserts *cs (if non-NULL)
 * and records it so the shared interrupt handler can deassert it
 * automatically on completion; pass NULL if the caller manages its own
 * CS deassertion (e.g. ADS toggles two CS lines from a single owner).
 *
 * @param owner Consumer identifier, used to route the completion callback
 * @param cs    CS GPIO to assert now and auto-deassert on completion, or NULL
 */
void spi_a_begin_transfer(spi_a_owner_t owner, const struct gpio_dt_spec *cs);

/**
 * @brief Returns the current owner of SPI_A, or SPI_A_OWNER_NONE if idle
 *
 * Lets a consumer holding spi_a_mutex confirm no other consumer's transfer
 * is still physically in flight before starting its own -- see
 * spi_a_mutex's doc comment for why the mutex alone isn't sufficient.
 */
spi_a_owner_t spi_a_current_owner(void);

/**
 * @brief Reconfigure SPI_A's CLK frequency
 *
 * Briefly tears down and reinitializes the SPIM peripheral with the same
 * pin/mode config but a new frequency. 
 *
 * @param frequency_hz Desired CLK frequency in Hz -- must be one of the
 *  discrete values nrfx_spim supports (e.g. SPI_A_ADS_CMD_SAFE_FREQ_HZ,
 *  SPI_A_ADS_STREAMING_FREQ_HZ, SPI_A_ESP_STREAMING_FREQ_HZ); arbitrary
 *  values are rejected by nrfx_spim_init() with NRFX_ERROR_INVALID_PARAM.
 * @return 0 on success, -1 on nrfx re-init failure
 */
int spi_a_set_frequency(uint32_t frequency_hz);

#endif // SPI_A_H
