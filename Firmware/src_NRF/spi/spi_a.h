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

/*
 * Idle configuration of the bus, i.e. what init_spi_a_bus() sets and what
 * spi_a_restore_default_config() returns to.
 *
 * CONFIG_MMWAVE_SPI_STATIC_MODE is a bring-up diagnostic: it brings the bus up
 * in the radar's dialect (mode 0) and makes the radar skip its per-transaction
 * reconfigure, so the mode switching itself can be ruled in or out as the cause
 * of a non-responsive radar. It breaks the ADS1298, which needs mode 1.
 */
#if defined(CONFIG_MMWAVE_SPI_STATIC_MODE)
#define SPI_A_DEFAULT_MODE NRF_SPIM_MODE_0
#define SPI_A_DEFAULT_FREQ NRF_SPIM_FREQ_8M
#define SPI_A_DEFAULT_FREQ_HZ NRFX_MHZ_TO_HZ(CONFIG_MMWAVE_SPI_FREQ_MHZ)
#else
/** @brief SPI mode the bus is left in when no owner has reconfigured it
 *  (Mode 1 = CPOL 0 / CPHA 1, required by the ADS1298) */
#define SPI_A_DEFAULT_MODE NRF_SPIM_MODE_1

/** @brief SPI clock the bus is left in when no owner has reconfigured it */
#define SPI_A_DEFAULT_FREQ NRF_SPIM_FREQ_4M
#define SPI_A_DEFAULT_FREQ_HZ NRFX_MHZ_TO_HZ(4)
#endif

/** @brief nrfx SPIM driver instance shared by every device on SPI_A */
extern nrfx_spim_t spi_a_inst;

/**
 * @brief Serializes access to SPI_A across all consumers (ADS, WiFi/SD, ...)
 *
 * @note The ADS1298 driver releases this mutex right after kicking off an
 * async nrfx_spim_xfer() (matching its pre-existing, hardware-validated
 * timing), rather than holding it until the transfer completes. This
 * leaves a narrow theoretical window where another consumer could start a
 * transfer before the ADS one physically finishes. In practice ADS
 * transfers are a few dozen bytes at 4 MHz (tens of microseconds), so the
 * window is small, but it is not zero -- treat concurrent ADS + WiFi/SD
 * traffic as a known follow-up to validate on hardware, or to close by
 * having ADS hold the mutex until completion too (would require moving
 * its irq_lock()/busy-wait handling around, since Zephyr mutexes cannot be
 * released from ISR context).
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
  SPI_A_OWNER_MMWAVE,
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
 * @brief Switch the bus mode and clock for the current owner
 *
 * Devices on SPI_A do not all speak the same SPI dialect: the ADS1298 needs
 * Mode 1 at 4 MHz, the BGT60TR13C radar needs Mode 0 and runs faster. Only
 * the CONFIG and FREQUENCY registers are rewritten, so pin assignment, DMA
 * and interrupt setup from init_spi_a_bus() stay intact.
 *
 * Must be called with spi_a_mutex held and no transfer in flight. An owner
 * that changes the configuration is responsible for calling
 * spi_a_restore_default_config() before releasing the mutex, so that owners
 * which never touch the configuration (the ADS1298 driver) keep seeing the
 * bus exactly as init_spi_a_bus() left it.
 *
 * @param mode      SPI mode to switch to
 * @param frequency SPI clock to switch to
 */
void spi_a_reconfigure(nrf_spim_mode_t mode, nrf_spim_frequency_t frequency);

/**
 * @brief Report whether a transfer is still in flight on the bus
 *
 * True between spi_a_begin_transfer() and the completion event. Because the
 * ADS1298 driver releases spi_a_mutex as soon as it has kicked off its async
 * transfer (see the note on spi_a_mutex), holding the mutex is not by itself
 * proof that the bus is idle. A consumer that rewrites the peripheral's
 * configuration -- rather than only pushing data -- must wait for this to go
 * false first, or it would corrupt the transfer still running.
 *
 * @return true if a transfer is in progress, false if the bus is idle
 */
bool spi_a_transfer_in_flight(void);

/**
 * @brief Restore the bus to SPI_A_DEFAULT_MODE / SPI_A_DEFAULT_FREQ
 *
 * Counterpart to spi_a_reconfigure(); see its documentation. Must be called
 * with spi_a_mutex held and no transfer in flight.
 */
void spi_a_restore_default_config(void);

#endif // SPI_A_H
