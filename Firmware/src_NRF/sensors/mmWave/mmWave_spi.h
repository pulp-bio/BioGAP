/*
 * ----------------------------------------------------------------------
 *
 * File: mmWave_spi.h
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
 * @file mmWave_spi.h
 * @brief BGT60TR13C SPI transport over the shared SPI_A bus
 *
 * The Infineon xensiv_bgt60trxx driver reaches the hardware through a small
 * set of platform callbacks. This module implements the SPI-related ones on
 * top of BioGAP's raw nrfx SPI_A layer (spi/spi_a.h), so the radar shares the
 * bus with the ADS1298 AFEs and the WiFi/SD shield instead of requiring
 * Zephyr's SPI subsystem (which cannot coexist with the nrfx driver on the
 * same SPIM instance).
 *
 * Bus arbitration follows the driver's strict CS pairing: the chip-select
 * callback takes spi_a_mutex and switches the bus to the radar's SPI mode and
 * clock when CS is asserted, and restores the ADS1298 defaults and releases
 * the mutex when CS is deasserted. Every register access and FIFO burst of the
 * driver is wrapped in exactly one such CS pair, so the radar holds the bus
 * for the duration of a transaction and never leaves it misconfigured.
 */

#ifndef MMWAVE_SPI_H
#define MMWAVE_SPI_H

#include <stdint.h>

/**
 * @brief Opaque interface handle to hand to xensiv_bgt60trxx_init()
 *
 * The Infineon driver stores this and passes it back to every platform
 * callback. This transport is a singleton on SPI_A and resolves the bus from
 * the devicetree, so the handle carries no state -- but the driver asserts
 * that it is not NULL, so it must be a valid pointer.
 */
extern void *const mmwave_spi_iface;

/**
 * @brief Bring up the SPI transport and the chip-select GPIO
 *
 * Initializes the shared SPI_A bus (idempotent, safe regardless of which
 * other shields already did so) and drives CS to its idle level.
 *
 * @return 0 on success, negative errno on failure
 */
int mmwave_spi_init(void);

/**
 * @brief Override the SPI clock used for radar transfers
 *
 * Defaults to CONFIG_MMWAVE_SPI_FREQ_MHZ. Exposed so the bring-up diagnostics
 * can re-probe the device at a lower clock: if a register read that fails at
 * 8 MHz succeeds at 4 MHz, the problem is bus timing (MISO sampling, wiring
 * capacitance) rather than power or addressing.
 *
 * @param freq_mhz 1, 2, 4 or 8. Other values are ignored.
 */
void mmwave_spi_set_clock_mhz(uint8_t freq_mhz);

/**
 * @brief Log the SPIM peripheral's current pin and timing configuration
 *
 * Reports ENABLE, FREQUENCY, CONFIG (mode/bit order) and the SCK/MOSI/MISO pin
 * selections, so a failed bring-up can be told apart from a bus that was left
 * configured for another device on the shared SPI_A.
 */
void mmwave_spi_log_bus_state(void);

/**
 * @brief Drive chip select to its parked level for an unpowered radar
 *
 * Unlike the driver's deassert callback -- which drives CS physically high, as
 * the radar's protocol requires while it is powered -- this parks the line
 * physically low, so no voltage reaches the part once its supply rail is cut.
 * Same level mmwave_spi_init() leaves the pin at before the first power-on.
 */
void mmwave_spi_park_cs(void);

/**
 * @brief SPI_A completion callback, invoked from the shared bus ISR
 *
 * Routed here by spi/spi_a.c when the in-flight transfer belongs to
 * SPI_A_OWNER_MMWAVE. Not intended to be called by application code.
 */
void mmwave_spim_transfer_complete(void);

/**
 * @brief Access the buffer holding the last FIFO burst, still 12-bit packed
 *
 * The radar streams ADC samples as 12-bit words, two per three bytes.
 * xensiv_bgt60trxx_platform_spi_fifo_read() keeps that raw representation
 * here while also handing unpacked uint16_t samples to the driver, so the
 * application can forward the packed form over BLE without re-packing it.
 *
 * @return Pointer to a buffer of MMWAVE_FRAME_SIZE_BYTES_PACKED bytes
 */
const uint8_t *mmwave_spi_get_packed_frame(void);

#endif /* MMWAVE_SPI_H */
