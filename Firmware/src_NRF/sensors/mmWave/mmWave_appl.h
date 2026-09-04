/*
 * ----------------------------------------------------------------------
 *
 * File: mmWave_appl.h
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
 * @file mmWave_appl.h
 * @brief Public interface for the mmWave radar application layer
 *
 * Controls the Infineon XENSIV BGT60TR13C 60 GHz FMCW radar on the SENSEI
 * mmWave shield: hardware bring-up, power control, register configuration,
 * runtime parameter updates and frame streaming over BLE.
 *
 * ## Lifecycle
 *
 * The subsystem is a small state machine (see @ref mmWave_state_t) and the
 * calls below must follow that order:
 *
 * ```
 * mmWave_HW_init()        // once at boot          -> HW_ACTIVE
 * mmWave_power_on()       // enable the shield     -> IDLE
 * mmWave_configure()      // write register list   -> CONFIGURED
 * mmWave_start_streaming()                         -> STREAMING
 * mmWave_stop_streaming()                          -> CONFIGURED
 * mmWave_power_off()                               -> HW_ACTIVE
 * ```
 *
 * A power cycle invalidates the register configuration, so
 * mmWave_configure() must be called again after every mmWave_power_on().
 *
 * ## Hardware sharing caveats
 *
 * - The radar sits on the shared SPI_A bus. Bus arbitration is handled in
 *   sensors/mmWave/mmWave_spi.c via spi_a_mutex, so ExG acquisition may run
 *   concurrently, at the cost of some added SPI latency for both.
 * - The shield's power-enable pin doubles as the battery-monitor ADC input
 *   (P0.07 / SAADC AIN3). While the radar is powered, that SAADC channel is
 *   released to the GPIO, so battery telemetry is not meaningful.
 * - The IRQ and RST pins (P0.12 / P0.04) are the PDM microphone's DATA and
 *   CLK lines. The radar and the onboard microphone are mutually exclusive;
 *   the mmWave build overlay disables the PDM node.
 */

#ifndef MMWAVE_APPL_H
#define MMWAVE_APPL_H

#include <stdbool.h>
#include <stdint.h>

/*==============================================================================
 * Packet Format
 *============================================================================*/

/**
 * @brief First byte of every mmWave BLE packet
 *
 * Distinct from every other streaming header (0x55 ExG, 0x56 IMU, 0xAA MIC,
 * 0x70 PPG, 0x10-0x13 WULPUS, 0x2B system status) so the host can demultiplex
 * packet types on a shared link.
 */
#define MMWAVE_DATA_HEADER 0x60U

/** @brief Last byte of every mmWave BLE packet */
#define MMWAVE_DATA_TRAILER 0x61U

/*==============================================================================
 * Type Definitions
 *============================================================================*/

/**
 * @enum mmWave_state_t
 * @brief mmWave device states
 */
typedef enum {
    mmWave_STATE_NO_HW,      /**< mmWave off, HW not ready, initial state */
    mmWave_STATE_HW_ACTIVE,  /**< mmWave off, HW ready */
    mmWave_STATE_IDLE,       /**< mmWave on, not configured */
    mmWave_STATE_CONFIGURED, /**< mmWave configured, not streaming */
    mmWave_STATE_STREAMING,  /**< mmWave actively streaming data */
    mmWave_STATE_STOPPING,   /**< mmWave stopping */
    mmWave_STATE_ERROR       /**< Error state */
} mmWave_state_t;

/*==============================================================================
 * Function Declarations
 *============================================================================*/

/**
 * @brief Initialize the mmWave hardware.
 *
 * Configures the radar's control GPIOs and brings up the shared SPI_A
 * transport. Must be called before mmWave_power_on().
 *
 * @return 0 on success.
 * @return -EALREADY if the hardware has already been initialized.
 * @return Negative error code on other failures.
 */
int mmWave_HW_init(void);

/**
 * @brief Power on the mmWave system.
 *
 * Enables power to the BGT60TR13C and probes the device. After this call the
 * sensor is powered and ready to be configured.
 *
 * While the mmWave system is powered on, the power control pin is reserved for
 * the mmWave subsystem and cannot be used by the ADC to read the battery
 * status.
 *
 * @return 0 on success.
 * @return -EPERM if the hardware is not initialized or already powered on.
 * @return -EIO if the device does not answer.
 */
int mmWave_power_on(void);

/**
 * @brief Configure the mmWave sensor.
 *
 * Writes the compiled-in register list plus the current IF gain, TX power and
 * frame rate, then arms the FIFO-limit interrupt.
 *
 * Must be called after mmWave_power_on() and before
 * mmWave_start_streaming(). A power cycle invalidates the configuration.
 *
 * @return 0 on success.
 * @return -EPERM if the device is not powered on.
 * @return -EIO on communication failure.
 */
int mmWave_configure(void);

/**
 * @brief Report whether the radar currently owns the shared power pin.
 *
 * The shield's power-enable pin is also the battery-monitor ADC input
 * (P0.07 / SAADC AIN3). While this returns true the pin is driven as a GPIO,
 * so a battery measurement would sample the radar's enable level instead of the
 * battery: callers that own the ADC must skip their measurement.
 *
 * @return true between mmWave_power_on() and mmWave_power_off().
 */
bool mmWave_is_powered(void);

/**
 * @brief Power off the mmWave system.
 *
 * Disables power to the BGT60TR13C and releases the power control pin back to
 * the battery-monitor ADC.
 *
 * @return 0 on success.
 * @return -EPERM if the device is currently streaming.
 */
int mmWave_power_off(void);

/**
 * @brief Change the IF gain of the radar's ADCs.
 *
 * If the device is configured, the new value is applied immediately; otherwise
 * it is stored and applied at the next mmWave_configure().
 *
 * @param ifGain IF gain in dB. One of
 *               18, 23, 28, 30, 33, 35, 38, 40, 43, 45, 48, 50, 55, 60.
 *
 * @return 0 on success.
 * @return -EINVAL if the IF gain value is not supported.
 * @return -EIO if the update could not be written.
 */
int mmWave_set_ifGain(uint8_t ifGain);

/**
 * @brief Change the radar's TX power level.
 *
 * If the device is configured, the new value is applied immediately; otherwise
 * it is stored and applied at the next mmWave_configure().
 *
 * @param txPower TX power level, 0-31.
 *
 * @return 0 on success.
 * @return -EINVAL if the txPower value is out of range.
 * @return -EIO if the update could not be written.
 */
int mmWave_set_txPower(uint8_t txPower);

/**
 * @brief Change the radar frame rate.
 *
 * If the device is configured, the new value is applied immediately; otherwise
 * it is stored and applied at the next mmWave_configure().
 *
 * @param fps Frame rate in frames per second. One of 25, 50, 100, 150, 200.
 *
 * @return 0 on success.
 * @return -EINVAL if the frame rate is not supported.
 * @return -EIO if the update could not be written.
 */
int mmWave_set_fps(uint8_t fps);

/**
 * @brief Start mmWave streaming.
 *
 * @return 0 on success.
 * @return -EALREADY if streaming is already active.
 * @return -EBUSY if the device is not in the configured state.
 */
int mmWave_start_streaming(void);

/**
 * @brief Stop mmWave streaming.
 *
 * @return 0 on success.
 * @return -EINVAL if streaming is not currently active.
 * @return -ETIMEDOUT if the streaming thread did not stop in time.
 */
int mmWave_stop_streaming(void);

#endif /* MMWAVE_APPL_H */
