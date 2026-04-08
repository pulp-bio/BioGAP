/*
 * ----------------------------------------------------------------------
 *
 * File: ads_defs.h
 *
 * Last edited: 24.11.2025
 *
 * Copyright (C) 2025, ETH Zurich
 *
 * Authors:
 * - Sebastian Frey (sefrey@iis.ee.ethz.ch), ETH Zurich
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
 * @file ads_defs.h
 * @brief ADS1298 Common Definitions and Constants
 *
 * This header contains all shared definitions, constants, and macros used
 * across the ADS1298 driver modules. It includes:
 * - SPI command definitions
 * - Register address definitions
 * - BLE packet format constants
 * - Hardware pin assignments
 * - SPIM instance configuration
 */

#ifndef ADS_DEFS_H
#define ADS_DEFS_H

#include <stdint.h>

/*==============================================================================
 * BLE Packet Format Constants
 *============================================================================*/

/**
 * @brief BLE packet header byte
 * Marks the start of each 234-byte BLE packet
 */
#define BLE_PCK_HEADER 0x55

/**
 * @brief BLE packet trailer byte
 * Marks the end of each 234-byte BLE packet
 */
#define BLE_PCK_TAILER 0xAA

/**
 * @brief Total EXG BLE packet length in bytes
 *
 * Packet structure (211 bytes total):
 * - 1 byte: Header (0x55)
 * - 2 bytes: Packet counter
 * - 4 bytes: Timestamp (microseconds, for cross-packet synchronization)
 * - 200 bytes: 4 samples × 50 bytes per sample
 *   - 24 bytes: ADS1298_A data (8 channels × 3 bytes)
 *   - 24 bytes: ADS1298_B data (8 channels × 3 bytes)
 *   - 1 byte: Counter_extra
 *   - 1 byte: Reserved (reserved for future use)
 * - 3 bytes: Metadata (reserved for future use)
 * - 1 byte: Trailer (0xAA)
 */
#define EXG_PCK_LNGTH 211


/** @brief Number of EXG samples per BLE packet */
#define EXG_SAMPLES_PER_PACKET 4

/** @brief Bytes per EXG sample (ADS_A + ADS_B + counter_extra + reserved) */
#define EXG_BYTES_PER_SAMPLE 50

/** @brief Index where sample data ends (before metadata)
 *  Calculation: Header(1) + Counter(2) + Timestamp(4) + 4×50 = 207
 */
#define EXG_SAMPLE_DATA_END 207

/*==============================================================================
 * SPIM Instance Configuration
 *============================================================================*/

/** @brief SPIM instance index to be used for ADS1298 communication */
#define SPIM_INST_IDX 2

/** @brief MOSI pin number for SPI communication */
#define MOSI_PIN 9

/** @brief MISO pin number for SPI communication */
#define MISO_PIN 10

/** @brief SCK (clock) pin number for SPI communication */
#define SCK_PIN 8

/*==============================================================================
 * ADS1298 SPI Command Definitions
 *============================================================================*/

/**
 * @brief Wake-up from standby mode
 * Exits low-power standby state and returns to normal operation
 */
#define _WAKEUP 0x02

/**
 * @brief Enter standby mode
 * Places device in low-power state with clock disabled
 */
#define _STANDBY 0x04

/**
 * @brief Reset the device
 * Restores all registers to their default power-on values
 */
#define _RESET 0x06

/**
 * @brief Start conversions
 * Begins ADC conversions and synchronizes multiple devices via START pin
 */
#define _START 0x08

/**
 * @brief Stop conversions
 * Halts ADC conversions immediately
 */
#define _STOP 0x0A

/**
 * @brief Enable Read Data Continuous mode
 * Places device in continuous data output mode (default at power-up)
 */
#define _RDATAC 0x10

/**
 * @brief Stop Read Data Continuous mode
 * Exits continuous mode and allows register access
 */
#define _SDATAC 0x11

/**
 * @brief Read data by command
 * Single data read command (not used in continuous mode)
 */
#define _RDATA 0x12

/**
 * @brief Read registers command
 * Format: _RREG | address, then number of registers - 1
 */
#define _RREG 0x20

/**
 * @brief Write registers command
 * Format: _WREG | address, then number of registers - 1, then data bytes
 */
#define _WREG 0x40

/*==============================================================================
 * ADS1298 Register Address Definitions
 *============================================================================*/

/**
 * @brief Device ID register (Read-only)
 * Expected value: 0xD2 for ADS1298
 */
#define ID 0x00

/**
 * @brief Configuration Register 1
 * Controls: Data rate, CLK connection, daisy-chain mode
 */
#define CONFIG1 0x01

/**
 * @brief Configuration Register 2
 * Controls: Test signal generation, internal/external reference
 */
#define CONFIG2 0x02

/**
 * @brief Configuration Register 3
 * Controls: Internal reference, RLD buffer, bias buffer settings
 */
#define CONFIG3 0x03

/**
 * @brief Lead-Off Control Register
 * Controls: Lead-off detection mode, frequency, and threshold
 */
#define LOFF 0x04

/**
 * @brief Channel 1 Settings
 * Controls: Power-down, PGA gain, input MUX for channel 1
 */
#define CH1SET 0x05

/**
 * @brief Channel 2 Settings
 * Controls: Power-down, PGA gain, input MUX for channel 2
 */
#define CH2SET 0x06

/**
 * @brief Channel 3 Settings
 * Controls: Power-down, PGA gain, input MUX for channel 3
 */
#define CH3SET 0x07

/**
 * @brief Channel 4 Settings
 * Controls: Power-down, PGA gain, input MUX for channel 4
 */
#define CH4SET 0x08

/**
 * @brief Channel 5 Settings
 * Controls: Power-down, PGA gain, input MUX for channel 5
 */
#define CH5SET 0x09

/**
 * @brief Channel 6 Settings
 * Controls: Power-down, PGA gain, input MUX for channel 6
 */
#define CH6SET 0x0A

/**
 * @brief Channel 7 Settings
 * Controls: Power-down, PGA gain, input MUX for channel 7
 */
#define CH7SET 0x0B

/**
 * @brief Channel 8 Settings
 * Controls: Power-down, PGA gain, input MUX for channel 8
 */
#define CH8SET 0x0C

/**
 * @brief RLD Positive Signal Derivation
 * Selects which channels contribute to positive RLD output
 */
#define RLD_SENSP 0x0D

/**
 * @brief RLD Negative Signal Derivation
 * Selects which channels contribute to negative RLD output
 */
#define RLD_SENSN 0x0E

/**
 * @brief Lead-Off Positive Signal Selection
 * Enables lead-off detection on positive inputs
 */
#define LOFF_SENSP 0x0F

/**
 * @brief Lead-Off Negative Signal Selection
 * Enables lead-off detection on negative inputs
 */
#define LOFF_SENSN 0x10

/**
 * @brief Lead-Off Current Direction Control
 * Flips the direction of lead-off current for specific channels
 */
#define LOFF_FLIP 0x11

/**
 * @brief Lead-Off Positive Status (Read-only)
 * Indicates which positive inputs have lead-off detected
 */
#define LOFF_STATP 0x12

/**
 * @brief Lead-Off Negative Status (Read-only)
 * Indicates which negative inputs have lead-off detected
 */
#define LOFF_STATN 0x13

/**
 * @brief GPIO Control Register
 * Controls general-purpose I/O pins
 */
#define GPIO 0x14

/**
 * @brief Pace Detect Register
 * Controls pacemaker detection circuit
 */
#define PACE 0x15

/**
 * @brief Respiration Control Register
 * Controls respiration impedance measurement
 */
#define RESP 0x16

/**
 * @brief Configuration Register 4
 * Controls: Lead-off comparator, GPIO, and other settings
 */
#define CONFIG4 0x17

/**
 * @brief Wilson Central Terminal 1
 * Controls Wilson Central Terminal and augmented lead configuration
 */
#define WCT1 0x18

/**
 * @brief Wilson Central Terminal 2
 * Controls Wilson Central Terminal power-down and routing
 */
#define WCT2 0x19

#endif // ADS_DEFS_H
