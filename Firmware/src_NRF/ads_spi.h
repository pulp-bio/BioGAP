/*
 * ----------------------------------------------------------------------
 *
 * File: ads_spi.h
 *
 * Last edited: 23.07.2025
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
#ifndef ADS_SPI_H
#define ADS_SPI_H

#include <stdint.h>
#include <stdbool.h>
#include "ads_appl.h"

#define BLE_PCK_TAILER                  0xAA
#define BLE_PCK_HEADER                  0x55
#define PCK_LNGTH                       234

/** @brief Symbol specifying SPIM instance to be used. */
#define SPIM_INST_IDX 2
/** @brief Symbol specifying pin number for MOSI. */
#define MOSI_PIN 9
/** @brief Symbol specifying pin number for MISO. */
#define MISO_PIN 10
/** @brief Symbol specifying pin number for SCK. */
#define SCK_PIN 8
///** @brief Symbol specifying pin number for CS. */
//#define ADS_A_CS_PIN 11


//ADS1298 SPI Command Definition Byte Assignments
#define _WAKEUP 	0x02	// Wake-up from standby mode
#define _STANDBY 	0x04	// Enter Standby mode
#define _RESET 		0x06	// Reset the device registers to default
#define _START 		0x08	// Start and restart (synchronize) conversions
#define _STOP 		0x0A	// Stop conversion
#define _RDATAC 	0x10	// Enable Read Data Continuous mode (default mode at power-up)
#define _SDATAC 	0x11	// Stop Read Data Continuous mode
#define _RDATA 		0x12	// Read data by command; supports multiple read back
#define _RREG		0x20	// Read n nnnn registers starting at address rrrr
#define _WREG		0x40	// Write n nnnn registers starting at address rrrr

//ASD1298 Register Addresses
#define ID      	0x00
#define CONFIG1 	0x01
#define CONFIG2 	0x02
#define CONFIG3 	0x03
#define LOFF 		0x04
#define CH1SET 		0x05
#define CH2SET 		0x06
#define CH3SET 		0x07
#define CH4SET 		0x08
#define CH5SET 		0x09
#define CH6SET 		0x0A
#define CH7SET 		0x0B
#define CH8SET 		0x0C
#define RLD_SENSP 	0x0D
#define RLD_SENSN 	0x0E
#define LOFF_SENSP	0x0F
#define LOFF_SENSN	0x10
#define LOFF_FLIP	0x11
#define LOFF_STATP	0x12
#define LOFF_STATN	0x13
#define GPIO 		0x14
#define PACE		0x15
#define RESP		0x16
#define CONFIG4		0x17
#define WCT1		0x18
#define WCT2		0x19

extern uint8_t pr_word[10];
extern bool ads_initialized;

void init_SPI();
void ADS_Start();
void ADS_Stop();
void ADS_Init(uint8_t *InitParams, enum ADS_id_t ads_id);
void ADS_check_ID(enum ADS_id_t ads_id);

int ADS_dr_init();
int ADS_dr_read();

void process_ads_data();


#endif // ADS_SPI_H