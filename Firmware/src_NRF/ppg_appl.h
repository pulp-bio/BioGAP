/*
 * ----------------------------------------------------------------------
 *
 * File: ppg_appl.h
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

#ifndef MAX86150_H_
#define MAX86150_H_

#include <zephyr/drivers/i2c.h>
#include <zephyr/kernel.h>
#include <stdint.h>
#include <stdbool.h>
#include <string.h>

// Define the I2C node used for the sensor.
// Ensure that your board’s DTS defines a node with label "max86150ppg"
// (or change the label below to match your DTS).
#define I2C0_NODE DT_NODELABEL(max86150ppg)

// MAX86150 7-bit I2C address
#define MAX86150_ADDRESS          0x5E

// For example purposes, we assume two active devices (e.g. red and IR)
#define activeDevices             2

// Each uint32_t occupies 4 bytes; adjust STORAGE_SIZE to your RAM limits.
#define STORAGE_SIZE 40

typedef struct Record {
    uint32_t red[STORAGE_SIZE];
    uint32_t IR[STORAGE_SIZE];
    // int32_t ecg[STORAGE_SIZE]; // Uncomment if ECG is used
    uint16_t head;
    uint16_t tail;
} sense_struct;

enum MAX20303_PPG_function_t { PPG_ACTIVE, PPG_STILL };

#ifdef __cplusplus
extern "C" {
#endif

bool MAX86150_begin(void);
void check_max86150(void);
uint32_t MAX86150_Red_get(void);
uint32_t MAX86150_IR_get(void);
enum MAX20303_PPG_function_t Get_PPG_Function(void);
void Set_PPG_Function(enum MAX20303_PPG_function_t f);

uint8_t MAX86150_getINT1(void);
uint8_t MAX86150_getINT2(void);
void MAX86150_enableAFULL(void);
void MAX86150_disableAFULL(void);
void MAX86150_enableDATARDY(void);
void MAX86150_disableDATARDY(void);
void MAX86150_enableALCOVF(void);
void MAX86150_disableALCOVF(void);
void MAX86150_enablePROXINT(void);
void MAX86150_disablePROXINT(void);
void MAX86150_softReset(void);
void MAX86150_shutDown(void);
void MAX86150_wakeUp(void);
void MAX86150_setLEDMode(uint8_t mode);
void MAX86150_setADCRange(uint8_t adcRange);
void MAX86150_setSampleRate(uint8_t sampleRate);
void MAX86150_setPulseWidth(uint8_t pulseWidth);
void MAX86150_setPulseAmplitudeRed(uint8_t amplitude);
void MAX86150_setPulseAmplitudeIR(uint8_t amplitude);
void MAX86150_setPulseAmplitudeProximity(uint8_t amplitude);
void MAX86150_setProximityThreshold(uint8_t threshMSB);
void MAX86150_enableSlot(uint8_t slotNumber, uint8_t device);
void MAX86150_disableSlots(void);
void MAX86150_setFIFOAverage(uint8_t numberOfSamples);
void MAX86150_clearFIFO(void);
void MAX86150_enableFIFORollover(void);
void MAX86150_disableFIFORollover(void);
void MAX86150_setFIFOAlmostFull(uint8_t numberOfSamples);
uint8_t MAX86150_getWritePointer(void);
uint8_t MAX86150_getReadPointer(void);
uint8_t MAX86150_checkOVF(void);
void MAX86150_setPROXINTTHRESH(uint8_t val);
uint8_t MAX86150_readPartID(void);
void MAX86150_setup(uint8_t sampleAverage);
uint8_t MAX86150_available(void);
uint32_t MAX86150_getRed(void);
uint32_t MAX86150_getIR(void);
int32_t MAX86150_getECG(void);
uint32_t MAX86150_getFIFORed(void);
uint32_t MAX86150_getFIFOIR(void);
int32_t MAX86150_getFIFOECG(void);
void MAX86150_nextSample(void);
uint16_t MAX86150_check(void);
bool MAX86150_safeCheck(uint8_t maxTimeToCheck);
void MAX86150_bitMask(uint8_t reg, uint8_t mask, uint8_t thing);
uint8_t MAX86150_readRegister8(uint8_t reg);
void MAX86150_writeRegister8(uint8_t reg, uint8_t value);

#ifdef __cplusplus
}
#endif

#endif // MAX86150_H_
