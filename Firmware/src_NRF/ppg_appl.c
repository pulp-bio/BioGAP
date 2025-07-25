/*
 * ----------------------------------------------------------------------
 *
 * File: ppg_appl.c
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

#include "ppg_appl.h"
#include <zephyr/device.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/kernel.h>
#include <string.h>
#include <zephyr/logging/log.h>
#include <zephyr/logging/log_ctrl.h>

// If you use SPI for other sensors, include its header here.
// #include "ADSXXXX_SPI.h"
LOG_MODULE_REGISTER(ppg_appl, LOG_LEVEL_INF);


/* Define stack sizes and priorities */
#define PPG_STACK_SIZE    2048
#define PPG_PRIORITY      6

K_SEM_DEFINE(ppg_data_ready, 0, 1);

// Define the I2C device spec using the DTS node.
// Make sure the DTS node "max86150ppg" is defined with proper properties.
static const struct i2c_dt_spec dev_i2c = I2C_DT_SPEC_GET(I2C0_NODE);

// Define register addresses
static const uint8_t MAX86150_INTSTAT1      = 0x00;
static const uint8_t MAX86150_INTSTAT2      = 0x01;
static const uint8_t MAX86150_INTENABLE1    = 0x02;
static const uint8_t MAX86150_INTENABLE2    = 0x03;

static const uint8_t MAX86150_FIFOWRITEPTR  = 0x04;
static const uint8_t MAX86150_FIFOOVERFLOW  = 0x05;
static const uint8_t MAX86150_FIFOREADPTR   = 0x06;
static const uint8_t MAX86150_FIFODATA      = 0x07;

static const uint8_t MAX86150_FIFOCONFIG    = 0x08;
static const uint8_t MAX86150_FIFOCONTROL1  = 0x09;
static const uint8_t MAX86150_FIFOCONTROL2  = 0x0A;

static const uint8_t MAX86150_SYSCONTROL    = 0x0D;
static const uint8_t MAX86150_PPGCONFIG1    = 0x0E;
static const uint8_t MAX86150_PPGCONFIG2    = 0x0F;
static const uint8_t MAX86150_LED_PROX_AMP  = 0x10;

static const uint8_t MAX86150_LED1_PULSEAMP = 0x11;
static const uint8_t MAX86150_LED2_PULSEAMP = 0x12;
static const uint8_t MAX86150_LED_RANGE     = 0x14;
static const uint8_t MAX86150_LED_PILOT_PA  = 0x15;

static const uint8_t MAX86150_ECG_CONFIG1   = 0x3C;
static const uint8_t MAX86150_ECG_CONFIG3   = 0x3E;
static const uint8_t MAX86150_PROXINTTHRESH = 0x10;

static const uint8_t MAX86150_PARTID        = 0xFF;

// MAX86150 Commands (bit masks and values)
static const uint8_t MAX86150_INT_A_FULL_MASK    = (uint8_t)~0b10000000;
static const uint8_t MAX86150_INT_A_FULL_ENABLE  = 0x80;
static const uint8_t MAX86150_INT_A_FULL_DISABLE = 0x00;

static const uint8_t MAX86150_INT_DATA_RDY_MASK  = (uint8_t)~0b01000000;
static const uint8_t MAX86150_INT_DATA_RDY_ENABLE= 0x40;
static const uint8_t MAX86150_INT_DATA_RDY_DISABLE=0x00;

static const uint8_t MAX86150_INT_ALC_OVF_MASK   = (uint8_t)~0b00100000;
static const uint8_t MAX86150_INT_ALC_OVF_ENABLE = 0x20;
static const uint8_t MAX86150_INT_ALC_OVF_DISABLE= 0x00;

static const uint8_t MAX86150_INT_PROX_INT_MASK  = (uint8_t)~0b00010000;
static const uint8_t MAX86150_INT_PROX_INT_ENABLE= 0x10;
static const uint8_t MAX86150_INT_PROX_INT_DISABLE=0x00;

static const uint8_t MAX86150_SAMPLEAVG_MASK     = (uint8_t)~0b11100000;
static const uint8_t MAX86150_SAMPLEAVG_1        = 0x00;
static const uint8_t MAX86150_SAMPLEAVG_2        = 0x20;
static const uint8_t MAX86150_SAMPLEAVG_4        = 0x40;
static const uint8_t MAX86150_SAMPLEAVG_8        = 0x60;
static const uint8_t MAX86150_SAMPLEAVG_16       = 0x80;
static const uint8_t MAX86150_SAMPLEAVG_32       = 0xA0;

static const uint8_t MAX86150_ROLLOVER_MASK      = 0xEF;
static const uint8_t MAX86150_ROLLOVER_ENABLE    = 0x10;
static const uint8_t MAX86150_ROLLOVER_DISABLE   = 0x00;

static const uint8_t MAX86150_A_FULL_MASK        = 0xF0;

static const uint8_t MAX86150_SHUTDOWN_MASK      = 0x7F;
static const uint8_t MAX86150_SHUTDOWN           = 0x80;
static const uint8_t MAX86150_WAKEUP             = 0x00;

static const uint8_t MAX86150_RESET_MASK         = 0xFE;
static const uint8_t MAX86150_RESET              = 0x01;

static const uint8_t MAX86150_MODE_MASK          = 0xF8;
static const uint8_t MAX86150_MODE_REDONLY       = 0x02;
static const uint8_t MAX86150_MODE_REDIRONLY     = 0x03;
static const uint8_t MAX86150_MODE_MULTILED      = 0x07;

static const uint8_t MAX86150_ADCRANGE_MASK      = 0x9F;
static const uint8_t MAX86150_ADCRANGE_2048      = 0x00;
static const uint8_t MAX86150_ADCRANGE_4096      = 0x20;
static const uint8_t MAX86150_ADCRANGE_8192      = 0x40;
static const uint8_t MAX86150_ADCRANGE_16384     = 0x60;

static const uint8_t MAX86150_SAMPLERATE_MASK    = 0xE3;
static const uint8_t MAX86150_SAMPLERATE_50      = 0x00;
static const uint8_t MAX86150_SAMPLERATE_100     = 0x04;
static const uint8_t MAX86150_SAMPLERATE_200     = 0x08;
static const uint8_t MAX86150_SAMPLERATE_400     = 0x0C;
static const uint8_t MAX86150_SAMPLERATE_800     = 0x10;
static const uint8_t MAX86150_SAMPLERATE_1000    = 0x14;
static const uint8_t MAX86150_SAMPLERATE_1600    = 0x18;
static const uint8_t MAX86150_SAMPLERATE_3200    = 0x1C;

static const uint8_t MAX86150_PULSEWIDTH_MASK    = 0xFC;
static const uint8_t MAX86150_PULSEWIDTH_69      = 0x00;
static const uint8_t MAX86150_PULSEWIDTH_118     = 0x01;
static const uint8_t MAX86150_PULSEWIDTH_215     = 0x02;
static const uint8_t MAX86150_PULSEWIDTH_411     = 0x03;

static const uint8_t MAX86150_SLOT1_MASK         = 0xF0;
static const uint8_t MAX86150_SLOT2_MASK         = 0x0F;
static const uint8_t MAX86150_SLOT3_MASK         = 0xF0;
static const uint8_t MAX86150_SLOT4_MASK         = 0x0F;

static const uint8_t SLOT_NONE                  = 0x00;
static const uint8_t SLOT_RED_LED               = 0x01;
static const uint8_t SLOT_IR_LED                = 0x02;
static const uint8_t SLOT_RED_PILOT             = 0x09;
static const uint8_t SLOT_IR_PILOT              = 0x0A;
static const uint8_t SLOT_ECG                   = 0x0D;

static const uint8_t MAX_30105_EXPECTEDPARTID   = 0x1E;

// Global variables (adjust scope as needed)
sense_struct sense;
volatile int data_available;
static int init_flag = 0;

static uint8_t FIFO_data[288];

// PPG sensor function state
static enum MAX20303_PPG_function_t MAX20303_PPG_function = PPG_STILL;


#define MAX86150_INT_NODE DT_NODELABEL(gpio_ppg_max86150_dr)
static const struct gpio_dt_spec max86150_int_gpio = GPIO_DT_SPEC_GET(MAX86150_INT_NODE, gpios);

static struct gpio_callback max86150_cb_data;

static uint32_t red = 0;
static uint32_t ir = 0;

// Funtion to get the PPG red data
uint32_t MAX86150_Red_get(void)
{
    return red;
}
// Funtion to get the PPG IR data
uint32_t MAX86150_IR_get(void)
{
    return ir;
}

/**
 * @brief BLE Send Thread
 *
 * This thread continuously retrieves data from the send message queue
 * and transmits it over BLE.
 *
 * @param arg1 Unused
 * @param arg2 Unused
 * @param arg3 Unused
 */
void ppg_receive_thread(void *arg1, void *arg2, void *arg3)
{

    int ret;

    LOG_INF("PPG thread started");
    // sleep for 3 seconds
    k_msleep(3000);

    // Initialize the sensor
    if (!MAX86150_begin()) {
        LOG_ERR("MAX86150 not found!");
        return;
    }

    LOG_INF("MAX86150 detected. Initializing...");

    // Configure the sensor (default settings)
    MAX86150_setup(16);  // Sample averaging set to 4

    // Enable Data Ready Interrupt
    MAX86150_enableDATARDY();

    while (1) {
        // Take the PPG semaphore
        //ret = k_sem_take(&ppg_data_ready, K_FOREVER);
        // read all samples
        //check_max86150();
        //red = MAX86150_getRed();
        //ir = MAX86150_getIR();

        k_sem_take(&ppg_data_ready, K_FOREVER);
        uint16_t samples = MAX86150_check();
        if (samples < 0) {
            //red = MAX86150_getRed();
                //ir = MAX86150_getIR();
                red = MAX86150_getFIFORed();
                ir = MAX86150_getFIFOIR();
            //LOG_INF("RED: %d | IR: %d", red, ir);
            if (red > 1000000 && ir > 0) {
                for (int i = 0; i < 10; i++) {
                LOG_INF("RED: %d | IR: %d", red, ir);
                }
            }
        }
        //
        //k_sleep(K_MSEC(5));  // Adjust sampling interval
        
    }
}

void max86150_irq_callback(const struct device *dev,
                           struct gpio_callback *cb,
                           uint32_t pins)
{
    // Give the semaphore to the PPG thread
    k_sem_give(&ppg_data_ready);
    //LOG_INF("Max86150 interrupt triggered on pin(s): 0x%08x", pins);
}

// Forward declaration for safeCheck (implementation can be modified as needed)
bool MAX86150_safeCheck(uint8_t maxTimeToCheck)
{
    /* Example implementation: Poll for new data for up to maxTimeToCheck ms.
       This is a blocking call; adjust as necessary. */
    uint8_t timeout = maxTimeToCheck;
    while (MAX86150_available() == 0 && timeout--) {
        k_msleep(1);
    }
    return (MAX86150_available() > 0);
}

bool MAX86150_begin(void)
{
    uint8_t partID = MAX86150_readPartID();
    if (partID != MAX_30105_EXPECTEDPARTID) {
        return false;
    }

    if (!device_is_ready(max86150_int_gpio.port)) {
        LOG_ERR("GPIO device %s is not ready", max86150_int_gpio.port->name);
        return false;
    }

    int ret = gpio_pin_configure_dt(&max86150_int_gpio, GPIO_INPUT);
    if (ret < 0) {
        LOG_ERR("Failed to configure GPIO pin %d (error %d)", max86150_int_gpio.pin, ret);
        return false;
    }

    ret = gpio_pin_interrupt_configure_dt(&max86150_int_gpio, GPIO_INT_EDGE_TO_ACTIVE);
    if (ret < 0) {
        LOG_ERR("Failed to configure interrupt on GPIO pin %d (error %d)", max86150_int_gpio.pin, ret);
        return false;
    }

    gpio_init_callback(&max86150_cb_data, max86150_irq_callback, BIT(max86150_int_gpio.pin));
    gpio_add_callback(max86150_int_gpio.port, &max86150_cb_data);
    LOG_INF("Interrupt configured on %s pin %d", max86150_int_gpio.port->name, max86150_int_gpio.pin);
    return true;
}

void check_max86150(void)
{
    // Process all available samples in the FIFO
    /*
    while (MAX86150_available() > 0) {
        data_available = MAX86150_check();
        // (Optionally, add code here to process or advance the FIFO pointer)
    }
        */
    MAX86150_check();

    // Clear the data ready interrupt by reading the INTSTAT registers.
    // The read values are discarded.
    //MAX86150_readRegister8(MAX86150_INTSTAT1);
    //MAX86150_readRegister8(MAX86150_INTSTAT2);

    // Correct head pointer wrap-around
    if ((sense.head > STORAGE_SIZE) && (init_flag == 0)) {
        sense.head = 0;
        init_flag = 1;
    }
    if (sense.head > STORAGE_SIZE) {
        sense.head = 0;
    }
}

/*
void check_max86150(void)
{
    data_available = MAX86150_check();
    if (data_available != 0) {
        //You can add debug printing here if desired.
    }
    if ((sense.head > STORAGE_SIZE) && (init_flag == 0)) {
        sense.head = 0;
        init_flag = 1;
    }
    if (sense.head > STORAGE_SIZE) {
        sense.head = 0;
    }
}
*/
uint8_t MAX86150_getINT1(void)
{
    return MAX86150_readRegister8(MAX86150_INTSTAT1);
}

uint8_t MAX86150_getINT2(void)
{
    return MAX86150_readRegister8(MAX86150_INTSTAT2);
}

void MAX86150_enableAFULL(void)
{
    MAX86150_bitMask(MAX86150_INTENABLE1, MAX86150_INT_A_FULL_MASK, MAX86150_INT_A_FULL_ENABLE);
}

void MAX86150_disableAFULL(void)
{
    MAX86150_bitMask(MAX86150_INTENABLE1, MAX86150_INT_A_FULL_MASK, MAX86150_INT_A_FULL_DISABLE);
}

void MAX86150_enableDATARDY(void)
{
    MAX86150_bitMask(MAX86150_INTENABLE1, MAX86150_INT_DATA_RDY_MASK, MAX86150_INT_DATA_RDY_ENABLE);
}

void MAX86150_disableDATARDY(void)
{
    MAX86150_bitMask(MAX86150_INTENABLE1, MAX86150_INT_DATA_RDY_MASK, MAX86150_INT_DATA_RDY_DISABLE);
}

void MAX86150_enableALCOVF(void)
{
    MAX86150_bitMask(MAX86150_INTENABLE1, MAX86150_INT_ALC_OVF_MASK, MAX86150_INT_ALC_OVF_ENABLE);
}

void MAX86150_disableALCOVF(void)
{
    MAX86150_bitMask(MAX86150_INTENABLE1, MAX86150_INT_ALC_OVF_MASK, MAX86150_INT_ALC_OVF_DISABLE);
}

void MAX86150_enablePROXINT(void)
{
    MAX86150_bitMask(MAX86150_INTENABLE1, MAX86150_INT_PROX_INT_MASK, MAX86150_INT_PROX_INT_ENABLE);
}

void MAX86150_disablePROXINT(void)
{
    MAX86150_bitMask(MAX86150_INTENABLE1, MAX86150_INT_PROX_INT_MASK, MAX86150_INT_PROX_INT_DISABLE);
}

void MAX86150_softReset(void)
{
    MAX86150_bitMask(MAX86150_SYSCONTROL, MAX86150_RESET_MASK, MAX86150_RESET);
    /* Poll for reset completion (timeout ~1000ms) */
    uint32_t delayed = 0;
    while (delayed < 1000) {
        uint8_t response = MAX86150_readRegister8(MAX86150_SYSCONTROL);
        if ((response & MAX86150_RESET) == 0) {
            break;
        }
        k_msleep(1);
        delayed++;
    }
}

void MAX86150_shutDown(void)
{
    MAX86150_bitMask(MAX86150_SYSCONTROL, MAX86150_SHUTDOWN_MASK, MAX86150_SHUTDOWN);
}

void MAX86150_wakeUp(void)
{
    MAX86150_bitMask(MAX86150_SYSCONTROL, MAX86150_SHUTDOWN_MASK, MAX86150_WAKEUP);
}

void MAX86150_setLEDMode(uint8_t mode)
{
    // The original code commented this out. Implement as needed.
    // MAX86150_bitMask(MAX86150_PPGCONFIG1, MAX86150_MODE_MASK, mode);
}

void MAX86150_setADCRange(uint8_t adcRange)
{
    // Implement ADC range setting if needed.
    // Example:
    // MAX86150_bitMask(PARTICLECONFIG, MAX86150_ADCRANGE_MASK, adcRange);
}

void MAX86150_setSampleRate(uint8_t sampleRate)
{
    // Implement sample rate setting if needed.
    // MAX86150_bitMask(PARTICLECONFIG, MAX86150_SAMPLERATE_MASK, sampleRate);
}

void MAX86150_setPulseWidth(uint8_t pulseWidth)
{
    // Implement pulse width setting if needed.
    // MAX86150_bitMask(MAX86150_PPGCONFIG1, MAX86150_PULSEWIDTH_MASK, pulseWidth);
}

void MAX86150_setPulseAmplitudeRed(uint8_t amplitude)
{
    MAX86150_writeRegister8(MAX86150_LED2_PULSEAMP, amplitude);
}

void MAX86150_setPulseAmplitudeIR(uint8_t amplitude)
{
    MAX86150_writeRegister8(MAX86150_LED1_PULSEAMP, amplitude);
}

void MAX86150_setPulseAmplitudeProximity(uint8_t amplitude)
{
    MAX86150_writeRegister8(MAX86150_LED_PROX_AMP, amplitude);
}

void MAX86150_setProximityThreshold(uint8_t threshMSB)
{
    MAX86150_writeRegister8(MAX86150_PROXINTTHRESH, threshMSB);
}

void MAX86150_enableSlot(uint8_t slotNumber, uint8_t device)
{
    switch (slotNumber) {
    case 1:
        MAX86150_bitMask(MAX86150_FIFOCONTROL1, MAX86150_SLOT1_MASK, device);
        break;
    case 2:
        MAX86150_bitMask(MAX86150_FIFOCONTROL1, MAX86150_SLOT2_MASK, device << 4);
        break;
    case 3:
        MAX86150_bitMask(MAX86150_FIFOCONTROL2, MAX86150_SLOT3_MASK, device);
        break;
    case 4:
        MAX86150_bitMask(MAX86150_FIFOCONTROL2, MAX86150_SLOT4_MASK, device << 4);
        break;
    default:
        break;
    }
}

void MAX86150_disableSlots(void)
{
    MAX86150_writeRegister8(MAX86150_FIFOCONTROL1, 0);
    MAX86150_writeRegister8(MAX86150_FIFOCONTROL2, 0);
}

void MAX86150_setFIFOAverage(uint8_t numberOfSamples)
{
    MAX86150_bitMask(MAX86150_FIFOCONFIG, MAX86150_SAMPLEAVG_MASK, numberOfSamples);
}

void MAX86150_clearFIFO(void)
{
    MAX86150_writeRegister8(MAX86150_FIFOWRITEPTR, 0);
    MAX86150_writeRegister8(MAX86150_FIFOOVERFLOW, 0);
    MAX86150_writeRegister8(MAX86150_FIFOREADPTR, 0);
}

void MAX86150_enableFIFORollover(void)
{
    MAX86150_bitMask(MAX86150_FIFOCONFIG, MAX86150_ROLLOVER_MASK, MAX86150_ROLLOVER_ENABLE);
}

void MAX86150_disableFIFORollover(void)
{
    MAX86150_bitMask(MAX86150_FIFOCONFIG, MAX86150_ROLLOVER_MASK, MAX86150_ROLLOVER_DISABLE);
}

void MAX86150_setFIFOAlmostFull(uint8_t numberOfSamples)
{
    MAX86150_bitMask(MAX86150_FIFOCONFIG, MAX86150_A_FULL_MASK, numberOfSamples);
}

uint8_t MAX86150_getWritePointer(void)
{
    return MAX86150_readRegister8(MAX86150_FIFOWRITEPTR);
}

uint8_t MAX86150_getReadPointer(void)
{
    return MAX86150_readRegister8(MAX86150_FIFOREADPTR);
}

uint8_t MAX86150_checkOVF(void)
{
    return MAX86150_readRegister8(MAX86150_FIFOOVERFLOW);
}

void MAX86150_setPROXINTTHRESH(uint8_t val)
{
    MAX86150_writeRegister8(MAX86150_PROXINTTHRESH, val);
}

uint8_t MAX86150_readPartID(void)
{
    return MAX86150_readRegister8(MAX86150_PARTID);
}

void MAX86150_setup(uint8_t sampleAverage)
{
    MAX86150_writeRegister8(MAX86150_SYSCONTROL, 0x01);
    k_msleep(100);

    MAX86150_writeRegister8(MAX86150_FIFOCONFIG, 0x1F);

    /* The original code for setting FIFO average is commented out.
       You can uncomment and adjust as needed.
    */
     if (sampleAverage == 1) MAX86150_setFIFOAverage(MAX86150_SAMPLEAVG_1);
     else if (sampleAverage == 2) MAX86150_setFIFOAverage(MAX86150_SAMPLEAVG_2);
     else if (sampleAverage == 4) MAX86150_setFIFOAverage(MAX86150_SAMPLEAVG_4);
     else if (sampleAverage == 8) MAX86150_setFIFOAverage(MAX86150_SAMPLEAVG_8);
     else if (sampleAverage == 16) MAX86150_setFIFOAverage(MAX86150_SAMPLEAVG_16);
     else if (sampleAverage == 32) MAX86150_setFIFOAverage(MAX86150_SAMPLEAVG_32);
     else MAX86150_setFIFOAverage(MAX86150_SAMPLEAVG_4);

    MAX86150_writeRegister8(MAX86150_FIFOCONTROL1, 0x21);
    MAX86150_writeRegister8(MAX86150_FIFOCONTROL2, 0x00);

    MAX86150_setPulseAmplitudeRed(0x25);
    /* Example: adjust IR amplitude based on external channel configuration.
       Assume Get_ADS_nr_of_channels() is defined elsewhere.
    */
    // if (Get_ADS_nr_of_channels() == USE_CH1_ONLY) {
    //     MAX86150_setPulseAmplitudeIR(0);
    // } else {
    //     MAX86150_setPulseAmplitudeIR(0x25);
    // }
    MAX86150_setPulseAmplitudeIR(0x25);

    MAX86150_writeRegister8(MAX86150_PPGCONFIG1, 0b11010011); // 100 SPS
    MAX86150_writeRegister8(MAX86150_PPGCONFIG2, 0x18);         // configuration value

    MAX86150_writeRegister8(MAX86150_LED_RANGE, 0x00); // PPG_ADC_RGE: 32768nA

    // MAX86150_writeRegister8(MAX86150_ECG_CONFIG1,0x03);
    // MAX86150_writeRegister8(MAX86150_ECG_CONFIG3,0x0E);

    MAX86150_writeRegister8(MAX86150_SYSCONTROL, 0x04); // Start FIFO

    MAX86150_clearFIFO();

    Set_PPG_Function(PPG_ACTIVE);
}

uint8_t MAX86150_available(void)
{
    int8_t numberOfSamples = sense.head - sense.tail;
    if (numberOfSamples < 0) {
        numberOfSamples += STORAGE_SIZE;
    }
    return (uint8_t)numberOfSamples;
}

uint32_t MAX86150_getRed(void)
{
    if (MAX86150_safeCheck(250))
        return sense.red[sense.head];
    else
        return 0;
}

uint32_t MAX86150_getIR(void)
{
    if (MAX86150_safeCheck(250))
        return sense.IR[sense.head];
    else
        return 0;
}

int32_t MAX86150_getECG(void)
{
    if (MAX86150_safeCheck(250)) {
        // Return ECG data if implemented.
        return 0;
    } else {
        return 0;
    }
}

uint32_t MAX86150_getFIFORed(void)
{
    return sense.red[sense.tail];
}

uint32_t MAX86150_getFIFOIR(void)
{
    return sense.IR[sense.tail];
}

int32_t MAX86150_getFIFOECG(void)
{
    // Return ECG data if implemented.
    return 0;
}

void MAX86150_nextSample(void)
{
    if (MAX86150_available()) {
        sense.tail++;
        sense.tail %= STORAGE_SIZE;
    }
}

uint16_t MAX86150_check(void)
{
    int numberOfSamples = 0;
    uint8_t readPointer = MAX86150_getReadPointer();
    uint8_t writePointer = MAX86150_getWritePointer();
    uint8_t OVF = MAX86150_checkOVF();
    int byte_index = 0;

    if (readPointer != writePointer) {
        numberOfSamples = writePointer - readPointer;
        if (numberOfSamples < 0) {
            numberOfSamples += 32;
        }
        if (OVF > 0) {
            numberOfSamples = 32;
        }

        int bytesLeftToRead = numberOfSamples * activeDevices * 3;
        int bytesPerRead = activeDevices * 3;
        uint8_t reg = MAX86150_FIFODATA;

        /* Write the FIFO data register address */
        if (i2c_write_dt(&dev_i2c, &reg, 1) < 0) {
            return 0;
        }

        while (bytesLeftToRead > 0) {
            byte_index = 0;
            if (i2c_read_dt(&dev_i2c, FIFO_data, bytesPerRead) < 0) {
                break;
            }

            sense.head++;
            sense.head %= STORAGE_SIZE;

            uint8_t temp[sizeof(uint32_t)];
            uint32_t tempLong;

            /* Read 3 bytes for RED */
            temp[3] = 0;
            temp[2] = FIFO_data[byte_index++];
            temp[1] = FIFO_data[byte_index++];
            temp[0] = FIFO_data[byte_index++];
            memcpy(&tempLong, temp, sizeof(tempLong));
            tempLong &= 0x7FFFF;
            sense.red[sense.head] = tempLong;

            if (activeDevices > 1) {
                /* Read 3 bytes for IR */
                temp[3] = 0;
                temp[2] = FIFO_data[byte_index++];
                temp[1] = FIFO_data[byte_index++];
                temp[0] = FIFO_data[byte_index++];
                memcpy(&tempLong, temp, sizeof(tempLong));
                tempLong &= 0x7FFFF;
                sense.IR[sense.head] = tempLong;
            }

            if (activeDevices > 2) {
                int32_t tempLongSigned;
                temp[3] = 0;
                temp[2] = FIFO_data[byte_index++];
                temp[1] = FIFO_data[byte_index++];
                temp[0] = FIFO_data[byte_index++];
                memcpy(&tempLongSigned, temp, sizeof(tempLongSigned));
                // Process ECG value if needed:
                // sense.ecg[sense.head] = (tempLongSigned << 14) >> 14;
            }
            /* In the original code, bytesLeftToRead is reduced by activeDevices * 3 * 3.
               (This looks like an extra factor of 3; preserve the behavior.)
            */
            bytesLeftToRead -= activeDevices * 3 * 3;
        }
    }
    // Reset the data ready interrupt
    MAX86150_readRegister8(MAX86150_INTSTAT1);

    return numberOfSamples;



}

void MAX86150_bitMask(uint8_t reg, uint8_t mask, uint8_t thing)
{
    uint8_t originalContents = MAX86150_readRegister8(reg);
    originalContents &= mask;
    MAX86150_writeRegister8(reg, originalContents | thing);
}

uint8_t MAX86150_readRegister8(uint8_t reg)
{
    uint8_t data = 0;
    /* Write register address */
    if (i2c_write_dt(&dev_i2c, &reg, 1) < 0) {
        return 0;
    }
    /* Read one byte */
    if (i2c_read_dt(&dev_i2c, &data, 1) < 0) {
        return 0;
    }
    return data;
}

void MAX86150_writeRegister8(uint8_t reg, uint8_t value)
{
    uint8_t data[2] = { reg, value };
    i2c_write_dt(&dev_i2c, data, sizeof(data));
}

enum MAX20303_PPG_function_t Get_PPG_Function(void)
{
    return MAX20303_PPG_function;
}

void Set_PPG_Function(enum MAX20303_PPG_function_t f)
{
    MAX20303_PPG_function = f;
}

//K_THREAD_DEFINE(ppg_tid, PPG_STACK_SIZE, ppg_receive_thread, NULL, NULL, NULL, PPG_PRIORITY, 0, 0);

