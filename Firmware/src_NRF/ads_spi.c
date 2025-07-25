/*
 * ----------------------------------------------------------------------
 *
 * File: ads_spi.c
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
#include "ads_spi.h"

#include <zephyr/drivers/spi.h>

#include <zephyr/logging/log.h>
#include <zephyr/logging/log_ctrl.h>

#include <nrfx_spim.h>

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/devicetree.h>

#include <zephyr/drivers/gpio.h>

#include "ads_appl.h"
#include "ble_appl.h"
#include "ppg_appl.h"
#include "lis2duxs12_sensor.h"

#include "pwr/pwr.h"
#include "bsp/pwr_bsp.h"
#include "pwr/pwr_common.h"


LOG_MODULE_REGISTER(ads_spi, LOG_LEVEL_DBG);

#define ADS_A_DR_NODE DT_NODELABEL(gpio_ads1298_a_dr)

#define CS_A_NODE DT_NODELABEL(gpio_ads1298_a_spi_cs)
#define CS_B_NODE DT_NODELABEL(gpio_ads1298_b_spi_cs)
#define ADS_START_NODE DT_NODELABEL(gpio_ads1298_start_pin)

static const struct gpio_dt_spec gpio_dt_ads1298_a_cs = GPIO_DT_SPEC_GET(CS_A_NODE, gpios);
static const struct gpio_dt_spec gpio_dt_ads1298_b_cs = GPIO_DT_SPEC_GET(CS_B_NODE, gpios);
static const struct gpio_dt_spec gpio_dt_ads1298_start_pin = GPIO_DT_SPEC_GET(ADS_START_NODE, gpios);

static const struct gpio_dt_spec gpio_dt_ads1298_a_dr = GPIO_DT_SPEC_GET(ADS_A_DR_NODE, gpios);


#define SPI_INT_PRIO 1

#define SPIOP	SPI_WORD_SET(8) | SPI_TRANSFER_MSB
struct spi_dt_spec spispec = SPI_DT_SPEC_GET(DT_NODELABEL(ads1298_a), SPIOP, 0);


uint8_t counter_extra = 0; // An extra counter that is read out in the Matlab conversion script



uint8_t pr_word[10]={_RESET,0,0,0,0,0,0,0,0,0};
static uint32_t      tx_buf_inx=0;

uint8_t       ble_tx_buf[PCK_LNGTH]={0}; // To build the BLE packet
uint8_t counter = 0;
static uint8_t       empty_buffer[27]={0};           /**< TX buffer. */
static uint8_t       ads_rx_buf[40];            /**< ADS RX SPI buffer. */

bool skip_reads = true;
int skiped_samples = 0;
bool ads_initialized = false;
static volatile bool ads_to_read = ADS1298_A;

static struct k_mutex spi_mutex;

// PPG sensor variables
extern sense_struct sense;



static volatile bool spi_xfer_done=true;  /**< Flag used to indicate that SPI instance completed the transfer. */
static volatile bool ads_data_ready = false;

bool drdy_served = true;
static bool pck_ble_ready = false;


nrfx_spim_t spim_inst = NRFX_SPIM_INSTANCE(SPIM_INST_IDX);

//struct spi_dt_spec spispec = SPI_DT_SPEC_GET(DT_NODELABEL(ads1298_a), SPI_WORD_SET(8), 0);

/**
 * @brief Function for handling SPIM driver events.
 *
 * @param[in] p_event   Pointer to the SPIM driver event.
 * @param[in] p_context Pointer to the context passed from the driver.
 */
static void spim_handler(nrfx_spim_evt_t const * p_event, void * p_context)
{

    if (p_event->type == NRFX_SPIM_EVENT_DONE)
    {
        if (gpio_pin_set_dt(&gpio_dt_ads1298_a_cs, 0) < 0) {  // Set CS pin to disable
            LOG_ERR("ADS1298 power GPIO set error");
            return -1;
        }
        if (gpio_pin_set_dt(&gpio_dt_ads1298_b_cs, 0) < 0) {  // Set CS pin to disable
            LOG_ERR("ADS1298 power GPIO set error");
            return -1;
        }
        
        if (Get_ADS_Function() == READ)
        {
            
            //printf("\nADS DATA received -- ADS is in READ\n");
            memcpy(&ble_tx_buf[tx_buf_inx], &ads_rx_buf[3], 24);
            tx_buf_inx+=24;

            if(Get_PPG_Function()==PPG_ACTIVE)
            {
                // Replace last two EEG channels with PPG data (IR and red LED)
                tx_buf_inx-=6;

                // Get the pointers to the start of the most recent PPG data
                uint8_t * address_red = (uint8_t *) &sense.red[sense.head];
                uint8_t * address_IR = (uint8_t *) &sense.IR[sense.head];
                // Add the PPG data such that the Biowolf GUI interprets it correctly
                memcpy(&ble_tx_buf[tx_buf_inx], address_red + 2, 1);
                tx_buf_inx++;
                memcpy(&ble_tx_buf[tx_buf_inx], address_red + 1, 1);
                tx_buf_inx++;
                memcpy(&ble_tx_buf[tx_buf_inx], address_red + 0, 1);
                tx_buf_inx++;
                //printf("%d\n", sense.red[sense.head]);

                memcpy(&ble_tx_buf[tx_buf_inx], address_IR + 2, 1);
                tx_buf_inx++;
                memcpy(&ble_tx_buf[tx_buf_inx], address_IR + 1, 1);
                tx_buf_inx++;
                memcpy(&ble_tx_buf[tx_buf_inx], address_IR + 0, 1);
                tx_buf_inx++;
            }
            
            
            if(ads_to_read == ADS1298_B)
            {
                //Attach IMU data
                ble_tx_buf[tx_buf_inx++] = (uint8_t) (data_xl.raw[0]>>8); //IMU-xH
                ble_tx_buf[tx_buf_inx++] = (uint8_t)( (data_xl.raw[0]) & (0x00FF) );  //IMU-xL

                ble_tx_buf[tx_buf_inx++] = (uint8_t) (data_xl.raw[1]>>8); //IMU-yH
                ble_tx_buf[tx_buf_inx++] = (uint8_t)( (data_xl.raw[1]) & (0x00FF) );  //IMU-yL

                ble_tx_buf[tx_buf_inx++] = (uint8_t) (data_xl.raw[2]>>8); //IMU-zH
                ble_tx_buf[tx_buf_inx++] = (uint8_t)( (data_xl.raw[2]) & (0x00FF) );  //IMU-zL
            
                //Attach other data
                // Set packet identifier to EEG all channels + PPG inactive
                ble_tx_buf[tx_buf_inx++] = counter_extra;                      //add here your custom data for each sample.
                ble_tx_buf[tx_buf_inx++] = get_trigger();             //this will capture the tigger per sample send throught UART BLE
            
            }
            
            if (tx_buf_inx ==226)
            {
                    //Check if the last pck was handled and reset flag
                
                if(pck_ble_ready == true)
                {
                    //printf("\nADS DATA received -- stop ADS\n");
                    Set_ADS_Function(STOP);
                    pck_ble_ready = false;
                    LOG_INF("Data packet not processed -- stop ADS");
                }
                else
                {
                    
                    //Reset and condition BLE buffers
                    tx_buf_inx = 0; 

                    //Prepare the next buffer
                    ble_tx_buf[tx_buf_inx++] = BLE_PCK_HEADER;
                    ble_tx_buf[tx_buf_inx++] = ++counter;


                    // Finish up and send
                    //Get the index to write
                    
                    int buf_current_size = 226;


                    //Here you can add information that you would send every now and then (every 7 samples)
                    ble_tx_buf[buf_current_size++] = (uint8_t) data_temp.heat.deg_c; // Temperature
                    ble_tx_buf[buf_current_size++] = 0x00;
                    
                    //Send Battery state
                    ble_tx_buf[buf_current_size++] = 0; //max20303_pmic_check_usb();
                    ble_tx_buf[buf_current_size++] = 0;        
                    ble_tx_buf[buf_current_size++] = 0;
    
                    ble_tx_buf[buf_current_size++] = (uint8_t) bsp_get_battery_soc();
                    ble_tx_buf[buf_current_size++] = (uint8_t) bsp_get_battery_voltage();
    
    
                    //BLE PCK tail
                    ble_tx_buf[buf_current_size++] = BLE_PCK_TAILER;

                    add_data_to_send_buffer(ble_tx_buf);
                    
                }
            }
            
            
            drdy_served = true;
            //k_sem_give(&sem_drdy_served);

        }
        

        spi_xfer_done = true;
    }
}

// Callback function for the ADS1298 A data ready pin
void cb_ads_a_dr(const struct device *dev, struct gpio_callback *cb, uint32_t pins)
{
    // Set flag indicating new data is ready
    //LOG_INF("DRDY interrupt received");
    ads_data_ready = true;
    counter_extra = counter_extra + 1;

}
// Define a variable of type static struct gpio_callback
static struct gpio_callback ads1298_a_dr_cb_data;





void init_SPI()
{
nrfx_err_t status;
    (void)status;

#if defined(__ZEPHYR__)
    IRQ_CONNECT(NRFX_IRQ_NUMBER_GET(NRF_SPIM_INST_GET(SPIM_INST_IDX)), SPI_INT_PRIO,
                NRFX_SPIM_INST_HANDLER_GET(SPIM_INST_IDX), 0, 0);
#endif


    
    //ADS_A_CS_PIN
    nrfx_spim_config_t spim_config = NRFX_SPIM_DEFAULT_CONFIG(SCK_PIN,
                                                              MOSI_PIN,
                                                              MISO_PIN,
                                                              NRF_SPIM_PIN_NOT_CONNECTED
);

    spim_config.frequency = NRFX_MHZ_TO_HZ(4);
    spim_config.mode = NRF_SPIM_MODE_1;
    spim_config.bit_order = NRF_SPIM_BIT_ORDER_MSB_FIRST;
    spim_config.irq_priority = SPI_INT_PRIO;

    void * p_context = "Some context";
    status = nrfx_spim_init(&spim_inst, &spim_config, spim_handler, p_context);
    NRFX_ASSERT(status == NRFX_SUCCESS);
    
    // Initialize SPI CS pin for ADS A
    if (!device_is_ready(gpio_dt_ads1298_a_cs.port)) {
        LOG_ERR("ADS1298 power GPIO port not ready");
        return -1;
    }
    if (gpio_pin_configure_dt(&gpio_dt_ads1298_a_cs, GPIO_OUTPUT_INACTIVE) < 0) {
      LOG_ERR("ADS pwr GPIO init error");
      return 0;
    }

    // Initialize SPI CS pin for ADS B
    if (!device_is_ready(gpio_dt_ads1298_b_cs.port)) {
        LOG_ERR("ADS1298 power GPIO port not ready");
        return -1;
    }
    if (gpio_pin_configure_dt(&gpio_dt_ads1298_b_cs, GPIO_OUTPUT_INACTIVE) < 0) {
      LOG_ERR("ADS pwr GPIO init error");
      return 0;
    }

    // Initialize SPI START pin for synchronized start of ADS A and B
    if (!device_is_ready(gpio_dt_ads1298_start_pin.port)) {
        LOG_ERR("ADS1298 power GPIO port not ready");
        return -1;
    }
    if (gpio_pin_configure_dt(&gpio_dt_ads1298_start_pin, GPIO_OUTPUT_INACTIVE) < 0) {
      LOG_ERR("ADS pwr GPIO init error");
      return 0;
    }
    
    
    
    k_mutex_init(&spi_mutex);

}


static int ads1298_read_spi(uint8_t *data, uint8_t size, enum ADS_id_t ads_id)
{
	int err;

    unsigned int key = irq_lock();  // Disable all interrupts
    k_mutex_lock(&spi_mutex, K_FOREVER);
    nrfx_spim_xfer_desc_t spim_xfer_desc = NRFX_SPIM_XFER_TRX(pr_word, size,
                                                              ads_rx_buf, size);

    if(ads_id == ADS1298_A)
    {
        if (gpio_pin_set_dt(&gpio_dt_ads1298_a_cs, 1) < 0) {  // Set pin to enable
            LOG_ERR("ADS1298 power GPIO set error");
            return -1;
        }
    }
    else
    {
        if (gpio_pin_set_dt(&gpio_dt_ads1298_b_cs, 1) < 0) {  // Set pin to enable
            LOG_ERR("ADS1298 power GPIO set error");
            return -1;
        }
    }
    nrfx_err_t status;
    status = nrfx_spim_xfer(&spim_inst, &spim_xfer_desc, 0);
    NRFX_ASSERT(status == NRFX_SUCCESS);
    
    k_mutex_unlock(&spi_mutex);
    irq_unlock(key);  // Restore interrupts

	return 0;
}

static int ads1298_read_samples(uint8_t *data, uint8_t size, enum ADS_id_t ads_id)
{
	int err;

    unsigned int key = irq_lock();  // Disable all interrupts
    k_mutex_lock(&spi_mutex, K_FOREVER);
    
    nrfx_spim_xfer_desc_t spim_xfer_desc = NRFX_SPIM_XFER_TRX(empty_buffer, size,
                                                              ads_rx_buf, size);
    if(ads_id == ADS1298_A)
    {
        if (gpio_pin_set_dt(&gpio_dt_ads1298_a_cs, 1) < 0) {  // Set pin to enable
            LOG_ERR("ADS1298 power GPIO set error");
            return -1;
        }
    }
    else
    {
        if (gpio_pin_set_dt(&gpio_dt_ads1298_b_cs, 1) < 0) {  // Set pin to enable
            LOG_ERR("ADS1298 power GPIO set error");
            return -1;
        }
    }
    nrfx_err_t status;
    status = nrfx_spim_xfer(&spim_inst, &spim_xfer_desc, 0);
    NRFX_ASSERT(status == NRFX_SUCCESS);
    k_mutex_unlock(&spi_mutex);
    irq_unlock(key);  // Restore interrupts

	return 0;
}

static int ads1298_write_spi(uint8_t size, enum ADS_id_t ads_id)
{


    nrfx_spim_xfer_desc_t spim_xfer_desc = NRFX_SPIM_XFER_TRX(pr_word, sizeof(pr_word),
                                                              ads_rx_buf, sizeof(pr_word));
    nrfx_err_t status;
    if(ads_id == ADS1298_A)
    {
        if (gpio_pin_set_dt(&gpio_dt_ads1298_a_cs, 1) < 0) {  // Set pin to enable
            LOG_ERR("ADS1298 power GPIO set error");
            return -1;
        }
    }
    else
    {
        if (gpio_pin_set_dt(&gpio_dt_ads1298_b_cs, 1) < 0) {  // Set pin to enable
            LOG_ERR("ADS1298 power GPIO set error");
            return -1;
        }
    }
    status = nrfx_spim_xfer(&spim_inst, &spim_xfer_desc, 0);
    NRFX_ASSERT(status == NRFX_SUCCESS);

	return 0;
}




void ADS_check_ID(enum ADS_id_t ads_id)
{
    int err;

 
    /*ADS INIZIALIZATION*/
    // RESET DEVICE
    pr_word[0]=_RESET;
    ads1298_write_spi(1, ads_id);
    k_msleep(30);
    
    // STOP DEVICE
    pr_word[0]=_SDATAC;
    ads1298_write_spi(1, ads_id);
    k_msleep(30);

    // Read out device ID
    pr_word[0]=_RREG | ID;
    pr_word[1]= 0;
    ads1298_read_spi(ads_rx_buf, 3, ads_id);
    k_msleep(30);

    if(ads_rx_buf[2]!=0xd2)
    {
        // Wait here if ID is not correct
        while(1){ LOG_ERR("ADS1298 ID not correct");}
    }
}

int ADS_dr_read(){

    return gpio_pin_get_dt(&gpio_dt_ads1298_a_dr);

}

int ADS_dr_init()
{
    // Initialize the data ready pin
    if (!device_is_ready(gpio_dt_ads1298_a_dr.port)) {
        LOG_ERR("ADS1298 DRDY GPIO port not ready");
        return -1;
    }
    if (gpio_pin_configure_dt(&gpio_dt_ads1298_a_dr, GPIO_INPUT) < 0) {
        LOG_ERR("ADS1298 DRDY GPIO init error");
        return -1;
    }
    // Enable interrrupt
    int ret;
    ret = gpio_pin_interrupt_configure_dt(&gpio_dt_ads1298_a_dr, GPIO_INT_EDGE_TO_ACTIVE);
    if (ret < 0) {
        LOG_ERR("ADS1298 DRDY GPIO interrupt enable error");
        return -1;
    }
    
    // Initialize the static struct gpio_callback variable
	gpio_init_callback(&ads1298_a_dr_cb_data, cb_ads_a_dr, BIT(gpio_dt_ads1298_a_dr.pin));
	// Add the callback function by calling gpio_add_callback()
	ret = gpio_add_callback(gpio_dt_ads1298_a_dr.port, &ads1298_a_dr_cb_data);
    if (ret < 0) {
        LOG_ERR("ADS1298 DRDY GPIO interrupt enable error");
        return -1;
    }

    return 0;
}

void ADS_Init(uint8_t *InitParams, enum ADS_id_t ads_id)
{

    //buffer_counter = 0;
    tx_buf_inx = 0;
    ble_tx_buf[tx_buf_inx++] = BLE_PCK_HEADER;
    ble_tx_buf[tx_buf_inx++] = ++counter;

    /*Work Arround for the issue of the SPI blocking execution sometimes. This forses CS to reset the bus*/
    //pr_word[0]=_RESET;
    //APP_ERROR_CHECK(nrf_drv_spi_transfer(&spi, pr_word, 1, empty_buffer, 1));

    /*ADS INIZIALIZATION*/
    // RESET DEVICE
    pr_word[0]=_RESET;
    spi_xfer_done=false;
    ads1298_write_spi(1, ads_id);
    while(spi_xfer_done==false);
    k_msleep(30);

    // STOP DEVICE
    pr_word[0]=_SDATAC;
    spi_xfer_done=false;
    ads1298_write_spi(1, ads_id);
    while(spi_xfer_done==false);
    k_msleep(30);
    
    // WRITE REGS FROM REG1 - Set SampleRate FROM INIT - Test Signal 1 Hz, Ref. Buffer OFF
    //pr_word[0]=_WREG|CONFIG1; pr_word[1]=InitParams[0]; pr_word[2]=(0xE0 +InitParams[0]); pr_word[3]=0x55; pr_word[4]=0xC0;
    //pr_word[0]=_WREG|CONFIG1; pr_word[1]=2; pr_word[2]= 0xE0 + InitParams[0] ; pr_word[3]=0x55; pr_word[4]=0xC0;
    
    pr_word[0]=_WREG|CONFIG1; pr_word[1]=2; pr_word[2]=0xC0+InitParams[0]; pr_word[3]=0x55; pr_word[4]=0xC0; //pr_word[4]=0b11001000; //pr_word[4]=0xC0;
    //pr_word[0]=_WREG|CONFIG1; pr_word[1]=2; pr_word[2]= 0xE5; pr_word[3]=0x55; pr_word[4]=0xC0;
    spi_xfer_done=false;
    ads1298_write_spi(5, ads_id);
    while(spi_xfer_done==false);
    
    // SET CHANNEL REGS - ON, GAIN 6, SHORTED ELECTRODE INPUT
    pr_word[0]=_WREG|CH1SET; pr_word[1]=7;
    //#ifdef ONE_CHANNEL_ONLY
    //pr_word[2]=InitParams[4]|InitParams[1]; // Configure first channel
    //for (int i=3; i<10; i++)
    //        pr_word[i]=0b10000000|InitParams[4]|InitParams[1]; // Disable all other channels
    //        //pr_word[i]=0x60;
    //#else
    //for (int i=2; i<10; i++)
    //        pr_word[i]=InitParams[4]|InitParams[1];
    //        //pr_word[i]=0x60;
    //#endif

    for (int i=2; i<10; i++)
    {
        pr_word[i]=InitParams[4]|InitParams[1];
    }
            //pr_word[i]=0x60;
    
    spi_xfer_done=false;
    ads1298_write_spi(10, ads_id);
    while(spi_xfer_done==false);
    k_msleep(30);



    ads_initialized = true;
}


void ADS_Stop()
{
   // STOP DEVICE
   
    skip_reads = true;  //Flag to skip the fist samples as to make sure the signal is stable.

    /*
    if (gpio_pin_set_dt(&gpio_dt_ads1298_start_pin, 0) < 0) {  // Set START pin to disable
        LOG_ERR("ADS1298 power GPIO set error");
        return -1;
    }
    */
    pr_word[0]=_SDATAC;
    spi_xfer_done=false;
    ads1298_write_spi(1, ADS1298_A);
    while(spi_xfer_done==false);
    k_msleep(30);

    pr_word[0]=_SDATAC;
    spi_xfer_done=false;
    ads1298_write_spi(1, ADS1298_B);
    while(spi_xfer_done==false);
}

void ADS_Start()
{
    //printf("...Starting ADS...\n");
    
    pr_word[0]=_START;
    pr_word[1]=_RDATAC;
    spi_xfer_done=false;
    ads1298_write_spi(2, ADS1298_A);
    while(spi_xfer_done==false);

    pr_word[0]=_START;
    pr_word[1]=_RDATAC;
    spi_xfer_done=false;
    ads1298_write_spi(2, ADS1298_B);
    while(spi_xfer_done==false);
    
    
}

void process_ads_data(void)
{
    if (ads_data_ready) {
        // Clear flag first to avoid missing next interrupt
        ads_data_ready = false;
        

 
        // Process received data as needed
        if(Get_ADS_Function()==READ)
        {
    
            if(skip_reads)
            {
              if(skiped_samples++==500)
              {
                skip_reads = false;
                skiped_samples = 0;
              }
            }
      
            if(!skip_reads)
            {
                
              if(!drdy_served)
              {
                //printf("________DATA READY NOT SERVED___________\n");
                Set_ADS_Function(STOP);
              }
              else
              {
                drdy_served = false;
    
                // Read data from ADS using SPI
                //uint8_t rx_data[27]; // Adjust size based on your data format
                spi_xfer_done=false;
                ads_to_read = ADS1298_A;
                ads1298_read_samples(ads_rx_buf, 27, ADS1298_A);
                while(spi_xfer_done==false);
                spi_xfer_done=false;
                ads_to_read = ADS1298_B;
                ads1298_read_samples(ads_rx_buf, 27, ADS1298_B);
                while(spi_xfer_done==false);
              }
                
            }
        }
    }
}


