/*
 * ----------------------------------------------------------------------
 *
 * File: ble_appl.c
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
#include "ble_appl.h"
#include "ads_appl.h"
#include "ads_spi.h"
#include "common.h"
#include "lis2duxs12_sensor.h"

#include <zephyr/logging/log.h>
#include <zephyr/logging/log_ctrl.h>

/* Initialize the logging module */
LOG_MODULE_REGISTER(ble_appl, LOG_LEVEL_DBG);

//#define PRINT_RECEIVED_DATA




/* Define message queues */
K_MSGQ_DEFINE(send_msgq, BLE_PCKT_SEND_SIZE, SEND_QUEUE_SIZE, 1);
K_MSGQ_DEFINE(receive_msgq, BLE_PCKT_RECEIVE_SIZE, RECEIVE_QUEUE_SIZE, 1);

/* Define stack sizes and priorities */
#define BLE_SEND_STACK_SIZE    2048
#define BLE_SEND_PRIORITY      5

#define BLE_RECEIVE_STACK_SIZE 2048
#define BLE_RECEIVE_PRIORITY   4

/* Define thread stacks */
//K_THREAD_STACK_DEFINE(ble_send_stack, BLE_SEND_STACK_SIZE);
//K_THREAD_STACK_DEFINE(ble_receive_stack, BLE_RECEIVE_STACK_SIZE);

/* Thread data structures */
//struct k_thread ble_send_tid;
//struct k_thread ble_receive_tid;



BLE_nus_data ble_data_available;

uint8_t WaitingForConfig=0;
uint8_t ConfigParams[5]={6,0,2,4,96};  // [SAMPLE_RATE ADS_MODE 2 4 GAIN]

int8_t biowolf_current_state = STATE_STREAMING_NORDIC;
uint8_t bat_data[7];

uart_to_pulp_data pck_uart_wolf;


K_SEM_DEFINE(ble_data_received, 0, 1);

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
void ble_send_thread(void *arg1, void *arg2, void *arg3)
{
    uint8_t send_data[BLE_PCKT_SEND_SIZE];
    int ret;

    LOG_INF("BLE send thread started");



    while (1) {
        // Retrieve data to send from the send_msgq 
        ret = k_msgq_get(&send_msgq, &send_data, K_FOREVER);
        if (ret == 0) {
            // Implement actual BLE send logic here 
            LOG_INF("Sending BLE data: %d", send_data);
            send_data_ble(&send_data, BLE_PCKT_SEND_SIZE);
        } else {
            LOG_ERR("Failed to get data from send_msgq (err %d)", ret);
        }


    }
}

/**
 * @brief BLE Process Received Data Thread
 *
 * This thread processes the received data from the receive message queue.
 *
 * @param arg1 Unused
 * @param arg2 Unused
 * @param arg3 Unused
 */

void process_received_data_thread(void *arg1, void *arg2, void *arg3)
{
    uint8_t data;
    int ret;

    LOG_INF("BLE receive thread started");

    while (1) {

        k_sem_take(&ble_data_received, K_FOREVER);

        #ifdef PRINT_RECEIVED_DATA
        bool is_ascii = true;
        for (int i = 0; i < ble_data_available.size; i++) {
            if (ble_data_available.data[i] < 0x20 || ble_data_available.data[i] > 0x7E) { // Non-printable ASCII range
                is_ascii = false;
                break;
            }
        }
        if (is_ascii) {
            LOG_INF("Received ASCII data: %.*s", ble_data_available.size, ble_data_available.data);
        } else {
            LOG_HEXDUMP_INF(ble_data_available.data, ble_data_available.size, "Received Binary Data:");
        }
        #endif

        // Process received data here
        if(ble_data_available.available)
    {
        ble_data_available.available = false;
        uint32_t err_code;

        LOG_DBG("Received data from BLE NUS. Writing data on UART.");
        //NRF_LOG_HEXDUMP_DEBUG(p_evt->params.rx_data.p_data, p_evt->params.rx_data.length);

        // When in STATE_PROGRAM_WOLF, other commands are ommited
        
        if(get_state_biogap() == STATE_PROGRAM_WOLF)
        {
          //Copy data and make it available
          //memcpy(pck_ble_dfu_wolf.data,ble_data_available.data,ble_data_available.size);
          //pck_ble_dfu_wolf.available = true;
          //pck_ble_dfu_wolf.size = pck_ble_dfu_wolf.data[PCK_PHY_SIZE_INX];
        }
          else
          {
            // Normal state
            uint8_t msg = ble_data_available.data[0];
            LOG_DBG("\n -----------BLE COMMAND---------- %d\n", msg);
            int k = 0;
            for(k=0; k<ble_data_available.size; k++)
            {
              LOG_DBG("\n %d \n", ble_data_available.data[k]);
            }
            LOG_DBG("\n -----------BLE COMMAND END------- \n");
            switch (msg)
            {
              case REQUEST_BATTERY_STATE:
                LOG_DBG("\nPing REQUEST_BATTERY_STATE\n");
                bat_data[0] = REQUEST_BATTERY_STATE;
                bat_data[1] = bsp_is_charging();
                bat_data[2] = 0;
                bat_data[3] = (uint8_t) bsp_get_total_power_mw();
                bat_data[4] = (uint8_t) bsp_get_battery_soc();
                bat_data[5] = (uint8_t) bsp_get_battery_voltage();
                bat_data[6] = (uint8_t) data_temp.heat.deg_c;                

                send_data_ble(bat_data,7);
                break;

              case GET_DEVICE_SETTINGS:
                SendDeviceSettings();
                break;
              
              case SET_DEVICE_SETTINGS:
                //set_device_througt_BLE_PCK(&ble_data_available.data[1], ble_data_available.size-1);
                break;

              case REQUEST_CONNECTING_STRING:
                SendReady_BLE();
                break;

            case REQUEST_HARDWARE_VERSION:
                SendHardwareVersion();
                break;

            case REQUEST_FIRMWARE_VERSION:
                SendFirmwareVersion();
                break;

            case REQUEST_AVAILABLE_SENSORS:
                SendAvailableSensors();
                break;
          
            case GET_BOARD_STATE:
                send_data_ble(&biowolf_current_state, 1);
                LOG_DBG("--> BOARD LOG --> Sent state: %d",biowolf_current_state);
                break;
          
            case SET_BOARD_STATE:
                LOG_DBG("\nPing STATE_SWITCH\n");
                LOG_DBG(".data[1], %d",ble_data_available.data[1]);
                LOG_DBG(".data[2], %d",ble_data_available.data[2]);
               
                if(ble_data_available.data[1] == 1)
                {
                  set_state_biogap(STATE_STREAMING_NORDIC);
                  //LOG_DBG("--> BOARD LOG --> New state: %d",switch_state);
                }
                else
                {
                  set_state_biogap(STATE_GAP9_MASTER);
                  //set_state_biogap(ble_data_available.data[2]);
                  //LOG_DBG("--> BOARD LOG --> New state: %d",ble_data_available.data[2]);
                }

                

                if(get_state_biogap() == STATE_STREAMING_NORDIC)
                {
                  Set_ADS_Function(STILL);
                }
                else
                {
                  Set_ADS_Function(INIT_GAP9_CTRL);
                }
            
                break;

            case RESET_GAP9:
                //Set_ADS_Function(RESTART_WOLF);
                break;

            case RESET_BOARD:
                // blabla
                break;

        case SET_TRIGGER_STATE:
            set_trigger(ble_data_available.data[1]);
            break;

        case ENTER_BOOTLOADERT_MODE:
            //nrf_sdh_disable_request();
            //nrf_power_gpregret_set(BOOTLOADER_DFU_START);
            //sd_nvic_SystemReset();
            break;

        case GO_TO_SLEEP:
            // TODO
            // IMU_NOT_SUPPORTED
            // gotosleep_NFC();
            //set_state_power(STATE_POWER_SAVE);
            break;

        default:
            LOG_DBG("\nPing default\n");
            
            if(get_state_biogap() == STATE_GAP9_MASTER)
            {
            LOG_DBG("\nPing STATE_GAP9_MASTER\n");
            pck_uart_wolf.data_len = ble_data_available.size;
            //pck_uart_wolf.p_data = &p_evt->params.rx_data.p_data[0];
            memcpy(pck_uart_wolf.p_data,ble_data_available.data,pck_uart_wolf.data_len);
            pck_uart_wolf.is_data_available = true;
            //flag_notify_wolf_uart = true;
            
            

            }
            else
            {
                LOG_DBG("\nelse\n");
                if(WaitingForConfig==0)
                {
                    LOG_DBG("\nWaitingForConfig==0\n");
                    switch(ble_data_available.data[0])
                    {
                    case START_STREAMING_NORDIC:
                    //nrf_gpio_pin_toggle(LED1);
                    set_SM_state(S_NORDIC_STREAM);
                    LOG_DBG("\nPing START_STREAMING_NORDIC\n");
                    WaitingForConfig=1;
                    Set_ADS_Function(START);
                    break;
                    case STOP_STREAMING_NORDIC:
                    set_SM_state(S_LOW_POWER_CONNECTED);
                    LOG_DBG("\nPing STOP_STREAMING_NORDIC\n");
                    //nrf_gpio_pin_toggle(LED1);
                    Set_ADS_Function(STOP);
                    break;

                    default:  
                    LOG_DBG("\ndefault\n");
                    break;
                    }
                }
                else
                {
                    LOG_DBG("\nConfig received\n");
                    WaitingForConfig=0;
                    ConfigParams[0]=ble_data_available.data[0] ;
                    ConfigParams[1]=ble_data_available.data[1] ;
                    ConfigParams[2]=ble_data_available.data[2] ;
                    ConfigParams[3]=ble_data_available.data[3] ;
                    ConfigParams[4]=ble_data_available.data[4] ;
                }            
            }
            break;
        }
    }
    }
    }
}

uint32_t GetConfigParam(uint8_t *InitParams)
{
    //WaitingForConfig=1;
    printf("\n set wait config to 1 \n");
    //SendReady_BLE();

    while(WaitingForConfig==1)
    {
      // check_post_event_flags();
      // do nothing
    }
    ; //WAIT FOR CONFIG PARAMETERS

    for (int i=0; i<5;i++)
    {
      InitParams[i]=ConfigParams[i];
    }
}



// Funtion to put data into receive buffer
void add_data_to_receive_buffer(uint8_t *data)
{
    int ret;

    ret = k_msgq_put(&receive_msgq, data, K_NO_WAIT);
    if (ret != 0) {
        LOG_ERR("Receive message queue overflow! Data not enqueued: %d (err %d)", data, ret);
    } else {
        LOG_DBG("Data enqueued for receiving: %d", data);
    }
}

/**
 * @brief Add Data to Send Buffer
 *
 * Enqueues data into the send message queue for transmission.
 *
 * @param data The pointer to the byte array to be sent over BLE.
 */
void add_data_to_send_buffer(uint8_t *data)
{
    int ret;
    //LOG_DBG("Start Data enqueued for sending");
    
    ret = k_msgq_put(&send_msgq, data, K_NO_WAIT);
    /*
    if (ret != 0) {
        LOG_ERR("Send message queue overflow! Data not sent: %d (err %d)", data, ret);
    } else {
        LOG_DBG("Data enqueued for sending: %d", data);
    }
    */
    
}


/**
 * @brief Initialize BLE Communication
 *
 * Creates the BLE send and receive threads.
 */
void init_ble_comm()
{
    k_tid_t err;
    LOG_INF("Initializing BLE communication");


}


void set_state_biogap(int8_t state)
{
  biowolf_current_state = state;
}

int8_t get_state_biogap(void)
{
  return biowolf_current_state;
}

void SendHardwareVersion() 
{

    uint8_t hardware_version_data[4];
    hardware_version_data[0] = REQUEST_HARDWARE_VERSION;
    hardware_version_data[1] = HARDWARE_VERSION;
    hardware_version_data[2] = HARDWARE_REVISION;
    hardware_version_data[3] = BLE_PCK_TAILER;

    return(send_data_ble(&hardware_version_data, sizeof(hardware_version_data)));  
}

void SendFirmwareVersion() 
{
    uint8_t firmware_version_data[4];
    firmware_version_data[0] = REQUEST_FIRMWARE_VERSION;
    firmware_version_data[1] = FIRMWARE_VERSION;
    firmware_version_data[2] = FIRMWARE_REVISION;
    firmware_version_data[3] = BLE_PCK_TAILER;

    return(send_data_ble( &firmware_version_data, sizeof(firmware_version_data)));   
}

void SendAvailableSensors() 
{
    uint8_t available_sensors[4];
    available_sensors[0] = REQUEST_AVAILABLE_SENSORS;
    available_sensors[1] = true;
    available_sensors[2] = 0;
    available_sensors[3] = BLE_PCK_TAILER;

    return(send_data_ble(&available_sensors, sizeof(available_sensors)));  
}

void SendDeviceSettings() {}


void SendReady_BLE()
{
    uint32_t ret_val;
    uint8_t ready[5]={'B','W','F','1','6'};
                 
    return(send_data_ble(&ready[0], 5));
     
}


K_THREAD_DEFINE(ble_send_tid, BLE_SEND_STACK_SIZE, ble_send_thread, NULL, NULL, NULL, BLE_SEND_PRIORITY, 0, 0);

K_THREAD_DEFINE(ble_receive_tid, BLE_RECEIVE_STACK_SIZE, process_received_data_thread, NULL, NULL, NULL, BLE_RECEIVE_PRIORITY, 0, 0);

