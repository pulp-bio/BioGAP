Ongoing temporary document to note step-by step changes done to enable WiFi integration.

Key decision:

a) in the original implementation, the start of acquisition from the different sensors 8EMG - EEG - MIC - IMU) and relative streaming is orchestrated by the BLE thread (specifically, handle_ble_command in the BLE application)

Flow: establish BLE communication --> wait for commands from the GUI --> start collecting and sending

ble/ble_appl.c
└─ handle_ble_command(uint8_t cmd)
   ├─ case START_EEG_STREAMING: eeg_start_streaming(); break;
   ├─ case START_EMG_STREAMING: emg_start_streaming(); break;
   ├─ case START_MIC_STREAMING: mic_start_streaming(); break;
   ├─ case START_EEG_MIC_STREAMING: sync_begin(2); mic_start_streaming(); eeg_start_streaming(); break;
   ├─ case START_STREAMING_ALL: sync_begin(3); mic_start_streaming(); eeg_start_streaming(); imu_start_streaming(); break;
   ├─ case START_IMU_STREAMING: imu_start_streaming(); break;
   ├─ case STOP_EEG_STREAMING: eeg_stop_streaming(); break;
   └─ ... (all STOP cases)


Refactored architecture 

core/command_dispatcher.c
└─ handle_streaming_command(uint8_t cmd)  ← move HERE (all the switch/case logic)
   ├─ case START_EEG_STREAMING: eeg_start_streaming(); break;
   ├─ case START_EMG_STREAMING: emg_start_streaming(); break;
   ├─ case START_MIC_STREAMING: mic_start_streaming(); break;
   └─ ... (entire switch/case block)

ble/ble_appl.c
└─ handle_ble_command(uint8_t cmd)  ← Becomes thin wrapper
   └─ handle_streaming_command(cmd);

wifi/wifi_appl.c  ← New transport layer
└─ handle_wifi_command(uint8_t cmd)  ← Also thin wrapper
   └─ handle_streaming_command(cmd);



Changes in progress:
a) Extract handle_ble_command() logic into command_dispatcher.c (the big switch/case)

b) Keep transport-specific code in BLE/WiFi folders:
    - send_data_ble() stays in ble
    - send_data_wifi() goes in new wifi/
    - Both call shared dispatcher

c) Watch for dependencies:
    - ble_reset_packet_counters() and ble_print_packet_stats() may need to be generic or transport-agnostic

d) ADS functions are made flexible to handle either BLE or Wi-Fi (src_NRF\afe\ads_spi_data.c). In particular:
   void ads_spim_handler_done(void);

   --> now this funciton will add data either to the BLE or to the Wi-Fi shield buffer
   --> the ble_tx_buffer is now kept as his. Instead of sending via BLE, data are send to ESP, which will then create the packet with the correct MTU for WiFi.
   Main change is in the add_data_to_send_buffer();


Header files:
Create command_dis
patcher.h with function prototype
Include it in both ble_appl.c and future wifi_appl.c