#ifndef SHIELD_CONFIG_H
#define SHIELD_CONFIG_H


// ---------------------- AP State Machine ---------------------
#define IS_WBAN  0


// ---------------------- SELECT IF ESP is SPI MASTER OR SLAVE ---------------------
#define IS_ESP_SPI_SLAVE  1

// ----------------------- ESP direct write to SD card -----------------------
#define ESP_ENABLE_SD_WRITE          0

// ----------------------- ESP-only dummy sensor generator -----------------------
// When set, the ESP32 generates synthetic dummy-sensor packets locally (identical
// wire format to Firmware/src_NRF/sensors/dummy_sensor/dummy_sensor_appl.c) and
// streams them straight to BioGUI, bypassing SPI/BIOGAP entirely. For testing the
// WiFi/GUI half of the system in isolation, with no nRF/SPI hardware attached.
#define ESP_LOCAL_DUMMY_SENSOR       0

#endif // SHIELD_CONFIG_H