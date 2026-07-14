#ifndef PIN_DEFINITIONS_H
#define PIN_DEFINITIONS_H

// -------------------- NRF <-> ESP SPI pins. Condition: NRF is SPI Master, ESP is SPI slave --------------------
#define NRF_ESP_DRDY_DIR_CTRL        12      // TO determine if NRF or ESP is the SPI master. LOW if NRF is master, HIGH if ESP is master.
#define NRF_ESP_MOSI            23      // NRF_ESP MOSI  -> SPI_NRF_MOSI --> SPI_A_MOSI
#define NRF_ESP_MISO            19      // NRF_ESP MISO  --> SPI_NRF_MISO --> SPI_A_MISO
#define NRF_ESP_SCLK            20      // NRF_ESP SCLK  -> SPI_NRF_CLK --> SPI_A_CLK
#define NRF_ESP_CS              21      // NRF_ESP_CS    -> NRF_QSPI_IO1
#define NRF_ESP_DATA_READY      22      // NRF_ESP_DRDY  -> NRF_QSPI_IO2


// -------------------- SD Card SPI pins(if ESP writes to SD Card) ------------------------
// Can be used only when ESP is SPI master and SD card is SPI slave.
// Must switch SPI bus at run-time. It will be updated later.
#define ESP_SD_MOSI  7                                  // ESP MOSI: writes data to SD Card | SD CARD: mSD_CMD ->CMD 
#define ESP_SD_MISO  2                                 // ESP MISO: received data from SD Card | SD CARD: mSD_DAT0 -> DAT0
#define ESP_SD_CLK   6                                // ESP CLK: clock singal | SD CARD: mSD_CLK -> CLK
#define ESP_SD_CS    18                                // ESP CS: chip select signal | SD CARD mSD_DAT3 -> CS (in spi mode)  
#define ESP_SDCTRL      9                              // To control if the ESP writes directly to SD card. 

// -------------------- RTC PINS  ------------------------
#define RTC_SCL 15
#define RTC_SDA 14
#define RTC_ON  13                                  // GPIO pin for enabling/disabling the RTC
#define RTC_FOUT    4
#define RTC_CLKOUT 3

#
#endif // PIN_DEFINITIONS_H