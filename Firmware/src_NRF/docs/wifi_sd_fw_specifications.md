# Wi-FI SD card shield

By defualt, SPI-A is used.
The user can decide to use SPI-B; by default, SPI-B is reserved for the WULPUS pro shield (US data).

PIN Mappings (old Wi-Fi shiled)


The Wi-Fi / SD card chips uses the QSPI pins to establish communication between ESP (CS and DRDY) and the SD card (CS).
IMPORTANT (for new Desing)
on the [hardware design](https://github.com/pulp-bio/sensei-base-board/blob/86c699ea255fa07d6760d5206c0c67f08c0a97ee/Documentation/SENSEI_Base-Board_Schematics.PDF) the QSPI pins are connected to the flash memory on the Mainboard. In order to use the Wi-Fi shiled, the flash memory must be disconnected.






