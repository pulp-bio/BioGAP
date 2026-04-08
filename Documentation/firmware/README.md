# Overview

This firmware runs on the Nordic Semiconductor nRF5340 dual-core SoC and provides multi-modal biosignal acquisition with Bluetooth Low Energy (BLE) streaming. The system supports synchronized acquisition from multiple sensors:

- **EEG/EMG**: 16 channels via dual ADS1298 AFE (500 Hz)
- **IMU**: LIS2DUXS12 3-axis accelerometer (400 Hz)
- **PPG**: MAX86150 sensor
- **Microphone**: PDM digital microphone (16 kHz)

> [!CAUTION]
> The PPG sensor has currently NOT being tested

## Getting Started

To get started with the firmware, please refer to the [Getting Started Guide](./getting_started.md).

## Authors

- Philipp Schilk (schilkp@ethz.ch), ETH Zurich
- Philip Wiese (wiesep@iis.ee.ethz.ch), ETH Zurich
- Sebastian Frey (sefrey@iis.ee.ethz.ch), ETH Zurich
- Giovanni Pollo (giovanni.pollo@polito.it), Politecnico di Torino