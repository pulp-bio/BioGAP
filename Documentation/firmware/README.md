# Firmware Documentation

This firmware runs on the **Nordic Semiconductor nRF5340** dual-core SoC and provides multi-modal biosignal acquisition with Bluetooth Low Energy (BLE) streaming. It is built on the **Zephyr RTOS** using the **nRF Connect SDK** and the custom **SENSEI SDK** board support package.

## Supported Sensors

| Sensor | Device | Channels | Sample Rate | Module |
|--------|--------|----------|-------------|--------|
| EEG | 2x ADS1298 | 16 | 500 Hz | `sensors/eeg` |
| EMG | 2x ADS1298 | 16 | 500 Hz | `sensors/emg` |
| IMU | LIS2DUXS12 | 3-axis accel | 400 Hz | `sensors/imu` |
| Microphone | PDM digital mic | 1 (mono) | 16 kHz | `sensors/mic` |
| PPG | MAX86150 | 2 (Red/IR) | ~100 Hz | `sensors/ppg` |

> **Note**: The PPG sensor has not been tested yet. PPG data, when active, is multiplexed into the ExG (i.e. EEG/EMG) BLE packet by replacing the last two ADS1298 channels.

## Key Features

- Multi-threaded sensor acquisition with dedicated threads per sensor
- Barrier-synchronized simultaneous start across all active sensors
- BLE streaming via Nordic UART Service (NUS) at 2M PHY, 251-byte MTU
- Inter-board GPIO synchronization for multi-device setups (Not tested)
- PMIC-based power management with battery monitoring

## Documentation Index

### Getting Started
- [Getting Started Guide](./getting_started.md) - Build, flash, and run the firmware

### Architecture
- [Architecture Overview](./architecture.md) - System architecture, threading model, and data flow

### Modules
- [AFE Module](./afe_module.md) - ADS1298 Analog Front-End driver (SPI, register configuration, data capture)
- [BLE Module](./ble_module.md) - BLE connectivity, NUS service, connection management, and streaming
- [BLE Protocol](./ble_protocol.md) - BLE command protocol specification (command codes and packet formats)
- [Sensor Modules](./sensor_modules.md) - EEG, EMG, IMU, Microphone, and PPG acquisition threads
- [BSP Module](./bsp_module.md) - Board support package (power management, battery, system status)
- [Core Module](./core_module.md) - Core utilities (synchronization, I2C helpers, board sync)

### Reference
- [Configuration](./configuration.md) - Build configuration, Kconfig options, device tree overlays
- [Data Formats](./data_formats.md) - BLE packet formats for all sensor types

## Source Code Layout

```
Firmware/
├── src_NRF/                                # Main firmware source
│   ├── main.c                              # Entry point, initialization sequence
│   ├── CMakeLists.txt                      # Build configuration
│   ├── prj.conf                            # Zephyr project configuration
│   ├── Kconfig                             # Custom Kconfig options
│   ├── nrf5340_senseiv1_cpuapp.overlay     # Device tree overlay
│   ├── pm_static.yml                       # Flash partition layout
│   ├── afe/                                # ADS1298 AFE driver
│   ├── ble/                                # BLE stack and application layer
│   ├── bsp/                                # Board support (power, battery)
│   ├── core/                               # Core utilities (sync, I2C)
│   ├── sensors/                            # Sensor acquisition modules
│   │   ├── eeg/                            # EEG streaming
│   │   ├── emg/                            # EMG streaming
│   │   ├── imu/                            # IMU (LIS2DUXS12) driver
│   │   ├── mic/                            # PDM microphone
│   │   └── ppg/                            # PPG (MAX86150) driver
│   └── child_image/                        # Network core and bootloader configs
├── custom_dts/                             # Custom device tree bindings for ADS1298
└── custom_shields/                         # Shield definitions for ExG and PPG boards
    ├── SENSEI_ExGShield/
    └── SENSEI_PPGShield/
```
