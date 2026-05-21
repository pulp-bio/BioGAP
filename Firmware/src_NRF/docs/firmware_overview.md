# Firmware Overview

This document summarizes the startup path and runtime architecture of the firmware in `main.c`.

## High-level purpose

The application boots the board, initializes power and USB, powers the GAP9 side, starts BLE advertising, and brings up the sensor subsystems used for biosignal streaming:

- ADS1298 EEG front end over SPI
- PDM microphone
- IMU
- EEG or EMG, depending on build-time configuration
- Inter-board hardware synchronization

The main thread does not perform continuous acquisition itself. After initialization, the work is handled by peripheral interrupts and dedicated subsystem threads.

## SPI hardware

The ADS1298 front end uses Nordic SPIM instance 2:

- Controller: SPIM2
- SCK: P0.08
- MOSI: P0.09
- MISO: P0.10
- Transfer width: 8-bit words
- Bit order: MSB first
- Clock mode: SPI Mode 1
- Frequency: 4 MHz

This is configured in [ads_defs.h](../afe/ads_defs.h#L97) and initialized in [ads_spi_hw.c](../afe/ads_spi_hw.c#L175).

## Component block diagram

```
                              +----------------+
                              |     main.c     |
                              +----------------+
                                      |
        +-----------------------------+------------------------------+
        |                             |                              |
        v                             v                              v
+------------------+      +--------------------+          +------------------+
| Power and PMIC   |      | Communication      |          | Sensor startup   |
| - pwr_init       |      | - usb_enable      |           | - mic_init       |
| - pwr_bsp_start  |      | - BLE adverts      |          | - imu_init       |
| - pwr_charge_en. |      | - BLE application  |          | - eeg_init       |
| - GAP9 rail      |      +--------------------+          | - emg_init       |
| - ship mode reset|                                         +------------------+
+------------------+
        |
        v
+------------------+
| MAX77654 PMIC    |
+------------------+

                              +------------------------+
                              | Analog Front End (AFE) |
                              | - init_spi             |
                              | - ADS1298 dual device  |
                              | - ads_dr_init          |
                              | - process_ads_data     |
                              +------------------------+

                              +------------------------+
                              | Board Sync             |
                              | - board_sync_init      |
                              | - GPIO sync line       |
                              +------------------------+
```

## Data flow diagram 

```
   ADS1298 EEG / EMG data   Microphone           IMU
           |                    |                     |
           v                    v                     v
   +---------------+    +---------------+     +---------------+
   |   ADS SPI     |    |   Mic app     |     |   IMU app     |
   |  layer / SPI  |    |  packetizer   |     |  packetizer   |
   +---------------+    +---------------+     +---------------+
           |                    |                     |
           +---------+----------+----------+----------+
                     |                     |
                     v                     v
                +--------------------------------+
                |        BLE send buffer         |
                +--------------------------------+
                               |
                               v
                    +-----------------------+
                    | BLE stack / adverts   |
                    +-----------------------+
                               |
                               v
                    +-----------------------+
                    | BLE central / host    |
                    +-----------------------+

   EEG/EMG control ----> ADS SPI / ADS1298 configuration

   board_sync ----> start / timing markers ----> EEG app
   board_sync ----> start / timing markers ----> IMU app

   USB CDC ACM ----> enumeration / control path ----> main
   PMIC / power ----> power enable / reset ---------> main
```

## Thread initialization and lifecycle

The firmware uses `K_THREAD_DEFINE()` to create threads with non-negative priorities. **These threads are created and automatically started when the kernel boots** (not deferred). Key thread definitions:

| Thread | File | Definition | Stack | Priority | Lifecycle |
|--------|------|------------|-------:|---------:|-----------|
| IMU streaming | [sensors/imu/imu_appl.c](../sensors/imu/imu_appl.c#L250) | `K_THREAD_DEFINE(imu_thread_id, ...)` | 2048 | 6 | Starts at boot; blocks on `imu_start_sem`; runs when woken by `imu_start_streaming()` |
| EEG streaming | [sensors/eeg/eeg_appl.c](../sensors/eeg/eeg_appl.c#L105) | `K_THREAD_DEFINE(eeg_thread_id, ...)` | 2048 | 5 | Starts at boot; blocks on `eeg_start_sem`; runs when woken by `eeg_start_streaming()` |
| EMG streaming | [sensors/emg/emg_appl.c](../sensors/emg/emg_appl.c#L105) | `K_THREAD_DEFINE(emg_thread_id, ...)` | 2048 | 5 | Starts at boot; blocks on `emg_start_sem`; runs when woken by `emg_start_streaming()` |
| Microphone streaming | [sensors/mic/mic_appl.c](../sensors/mic/mic_appl.c#L232) | `K_THREAD_DEFINE(mic_thread_id, ...)` | 2048 | 6 | Starts at boot; blocks on `mic_start_sem`; runs when woken by `mic_start_streaming()` |
| BLE send | [ble/ble_appl.c](../ble/ble_appl.c#L372) | `K_THREAD_DEFINE(ble_send_tid, ...)` | 2048 | 5 | Starts at boot; always running; waits on `send_msgq` for packets to transmit |
| BLE receive / command processing | [ble/ble_appl.c](../ble/ble_appl.c#L373) | `K_THREAD_DEFINE(ble_receive_tid, ...)` | 2048 | 4 | Starts at boot; always running; waits on `ble_data_received` semaphore for incoming BLE data |

**Thread startup sequence:**
1. **Kernel boot** → All threads created and started immediately
2. **Streaming threads enter wait loop** → Each calls `k_sem_take(&*_start_sem, K_FOREVER)` and blocks, suspending execution
3. **BLE command received** → User sends start command from BIOGUI
4. **BLE receive thread processes command** → Calls `handle_ble_command()` which invokes `*_start_streaming()`
5. **Start function wakes thread** → `*_start_streaming()` performs power-on and device init, then calls `k_sem_give(&*_start_sem)` to wake the waiting thread
6. **Streaming thread resumes** → The thread wakes from semaphore, passes any sync barriers, and enters its acquisition loop

All streaming threads coordinate via `core/sync_streaming.h` when synchronized starts are requested (e.g., `START_EEG_MIC_STREAMING`).

## Streaming triggers and ongoing execution

Streaming is initiated by BLE commands in the current implementation. The WiFi path is planned as a later transport layer and is still represented by placeholders in the dispatcher.

**Streaming initiation (BLE commands):**

The BLE receive thread's command handler (`handle_ble_command()` in [ble/ble_appl.c](ble/ble_appl.c#L119)) is the active source of streaming start commands right now. When the board is in `STATE_STREAMING_NORDIC` mode:

- `START_EEG_STREAMING` → calls `eeg_start_streaming()` ([ble/ble_appl.c](ble/ble_appl.c#L207-L211)).
- `START_EMG_STREAMING` → calls `emg_start_streaming()` ([ble/ble_appl.c](ble/ble_appl.c#L219-L223)).
- `START_MIC_STREAMING` → calls `mic_start_streaming()` ([ble/ble_appl.c](ble/ble_appl.c#L231-L233)).
- `START_EEG_MIC_STREAMING` → starts mic then eeg with `sync_begin()` for synchronized start ([ble/ble_appl.c](ble/ble_appl.c#L241-L246)).
- `START_STREAMING_ALL` → starts mic, eeg, and imu with `sync_begin(3)` ([ble/ble_appl.c](ble/ble_appl.c#L255-L261)).
- `START_IMU_STREAMING` → calls `imu_start_streaming()` ([ble/ble_appl.c](ble/ble_appl.c#L271-L273)).

**Execution model:**

Each `*_start_streaming()` call:
1. Validates current state
2. Turns on power to the sensor hardware
3. Initializes the sensor (ADS1298, etc.)
4. Signals the waiting thread with `k_sem_give(&*_start_sem)` to wake it up

Once the thread wakes:
- **EEG/EMG threads** enter `eeg_streaming_thread()` or `emg_streaming_thread()` and run in a loop driven by ADS1298 data-ready interrupts (from `ads_drdy_callback()` in [afe/ads_spi_hw.c](../afe/ads_spi_hw.c#L339)).
- **Microphone thread** enters `mic_streaming_thread()` and runs driven by PDM data callbacks.
- **IMU thread** enters `imu_streaming_thread()` and runs driven by LIS2DUXS12 data-ready interrupts (from `lis2duxs12_irq_callback()` in [sensors/imu/lis2duxs12_sensor.c](../sensors/imu/lis2duxs12_sensor.c#L97)).

**Important:** BLE does not continuously "drive" data acquisition; it only initiates the sequence. After the start command, the threads are entirely driven by their respective hardware interrupt callbacks, which call `process_ads_data()`, `process_mic_data()`, and `process_imu_data()` respectively.

## Current connectivity split

At the moment, the connectivity control path is BLE-only:

- `ble/ble_appl.c` receives commands and forwards them to the dispatcher.
- `core/command_dispatcher.c` contains the shared command logic.
- WiFi-specific send/receive hooks are still placeholders and can be filled in later when the WiFi shield is integrated.

## Startup sequence in `main()`

1. Log startup messages and initialize logging.
2. Initialize the power subsystem with `pwr_init()` and `pwr_bsp_start()`.
3. Verify that the USB CDC ACM device is ready with `device_is_ready(uart_dev)`.
4. Enable USB with `usb_enable(NULL)`.
5. Enable charging and initialize the ADS data-ready interrupt path with `ads_dr_init()`.
6. Initialize the SPI bus with `init_spi()`.
7. Power the GAP9 rail with `gap9_pwr(true)`.
8. Start BLE advertising with `start_bluetooth_adverts()`.
9. Initialize optional streaming subsystems:
   - microphone via `mic_init()`
   - IMU via `imu_init()`
   - EEG via `eeg_init()` when `CONFIG_SENSOR_EEG` is enabled and `CONFIG_SENSOR_EMG` is disabled
   - EMG via `emg_init()` when `CONFIG_SENSOR_EMG` is enabled and `CONFIG_SENSOR_EEG` is disabled
10. Initialize board synchronization with `board_sync_init()`.
11. Enter the main sleep loop and only react to the soft-reset flag.

## Runtime behavior

After initialization, `main()` mostly stays idle:

- `k_msleep(1000)` keeps the thread dormant.
- The actual sensor acquisition and BLE traffic are handled by subsystem code and interrupts.
- If `flag_isr_soft_reset` is set, the firmware waits briefly, places the PMIC into ship mode with `max77654_factory_ship_mode(&pmic_h)`, clears the flag, and continues.

## Notes

- The main control path is split across power, USB, SPI/AFE, sensor application layers, BLE, and board synchronization.
- EEG and EMG are mutually exclusive at build time in this entry point.
- The documentation above reflects the control flow visible in [main.c](../main.c#L79) and the subsystem interfaces in:
  - [pwr_bsp.h](../bsp/pwr_bsp.h)
  - [board_sync.h](../core/board_sync.h)
  - [ble_appl.h](../ble/ble_appl.h)
  - [mic_appl.h](../sensors/mic/mic_appl.h)
  - [imu_appl.h](../sensors/imu/imu_appl.h)
  - [eeg_appl.h](../sensors/eeg/eeg_appl.h)
  - [ads_spi.h](../afe/ads_spi.h)


## What to change before flashing

You need to update the [prj.conf](../prj.conf) file to enable your desired shields. Specifically:

### Biopotential Shields

CONFIG_SENSOR_EMG --> if the mainboard is connected to the EMG shield
CONFIG_SENSOR_EEG --> if the mainboard is connected to the EEG shield

### Connectivity and extra memory Shield
CONFIG_SD_WIFI_SHIELD --> if the mainboard is connected to the SD Card / Wi-Fi shield.

Note: the use of WiFi or BLE can be tuned at run-time without changing the settings.

## Add your own shield 
If you want to add your own shield:

a) Add your config and overlay files inside the [custom_shields](firmware/custom_shields) folder and copy them to the Sensei SDK
b) Add your dts (if needed) inside the [custom_shields](firmware/custom_dts) folder and copy them to the Sensei SDK
c) Add your source code under the [sensors][]
d) Modify the [CMakeLists.txt](../CMakeLists.txt) to enable the shield and the target source. Specifically:


```bash
set(SHIELD "SENSEI_ExGShield SENSEI_PPGShield SENSEI_YourShield")
```
```bash
target_sources(app PRIVATE

# Your C Files

)
```
