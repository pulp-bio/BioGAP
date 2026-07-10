# BioGAP NRF Firmware Overview

This document describes the architecture of the nRF5340 firmware in `Firmware/src_NRF`. It targets the `nrf5340_senseiv1_cpuapp` board defined in the companion [sensei-sdk](https://github.com/pulp-bio/sensei-sdk) repository, built as a Zephyr/nRF Connect SDK application.

## 1. Purpose

The nRF5340 is the central controller of the BioGAP wearable: it drives the ADS1298 analog front-end(s) for ExG (EEG/EMG) acquisition, talks to a set of optional sensor shields (PPG, IMU, microphone, WULPUS ultrasound), and streams the resulting data to a host application over one of two mutually-exclusive transports:

- **BLE** (Nordic UART Service) — the default.
- **WiFi/SD shield** (an ESP32-C6 acting as SPI slave to the nRF) — selected at build time via `CONFIG_WI_FI`.

Both transports funnel *incoming* commands through a single transport-agnostic dispatcher, so the sensor/streaming logic itself doesn't need to know which transport is active.

## 2. Directory Layout

```
src_NRF/
├── main.c                    Entry point: power-up, SPI/GPIO init, transport selection, sensor init
├── Kconfig                    Feature toggles (sensors, transport, diagnostics, multi-board sync)
├── CMakeLists.txt              Build config: shields, sources, conditional compilation
├── prj.conf                    Zephyr/NCS project configuration (peripherals, BLE stack, logging, ...)
│
├── afe/                       ADS1298 analog front-end driver (ExG: EEG/EMG)
├── ble/                       BLE stack (NUS) + BLE-side application glue
├── bsp/                       Board support: power thread, system status, I2C mux, diagnostics
├── core/                      Cross-cutting: command dispatcher, sync primitives, shared definitions
├── sensors/                   One subfolder per sensor modality
│   ├── eeg/  emg/              Built on top of afe/ (ADS1298)
│   ├── imu/                    LSM6DSV16BX accelerometer + gyroscope
│   ├── mic/                    PDM microphone
│   ├── ppg_new/                MAXM86161 PPG (1-8x via TCA9548A I2C mux)
│   ├── wulpus/                 WULPUS ultrasound probe (MSP430 bridge)
│   └── dummy_sensor/            Synthetic data generator, no hardware required
├── spi/                       Shared SPI_A bus (SPIM4) — see §4
└── wifi_sd_shield/             ESP32-C6 WiFi/SD shield transport
```

## 3. Boot Sequence (`main.c`)

`main()` runs the following steps in order:

1. **Power**: `pwr_init()` → `pwr_charge_enable()` → `pwr_start()` (SDK power thread; refreshes battery/charger telemetry, gated off from PMIC I2C reads while the ADS streams — see §6).
2. **SPI_A + ADS1298**: `ads_dr_init()` (DRDY GPIO/interrupt) and `init_spi()` (brings up the shared SPI_A bus via `init_spi_a_bus()`, then the ADS1298 CS/START GPIOs — see §4).
3. **Transport** (mutually exclusive, `CONFIG_WI_FI`):
   - **WiFi/SD shield**: `wifi_sd_shield_cs_init()` (CS/DRDY/QSPI-mux GPIOs on the shared SPI_A bus) then `initial_handshake_nrf_esp()`.
   - **BLE** (default): `start_bluetooth_adverts()`.
4. **Dummy sensor** (`CONFIG_DUMMY_SENSOR`, optional): `dummy_sensor_init()`.
5. **Microphone**, **IMU**: `mic_init()` / `imu_init()`.
6. **EEG / EMG** (`CONFIG_SENSOR_EEG` / `CONFIG_SENSOR_EMG`, can both be built into one image — the mode is selected at runtime by the GUI start command; simultaneous EEG+EMG streaming is rejected at runtime since they share the ADS1298 pair).
7. **PPG** (`CONFIG_SENSOR_PPG_NEW`), **WULPUS** (`CONFIG_SENSOR_WULPUS`, default-enabled).
8. **Board sync** (currently disabled in `main.c`; see §8).
9. Idle loop: sleeps, and services `flag_isr_soft_reset` (long-press → PMIC factory-ship mode via `max77654_factory_ship_mode()`).

Everything after step 3 runs from dedicated Zephyr threads (`K_THREAD_DEFINE` per module) — `main()` itself does not block on streaming.

## 4. SPI_A: the Shared High-Speed Bus (`spi/`)

`spi/spi_a.c` / `spi_a.h` own the one nrfx SPIM peripheral (**SPIM4**, pins SCK=P0.08/MOSI=P0.09/MISO=P0.10) that is physically shared by every device on **SPI_A**: the ADS1298 pair (ExG shield) and the ESP32-C6 / on-board SD path (WiFi/SD shield). There is no hardware-managed chip select — every device gets its own software-toggled CS GPIO, since more than one device shares the bus.

Key pieces:
- `init_spi_a_bus()` — idempotent bring-up (nrfx init + IRQ). Safe to call from both `afe/ads_spi_hw.c`'s `init_spi()` and `wifi_sd_shield/wifi_sd_shield_inits.c`'s `wifi_sd_shield_cs_init()`, in either order.
- `spi_a_begin_transfer(owner, cs)` — since nrfx allows only **one** registered completion handler per peripheral instance, this records which consumer (`SPI_A_OWNER_ADS` / `SPI_A_OWNER_WIFI_SD`) issued the in-flight transfer, so the single shared interrupt handler can route completion to the right place (`ads_spim_transfer_complete()` or `wifi_sd_spim_transfer_complete()`).
- `spi_a_mutex` — serializes bus access across consumers. The ADS driver releases it immediately after kicking off its async transfer (matching its existing, hardware-validated timing); the WiFi/SD path holds it through completion (it already blocks on a semaphore).

`SPI_B` (SPIM2, SCK=P1.07/MOSI=P0.30/MISO=P0.29) is a separate bus reserved for the WULPUS shield (MSP430 bridge) and is owned directly by `sensors/wulpus/wulpus_appl.c`.

## 5. ADS1298 / ExG (`afe/`)

- `ads_spi_hw.c` — GPIO (CS ×2, START, DRDY) and SPI_A bring-up.
- `ads_spi_comm.c` — low-level read/write transactions (`ads1298_read_spi`, `ads1298_read_samples`, `ads1298_write_spi`), each asserting the correct device's CS and registering itself as `SPI_A_OWNER_ADS`.
- `ads_spi_config.c` — device configuration/register programming, ID check (`ads_check_id`).
- `ads_spi_data.c` — DRDY-interrupt-driven acquisition (`ads_spim_handler_done`, `process_ads_data`): reads both ADS1298_A/B, assembles the 211-byte ExG packet (header, counter, timestamp, 4 samples × 50 bytes, board-sync metadata, trailer), and hands it to the active transport (`add_data_to_send_buffer` for BLE, `add_data_to_esp_send_buffer` for WiFi).
- `sensors/eeg/` and `sensors/emg/` are thin mode-selection layers on top of this shared ADS1298 pair (unipolar vs. bipolar supply rails).

## 6. Transport Layer

### 6.1 BLE (`ble/`)

- `bluetooth.c` — Zephyr BT stack, NUS init/advertising, `send_data_ble()`.
- `ble_appl.c` — two threads (disabled at compile time when `CONFIG_WI_FI` is set): `ble_send_thread` drains `send_msgq` → `send_data_ble()`; `process_received_data_thread` drains `nus_rx_msgq` and forwards each command to `handle_connectivity_command()` (§7). Also exposes the transport-agnostic `add_data_to_send_buffer()` sensors call to enqueue outgoing data.

### 6.2 WiFi/SD shield (`wifi_sd_shield/`)

The nRF acts as SPI master to an ESP32-C6 SPI slave on the shared SPI_A bus (§4), using its own software-toggled CS (`gpio_nrf_esp_cs`). A second CS (`gpio_nrf_sd_cs`) is reserved for the nRF to write to the SD card directly, bypassing the ESP32 (not yet implemented). Two QSPI-mux GPIOs (`gpio_nrf_qspi_cs`, `_sel`) are parked at init so the on-board QSPI PSRAM (unused on this bus) stays deselected.

- `wifi_sd_shield_inits.c` — CS/DRDY GPIO setup + shared SPI_A bring-up (`wifi_sd_shield_cs_init()`).
- `wifi_sd_spi_functions.c` — `spi_master_transceive()` (generic transaction primitive, `SPI_A_OWNER_WIFI_SD`), `biogap_to_esp_transaction()` (sends a sensor packet, checks for an embedded stop command from the ESP).
- `wifi_sd_shield_appl.c` — sender/receiver threads (`spi_nrf_esp_sender_thread`, `spi_nrf_esp_receiver_thread`), DRDY-driven `process_esp_data()`, `add_data_to_esp_send_buffer()`, and `initial_handshake_nrf_esp()`.

> **Known gap**: the shield schematic's bus-direction/level-translator control lines (`NRF_ESP_DIR_CTRL`, `NRF_ESP_DRDY_DIR_CTRL`) aren't yet modeled in the devicetree overlay or initialized in code — needed before this shield works on real hardware.

### 6.3 Command Dispatcher (`core/command_dispatcher.c`)

`handle_connectivity_command(const uint8_t *data, uint16_t size)` is the single entry point both transports route incoming commands through (`data[0]` is the opcode; `data[1..]` is any command-specific payload). It dispatches to: battery/system status requests, device/board state, streaming start/stop for every sensor (EEG, EMG, IMU, MIC, PPG, WULPUS, dummy), and combined start/stop (e.g. `START_STREAMING_ALL`) with `core/sync_streaming.c` barriers so multiple sensors begin sampling together.

Commands that need payload bytes beyond the opcode (PPG_NEW config, WULPUS MSP430 config) gracefully fall back to defaults when the caller can only supply the opcode itself — relevant for the WiFi shield's fixed-size 4-byte command transaction, which has little room for a payload.

Opcodes live in `core/connectivity_commands.h` (transport-agnostic superset of the legacy `ble/ble_commands.h`).

## 7. Sensor Modules (`sensors/`)

| Module | Hardware | Notes |
|---|---|---|
| `eeg/`, `emg/` | ADS1298 (via `afe/`) | Mutually exclusive at runtime, share the ADS1298 pair |
| `imu/` | LSM6DSV16BX (accel + gyro) | Always initialized in `main.c` |
| `mic/` | PDM microphone | Timestamped packets (harmonized first-sample convention, matching ExG) |
| `ppg_new/` | 1-8× MAXM86161 via TCA9548A I2C mux | `CONFIG_SENSOR_PPG_NEW`; config parsed from the start-streaming payload |
| `wulpus/` | MSP430 ultrasound bridge, on SPI_B | `CONFIG_SENSOR_WULPUS` (default-enabled); BLE-only rich config, forwards raw MSP430 config bytes |
| `dummy_sensor/` | None | `CONFIG_DUMMY_SENSOR`; generates synthetic ExG-shaped packets on a timer, routes through the same BLE/WiFi send path as a real sensor — useful for pipeline testing without hardware |

## 8. BSP and Cross-Cutting (`bsp/`, `core/`)

- `bsp/pwr_bsp.c`, `bsp/power/` — SDK power-thread integration (battery/charger telemetry cache, gated off from PMIC I2C reads while ADS streams to avoid noise injection).
- `bsp/system_status/system_status.c` — battery/status/version/board-state packet builders and senders (transport-branching via `CONFIG_WI_FI`).
- `bsp/i2c_mux/` — TCA9548A I2C mux driver (PPG).
- `bsp/pmic_noise_test.c` — diagnostic sweep to correlate PMIC activity with ExG noise bursts (`CONFIG_PMIC_NOISE_TEST`).
- `core/sync_streaming.c` — barrier (`sync_begin(n)`/`sync_reset()`) for simultaneous multi-sensor start.
- `core/board_sync.c` — multi-board hardware sync (primary/secondary GPIO pulse roles), currently disabled in `main.c`.
- `core/i2c_helpers.c` — generic I2C helper routines.
- `core/common.h` — shared board-state constants, firmware/hardware version macros.

## 9. Build Configuration

- **`CMakeLists.txt`**: selects shields (`SENSEI_ExGShield`, `SENSEI_PPGShield`, `SENSEI_WiFi_SD_Shield_V1` — all defined in the sensei-sdk repo, not in this one), and conditionally compiles WULPUS, PMIC noise test, PPG_NEW, WiFi/SD shield + dispatcher, and dummy sensor sources based on their Kconfig symbols.
- **`Kconfig`**: `SENSOR_EEG`, `SENSOR_EMG`, `SENSOR_WULPUS`, `SENSOR_PPG_NEW` (+ sub-options), `WI_FI`, `DUMMY_SENSOR`, `PMIC_NOISE_TEST`, plus Multi-Board Sync and State Machine menus.
- **`prj.conf`**: default `y`/`n` values for the above, BLE stack config, SPI instance enables (`CONFIG_NRFX_SPIM4`, `CONFIG_NRFX_SPIM2`), logging/debug backends, MCUboot.

Shield devicetree overlays (GPIO/pin definitions) live in the [sensei-sdk](https://github.com/pulp-bio/sensei-sdk) repo under `NRF/boards/shields/`, not in this repository — see the top-level `Documentation/firmware/getting_started.md` for the two-repo build setup.
