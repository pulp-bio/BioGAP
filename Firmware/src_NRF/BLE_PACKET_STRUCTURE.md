# BioGAP-Ultra BLE Packet Structure

This document describes the complete BLE packet protocol used by the BioGAP-Ultra
(nRF5340) firmware in `src_NRF`. It is the byte-level reference for the streaming
data packets the device sends, the command packets the host sends, and the
response packets the device sends back.

> **Source of truth.** All layouts below are taken directly from the firmware
> C source (`src_NRF/...`) and cross-checked against the BioGUI parsers
> (`bioguiForHGR/biogui/...`). Where an older code comment contradicts the
> actual behaviour, the **code** wins — see [Known discrepancies](#known-discrepancies).

## Conventions

- **Byte numbering is 0-based**: byte `0` is the first byte of the BLE payload.
  This matches the C array index (`ble_tx_buf[0]`) and the Python slice offsets
  in BioGUI. A range like `1-2` means bytes 1 and 2 inclusive.
- **Endianness is stated per field.** Counters, timestamps, MIC audio and WULPUS
  RF samples are **little-endian (LE)**. ExG (ADS1298) channel samples and IMU
  accelerometer samples are **big-endian (BE)**. Do not assume one convention
  for a whole packet.
- **`int24 BE`** = 24-bit two's-complement, most-significant byte first.
- All multi-sample sensor packets are **fixed-size**; unused tail bytes are
  zero padding.

---

## Table of contents

1. [Transport overview](#transport-overview)
2. [Packet dispatch table](#packet-dispatch-table)
3. Streaming data packets (device → host)
   - [3.1 ExG — EEG / EMG (ADS1298 × 2)](#31-exg--eeg--emg-ads1298--2)
   - [3.2 IMU (LSM6DSV16BX accelerometer + gyroscope)](#32-imu-lsm6dsv16bx-accelerometer--gyroscope)
   - [3.3 Microphone (PDM audio)](#33-microphone-pdm-audio)
   - [3.4 WULPUS PRO (ultrasound)](#34-wulpus-pro-ultrasound)
   - [3.5 PPG (MAXM86161)](#35-ppg-maxm86161)
4. [Command packets (host → device)](#4-command-packets-host--device)
5. [Response / status packets (device → host)](#5-response--status-packets-device--host)
6. [Scaling and units](#6-scaling-and-units)
7. [Known discrepancies](#known-discrepancies)
8. [Appendix: legacy formats](#appendix-legacy-formats)

---

## Transport overview

| Property | Value | Source |
|---|---|---|
| GATT service | Nordic UART Service (NUS) | `ble/bluetooth.c` (`bt_nus_init`) |
| Direction: device → host | NUS TX notifications (`bt_nus_send`) | `ble/bluetooth.c` (`send_data_ble`) |
| Direction: host → device | NUS RX writes | `ble/bluetooth.c` (`nus_cb`) |
| Connection interval | 7.5 ms | `ble/bluetooth.h` |
| PHY | 2M | `ble/bluetooth.h` |
| Data length / MTU | max DLE, ~244 B usable payload | `ble/bluetooth.h` |
| Max app TX packet | `BLE_PCKT_MAX_SIZE` = 244 | `ble/ble_appl.h` |
| RX buffer size | `BLE_PCKT_RECEIVE_SIZE` = 234 | `ble/ble_appl.h` |
| Device name | `CONFIG_BT_DEVICE_NAME` | `ble/bluetooth.c` |

The device advertises the NUS UUID. The host writes single-byte command codes
(optionally followed by a config payload) to the NUS RX characteristic; the
device streams data packets and status responses back as NUS notifications.

The host distinguishes packet types by the **first byte (header)** combined with
the **total length**. Several sensor packets deliberately share a 211-byte size
so a single stream can carry a mix (e.g. ExG + WULPUS) and be demuxed by header.

### Packet dispatch table

| Header byte | Trailer | Length (B) | Packet type | Direction |
|---|---|---|---|---|
| `0x55` | `0xAA` | 211 | ExG (EEG / EMG) data | device → host |
| `0x56` | `0x57` | 236 | IMU (accelerometer + gyroscope) data | device → host |
| `0xAA` | `0x55` | 136 | Microphone (PDM) data | device → host |
| `0x10`–`0x13` | (metadata tail) | 211 | WULPUS PRO ultrasound chunk (4 = 1 frame) | device → host |
| `0x70` | `0x71` | 211 | PPG data | device → host |
| `0xFA` (250) | — | 105 | WULPUS MSP430 config | host → device |
| `0xFB` (251) | — | 105 | WULPUS restart | host → device |
| 12–43 | — | 1–12 | Command code | host → device |
| 14/17/28/29 | `0xAA` | 4 / 7 | Status / version response | device → host |
| `0x2B` (43) | `0xAA` | 21 | Extended system-status response | device → host |

> ⚠️ The legacy battery response echoes `17` = `0x11` as its header, which
> collides with WULPUS chunk 2 (`0x10`–`0x13`). Do not poll command 17 while
> WULPUS is streaming — use command 43 (`0x2B`, collision-free) instead.

> Note the reuse of `0x55` and `0xAA`: ExG uses `0x55` header + `0xAA` trailer,
> while the MIC packet uses `0xAA` header + `0x55` trailer. Disambiguate by the
> total packet length (211 vs 136), not the header alone.

### Common streaming-packet prefix

All five streaming data packets (ExG, IMU, MIC, WULPUS, PPG) share the same
7-byte prefix:

| Byte | Field name | Data type |
|---|---|---|
| 0 | Header (packet-type byte) | uint8 |
| 1-2 | Packet counter | uint16 LE |
| 3-6 | Timestamp (µs) | uint32 LE |

Packet-type-specific payload follows from byte 7. For WULPUS the counter is a
per-frame value mirrored across the four BLE chunks; for the others it increments
once per BLE packet. All counters reset to 0 at the start of each streaming
session.

**Timestamp reference point.** The µs timestamp is the free-running Zephyr uptime
(`k_cyc_to_us_floor32(k_cycle_get_32())`) and, across all modalities, references
the **first sample of the packet**, so:

```
sample_i_time ≈ packet_ts + i / fs
```

How `packet_ts` is obtained per modality (the value always denotes the first
sample; the mechanism differs because of how each sensor delivers data):

| Modality | Mechanism |
|---|---|
| ExG (EEG/EMG) | captured directly at the first sample's DRDY |
| IMU | captured directly when the first sample is read |
| MIC | one DMA block = one packet, so block-arrival time is back-dated by (N−1)/fs |
| PPG | INTB (FIFO-batch-ready) time back-dated by (N−1)/fs |
| WULPUS | frame-acquisition time, captured in the SPI thread when `HOST_DATA_RDY` fires (not at BLE-forward time, so ring-buffer queueing does not skew it) |

For the block/FIFO sensors (MIC, PPG) the first-sample instant is not directly
observable, so it is estimated by back-dating from the batch-ready time assuming
uniform 1/fs spacing — the same assumption the `packet_ts + i/fs` formula makes.

---

## 3.1 ExG — EEG / EMG (ADS1298 × 2)

Two ADS1298 analog front-ends (**ADS_A**, **ADS_B**), 8 channels each = 16
channels total, 24-bit per channel. EEG and EMG packets are **byte-for-byte
identical**; they differ only in which start command was issued
(`START_EEG_STREAMING` vs `START_EMG_STREAMING`).

- **Header:** `0x55` (`BLE_PCK_HEADER`) · **Trailer:** `0xAA` (`BLE_PCK_TAILER`)
- **Total length:** 211 bytes (`EXG_PCK_LNGTH` / `EEG_PCKT_SIZE`)
- **Samples per packet:** 4 (`EXG_SAMPLES_PER_PACKET`)
- **Sample rate:** 500 Hz per channel → 125 packets/s (ADS1298 in High-Resolution mode, CONFIG1 = `0xC0` \| data-rate code 6, DR = 110)
- **Firmware:** `afe/ads_defs.h`, `afe/ads_spi_data.c` · **GUI:** `biogui/platforms/biogapultra/biogapultra_exg/interface_biogapultra_eeg.py`

### Overall layout

| Byte | Field name | Data type |
|---|---|---|
| 0 | Header (`0x55`) | uint8 |
| 1-2 | Packet counter | uint16 LE |
| 3-6 | Timestamp (µs) | uint32 LE |
| 7-56 | Sample 1 (50 bytes) | see per-sample block |
| 57-106 | Sample 2 (50 bytes) | see per-sample block |
| 107-156 | Sample 3 (50 bytes) | see per-sample block |
| 157-206 | Sample 4 (50 bytes) | see per-sample block |
| 207 | Metadata: board ID | uint8 |
| 208 | Metadata: sync pulse count | uint8 |
| 209 | Metadata: reserved | uint8 |
| 210 | Trailer (`0xAA`) | uint8 |

### Per-sample block (50 bytes, relative offsets within a sample)

| Rel. byte | Field name | Data type |
|---|---|---|
| 0-2 | ADS_A channel 1 | int24 BE |
| 3-5 | ADS_A channel 2 | int24 BE |
| 6-8 | ADS_A channel 3 | int24 BE |
| 9-11 | ADS_A channel 4 | int24 BE |
| 12-14 | ADS_A channel 5 | int24 BE |
| 15-17 | ADS_A channel 6 | int24 BE |
| 18-20 | ADS_A channel 7 | int24 BE |
| 21-23 | ADS_A channel 8 | int24 BE |
| 24-26 | ADS_B channel 1 | int24 BE |
| 27-29 | ADS_B channel 2 | int24 BE |
| 30-32 | ADS_B channel 3 | int24 BE |
| 33-35 | ADS_B channel 4 | int24 BE |
| 36-38 | ADS_B channel 5 | int24 BE |
| 39-41 | ADS_B channel 6 | int24 BE |
| 42-44 | ADS_B channel 7 | int24 BE |
| 45-47 | ADS_B channel 8 | int24 BE |
| 48 | `counter_extra` (DRDY tick counter, wraps at 255) | uint8 |
| 49 | Reserved (0x00) | uint8 |

Absolute offset of sample *s* (0-based) = `7 + s*50`. The `counter_extra` byte
is a free-running DRDY interrupt counter for timing analysis; the reserved byte
is a hook for custom per-sample data.

---

## 3.2 IMU (LSM6DSV16BX accelerometer + gyroscope)

6-axis IMU (3-axis accelerometer + 3-axis gyroscope), 480 Hz, 19 samples per
packet. Replaces the LIS2DUXS12 3-axis accelerometer of older mainboard
revisions (see [Appendix: legacy formats](#appendix-legacy-formats)).

- **Header:** `0x56` (`IMU_DATA_HEADER`) · **Trailer:** `0x57` (`IMU_DATA_TRAILER`)
- **Total length:** 236 bytes (`IMU_PCKT_SIZE` = 1+2+4+228+1)
- **Samples per packet:** 19 (`IMU_SAMPLES_PER_PACKET`) → ~25.3 packets/s
- **Sample rate:** 480 Hz (accel and gyro share the ODR; the LSM6DSV16BX has no
  400 Hz setting, 480 Hz is the closest to the legacy rate)
- **Firmware:** `sensors/imu/imu_appl.h`, `sensors/imu/imu_appl.c`,
  `sensors/imu/lsm6dsv16bx_sensor.c` · **GUI:** `biogui/platforms/biogapultra/biogapultra_imu/interface_biogapultra_imu.py`

### Overall layout

| Byte | Field name | Data type |
|---|---|---|
| 0 | Header (`0x56`) | uint8 |
| 1-2 | Packet counter | uint16 LE |
| 3-6 | Timestamp (µs) | uint32 LE |
| 7-234 | 19 samples × 12 bytes | see per-sample block |
| 235 | Trailer (`0x57`) | uint8 |

### Per-sample block (12 bytes)

| Rel. byte | Field name | Data type |
|---|---|---|
| 0-1 | Acceleration X | int16 BE |
| 2-3 | Acceleration Y | int16 BE |
| 4-5 | Acceleration Z | int16 BE |
| 6-7 | Angular rate X | int16 BE |
| 8-9 | Angular rate Y | int16 BE |
| 10-11 | Angular rate Z | int16 BE |

Absolute offset of sample *s* (0-based) = `7 + s*12`. Full scales are ±8 g
(`LSM6DSV16BX_8g`, 0.244 mg/LSB) and ±2000 dps (`LSM6DSV16BX_2000dps`,
70 mdps/LSB) — see [Scaling](#6-scaling-and-units).

---

## 3.3 Microphone (PDM audio)

Single-channel PDM microphone, 16 kHz, 16-bit PCM, 64 samples per packet.

- **Header:** `0xAA` (`MIC_DATA_HEADER`) · **Trailer:** `0x55` (`MIC_DATA_TRAILER`)
- **Total length:** 136 bytes (`MIC_PCKT_SIZE` = 1+2+4+128+1)
- **Samples per packet:** 64 (`MIC_SAMPLES_PER_PACKET`)
- **Firmware:** `sensors/mic/mic_appl.h`, `sensors/mic/mic_appl.c`

### Overall layout

| Byte | Field name | Data type |
|---|---|---|
| 0 | Header (`0xAA`) | uint8 |
| 1-2 | Packet counter | uint16 LE |
| 3-6 | Timestamp (µs) | uint32 LE |
| 7-134 | 64 PCM samples × 2 bytes | int16 LE (each) |
| 135 | Trailer (`0x55`) | uint8 |

Absolute offset of PCM sample *s* (0-based) = `7 + s*2`.

---

## 3.4 WULPUS PRO (ultrasound)

WULPUS PRO is driven by an external MSP430 that captures ultrasound frames and
streams them over SPI to the nRF5340, which forwards them over BLE. One
ultrasound frame is a full-duplex exchange of **4 × 201 SPI bytes**. The
nRF5340 wraps each 201-byte SPI chunk in one BLE packet, so **one ultrasound
frame = 4 BLE packets** with headers `0x10, 0x11, 0x12, 0x13`.

- **Headers:** `0x10`–`0x13` (`WULPUS_BLE_HDR_XFER_0..3`)
- **Total length per BLE packet:** 211 bytes (`WULPUS_BLE_PKT_SIZE` = 201 + 1 + 9)
- **Firmware:** `sensors/wulpus/wulpus_appl.c` · **GUI:** `biogui/platforms/wulpus_pro/interface_wulpus_pro.py`, `biogui/platforms/wulpus_pro/protocol.py`

### BLE packet layout (each of the 4 chunks)

| Byte | Field name | Data type |
|---|---|---|
| 0 | Chunk header (`0x10` / `0x11` / `0x12` / `0x13`) | uint8 |
| 1-2 | Frame counter | uint16 LE |
| 3-6 | Frame timestamp (µs) | uint32 LE |
| 7-207 | SPI transfer payload chunk (201 bytes) | opaque bytes |
| 208-210 | Reserved (zero) | uint8 × 3 |

The header/counter/timestamp prefix (bytes 0-6) is **identical in structure to the
ExG/MIC/PPG packets**. The nRF5340 writes a per-frame **counter** (increments once
per ultrasound frame, resets each streaming session) and a **µs timestamp**
(captured once per frame at acquisition, in the SPI thread when `HOST_DATA_RDY`
fires) and **mirrors the same values into all four chunks** of the frame — so the receiver
can read them from any chunk (the GUI reads them from the 4th, `0x13`). The
201-byte SPI payload follows at byte 7; bytes 208-210 stay zero.

### Reassembled ultrasound frame payload

Concatenate the four 201-byte SPI chunks (bytes 7-207 of each) in header
order `0x10 → 0x13` to get an 804-byte frame payload:

| Byte (in frame) | Field name | Data type |
|---|---|---|
| 0 | SOF mask (start-of-frame marker) | uint8 |
| 1 | `tx_rx_id` (active TX/RX config index) | uint8 |
| 2-3 | Acquisition number | uint16 LE |
| 4-803 | 400 ultrasound RF samples | int16 LE (each) |

`tx_rx_id` selects which of up to 16 TX/RX configurations produced this frame; the
GUI routes samples to the matching signal by this id.

### WULPUS MSP430 configuration packet (host → device)

The host configures the MSP430 by writing a 105-byte config packet
(`PACKAGE_LEN` = 105). It may arrive either after a `START_WULPUS_STREAMING` (41)
command byte, or as a raw write with no command prefix (the firmware `default:`
case forwards any unrecognised payload to the MSP430; long configs may be
fragmented and are reassembled by `wulpus_set_msp_config`).

| Byte | Field name | Data type |
|---|---|---|
| 0 | Start byte: `0xFA` (250) = new config, `0xFB` (251) = restart | uint8 |
| 1+ | MSP430 register block (see `configuration_package` in `protocol.py`) | mixed |
| … | zero-padded to 105 bytes total | uint8 |

Config field order (little-endian), built by `WulpusUssConfig.get_conf_package()`:
`dcdc_turnon` (u2), `meas_period` (u2), `meas_mode` (u4), `pulse_freq` (u4),
`num_pulses` (u1), `sampling_freq` (u2), `num_samples` (u2), `rx_gain` (u1),
`enable_env_det` (u1), `num_txrx_configs` (u1), then per config slot a `tx_config`
(u2) + `rx_config` (u2), then the timing block (`start_hvmuxrx`, `start_ppg`,
`turnon_adc`, `start_pgainbias`, `start_adcsampl`, `restart_capt`, `capt_timeout`,
`vga_rc_prech_cyc`, `vga_slope_code`, all u2). A **restart** packet (`0xFB`) is
just the start byte followed by zero padding — it stops the current acquisition
loop and waits for a fresh config.

WULPUS PRO has 16 independent TX switches and 16 independent RX switches, so
`tx_config`/`rx_config` are per-slot bitmasks over channel IDs 0–15.

---

## 3.5 PPG (MAXM86161)

PPG data from up to 8 MAXM86161 sensors on an I²C mux, up to 3 LEDs each
(green / IR / red). **Variable content, fixed size:** the packet is always 211
bytes, but only the enabled sensors' and LEDs' samples are present — the decoder
must use `sensor_mask` and `led_mask` to know where data ends.

- **Header:** `0x70` (`PPG_DATA_HEADER`) · **Trailer:** `0x71` (`PPG_DATA_TRAILER`, at byte 210)
- **Total length:** 211 bytes (`PPG_BLE_PKT_SIZE`)
- **Samples per packet:** `PPG_SAMPLES_PER_PKT` (4)
- **Firmware:** `sensors/ppg_new/ppg_new_appl.c`, `sensors/ppg_new/ppg_new_appl.h`
- **GUI parser:** none yet — this is a firmware-only packet type at present.

### Overall layout

| Byte | Field name | Data type |
|---|---|---|
| 0 | Header (`0x70`) | uint8 |
| 1-2 | Packet counter | uint16 LE |
| 3-6 | Timestamp (µs) | uint32 LE |
| 7 | `sensor_mask` — bit N = MUX channel N active | uint8 |
| 8 | `led_mask` — bit0 green, bit1 IR, bit2 red | uint8 |
| 9 | Trigger — debug GPIO state (0 / 1) | uint8 |
| 10-N | Sample data (see below) | uint16 LE per value |
| … | zero padding to byte 209 | uint8 |
| 210 | Trailer (`0x71`) | uint8 |

### Sample data region (starting at byte 10)

For each of the 4 samples, for each active sensor (in ascending `sensor_mask`
bit order), the enabled LED values appear in green → IR → red order. Each value
is a 19-bit ADC reading right-shifted by 3 into a uint16 LE:

```
for sample in 0..3:
    for ch where sensor_mask bit ch set (ascending):
        if led_mask bit0 (green): [G_lo, G_hi]   uint16 LE
        if led_mask bit1 (IR):    [IR_lo, IR_hi] uint16 LE
        if led_mask bit2 (red):   [R_lo, R_hi]   uint16 LE
```

Bytes present = `4 × (active sensors) × (enabled LEDs) × 2`. The rest is zero
padding. Inactive sensors/LEDs produce **no bytes** (not zeros interleaved).

---

## 4. Command packets (host → device)

Every command is a single command-code byte written to the NUS RX
characteristic, optionally followed by a config payload. Defined in
`ble/ble_commands.h`; dispatched in `ble/ble_appl.c` (`handle_ble_command`).

| Code | Command | Payload after code | Action |
|---|---|---|---|
| 12 | `SET_DEVICE_SETTINGS` | — | (no-op / reserved) |
| 13 | `GET_DEVICE_SETTINGS` | — | (not implemented) |
| 14 | `REQUEST_HARDWARE_VERSION` | — | → HW version response |
| 15 | `GET_BOARD_STATE` | — | → board state response |
| 17 | `REQUEST_BATTERY_STATE` | — | → battery/status response |
| 18 | `START_EEG_STREAMING` | — | start ExG (EEG) stream |
| 19 | `STOP_EEG_STREAMING` | — | stop ExG (EEG) stream |
| 20 | `SET_BOARD_STATE` | `[1]`: 1 = STREAMING_NORDIC, else GAP9_MASTER | set board state |
| 21 | `RESET_BOARD` | — | (no-op stub) |
| 22 | `ENTER_BOOTLOADERT_MODE` | — | (no-op stub) |
| 23 | `SET_TRIGGER_STATE` | — | deprecated (trigger removed) |
| 24 | `GO_TO_SLEEP` | — | (no-op stub) |
| 25 | `RESET_GAP9` | — | (no-op stub) |
| 26 | `START_MIC_STREAMING` | — | start microphone stream |
| 27 | `STOP_MIC_STREAMING` | — | stop microphone stream |
| 28 | `REQUEST_AVAILABLE_SENSORS` | — | → available-sensors response |
| 29 | `REQUEST_FIRMWARE_VERSION` | — | → FW version response |
| 30 | `REQUEST_CONNECTING_STRING` | — | → ready string `"BWF16"` |
| 31 | `START_STREAMING_ALL` | — | start EEG + MIC + IMU (synced) |
| 32 | `STOP_STREAMING_ALL` | — | stop EEG + MIC + IMU |
| 33 | `START_IMU_STREAMING` | — | start IMU stream |
| 34 | `STOP_IMU_STREAMING` | — | stop IMU stream |
| 35 | `START_EEG_MIC_STREAMING` | — | start EEG + MIC (synced) |
| 36 | `STOP_EEG_MIC_STREAMING` | — | stop EEG + MIC |
| 37 | `START_EMG_STREAMING` | — | start ExG (EMG) stream |
| 38 | `STOP_EMG_STREAMING` | — | stop ExG (EMG) stream |
| 39 | `START_PPG_STREAMING` | 11-byte PPG config (see below) | start PPG stream |
| 40 | `STOP_PPG_STREAMING` | — | stop PPG stream |
| 41 | `START_WULPUS_STREAMING` | WULPUS MSP430 config bytes (see [3.4](#34-wulpus-pro-ultrasound)) | forward config to MSP430 |
| 42 | `STOP_WULPUS_STREAMING` | — | stop WULPUS |
| 43 | `REQUEST_SYSTEM_STATUS` | — | → extended system-status response |
| *any other* | (unrecognised) | full payload | treated as raw WULPUS MSP430 config |

### `START_PPG_STREAMING` (39) config payload

11 config bytes follow the command code (total packet = 12 bytes). Trailing
bytes may be omitted; the firmware fills missing ones with defaults.

| Byte | Field name | Data type | Values / default |
|---|---|---|---|
| 0 | Command code (39) | uint8 | — |
| 1 | `sensor_mask` | uint8 | bit N = MUX channel N (default `0x01`) |
| 2 | `sample_rate_hz` | uint8 | ≥ 8 Hz (default 125) |
| 3 | `led_green` | uint8 | 0 = off, else PA current (default `0x7F`) |
| 4 | `led_ir` | uint8 | 0 = off, else PA current (default `0x7F`) |
| 5 | `led_red` | uint8 | 0 = off, else PA current (default `0x7F`) |
| 6 | `led_range` | uint8 | 0=4k 1=8k 2=16k 3=32k (default 3) |
| 7 | `tint` | uint8 | 0=14.8µs 1=29.4µs 2=58.7µs 3=117.3µs (default 3) |
| 8 | `adc_range` | uint8 | 0=4k 1=8k 2=16k 3=32k (default 3) |
| 9 | `sample_avg` | uint8 | 0=1× … 7=128× (default 0) |
| 10 | `alc_enable` | uint8 | 0=off 1=on (default 1) |
| 11 | `proximity_enable` | uint8 | 0=off 1=on (default 0) |

### `SET_BOARD_STATE` (20) values

Board-state constants (`core/common.h`): `STATE_STREAMING_NORDIC` = 50,
`STATE_GAP9_MASTER` = 60, `STATE_PROGRAM_WOLF` = 80. The command byte `data[1]`
is a boolean: `1` selects `STATE_STREAMING_NORDIC`, anything else selects
`STATE_GAP9_MASTER`.

### Example start/stop sequences (from BioGUI interfaces)

| Stream | Start sequence | Stop |
|---|---|---|
| EEG | `[20,1,0]`, wait 0.2 s, `[18]` | `[19]` |
| EMG | `[20,1,0]`, wait 0.2 s, `[37]` | `[38]` |
| EMG + MIC | `[37]`, wait 0.2 s, `[26]` | `[38]`, wait 0.2 s, `[27]` |

---

## 5. Response / status packets (device → host)

Built in `bsp/system_status/system_status.c`. These are short, self-identified
by their first byte (the request command code they answer), and most end in the
`0xAA` tail marker.

All battery/charger values in both status responses come from a cache that
the SDK power thread (`sensei-sdk: system/pwr/thread_pwr.c`) refreshes every
`THREAD_PWR_UPDATE_PERIOD_MS` (20 s) and on PMIC interrupt (USB plug/unplug,
button). Serving a request therefore costs **no PMIC/I2C access** and is safe
during streaming. State of charge is estimated from the battery voltage via a
LiPo lookup table (the MAX77654 has no fuel gauge). While charging, a live
voltage reading would track the charger rather than the cell, so the firmware
periodically (~every 60 s) **pauses the charger for ~350 ms**, lets the
unloaded cell relax, measures its near-rest voltage, and resumes charging —
so the percentage keeps rising credibly during a charge. The pause is
invisible to hosts (all responses come from the cache, filled while
charging); the percentage is capped at 99 % while charging and reads 100 %
once the charger signals charging done. The charging flag is the
charger state machine's "actively charging" bit — it is 0 when a charger is
attached but charging has completed. If **no battery is attached** (the BATT
pin is driven by the charger/VSYS and reads > 4.3 V — impossible for a real
LiPo), voltage and state of charge are reported as **0**. Temperature is the
IMU die temperature (cached for 2 s; only meaningful while the IMU is
powered).

### Battery / status — response to command 17 (7 bytes, legacy)

Byte layout matches what the legacy BioWolf Java GUI parses
(`RefreshRTDeviceInfo` in `FXMLDocumentController.java`; the original values
came from a BQ27441 fuel gauge). Prefer command 43 for new hosts: it carries
untruncated values and its header does not collide with WULPUS chunk headers.

| Byte | Field name | Data type | Notes |
|---|---|---|---|
| 0 | Command echo (`17`) | uint8 | `REQUEST_BATTERY_STATE` |
| 1 | Charging flag | uint8 | 1 = charger actively charging |
| 2 | Current (mA) | uint8 | charge current while charging, discharge current otherwise; clamped to 255 |
| 3 | Power (mW) | uint8 | input power on USB, discharge power on battery; clamped to 255 |
| 4 | State of charge (%) | uint8 | 0–100 |
| 5 | Voltage (0.1 V) | uint8 | e.g. 41 = 4.1 V (legacy GUI renders `v/10 . v%10`) |
| 6 | Temperature (°C) | uint8 | from IMU; 0 if read fails |

> Legacy-GUI quirk (GUI-side, not fixable from firmware): its "mW" label is
> wired to byte 4, so it displays the SoC number as power. The real power is
> in byte 3. Its streaming-mode battery display reads bytes from the old
> 234-byte BioWolf packet tail and does not apply to BioGAP packets.

### Extended system status — response to command 43 (21 bytes)

| Byte | Field name | Data type | Notes |
|---|---|---|---|
| 0 | Command echo (`43` = `0x2B`) | uint8 | `REQUEST_SYSTEM_STATUS` |
| 1 | Flags | uint8 | bit0 USB present, bit1 charging, bit2 charger fault, bit3 PMIC thermal alarm, bit4 temperature valid, bit5 battery present |
| 2 | Charger state | uint8 | raw MAX77654 `CHG_DTLS`: 0 off, 1 prequal, 2/3 CC, 4/5 CV, 6/7 top-off, 8/9 done, 10–12 faults. With no battery attached it typically reads "done" — check flags bit5 |
| 3 | State of charge (%) | uint8 | 0–100, voltage-estimated |
| 4–5 | Battery voltage (mV) | uint16 LE | |
| 6–7 | System voltage VSYS (mV) | uint16 LE | |
| 8–9 | Charger input voltage (mV) | uint16 LE | 0 when unplugged |
| 10–11 | Charger input current (0.1 mA) | uint16 LE | 0 when unplugged |
| 12–13 | Battery current (0.1 mA) | uint16 LE | charge current while charging, discharge current otherwise (auto-ranged, ~1 % of range resolution) |
| 14–15 | Charger input power (mW) | uint16 LE | 0 when unplugged |
| 16–17 | Battery power (mW) | uint16 LE | into battery while charging, out of battery otherwise |
| 18–19 | Temperature (0.01 °C) | int16 LE | IMU die temperature; check flags bit4 |
| 20 | Tail | uint8 | `0xAA` |

### Hardware version — response to command 14 (4 bytes)

| Byte | Field name | Data type | Value |
|---|---|---|---|
| 0 | Command echo (`14`) | uint8 | — |
| 1 | `HARDWARE_VERSION` | uint8 (ASCII) | `'2'` |
| 2 | `HARDWARE_REVISION` | uint8 (ASCII) | `'b'` |
| 3 | Tail | uint8 | `0xAA` |

### Firmware version — response to command 29 (4 bytes)

| Byte | Field name | Data type | Value |
|---|---|---|---|
| 0 | Command echo (`29`) | uint8 | — |
| 1 | `FIRMWARE_VERSION` | uint8 (ASCII) | `'2'` |
| 2 | `FIRMWARE_REVISION` | uint8 (ASCII) | `'c'` |
| 3 | Tail | uint8 | `0xAA` |

### Available sensors — response to command 28 (4 bytes)

| Byte | Field name | Data type | Value |
|---|---|---|---|
| 0 | Command echo (`28`) | uint8 | — |
| 1 | Available flag | uint8 | 1 |
| 2 | Reserved | uint8 | 0 |
| 3 | Tail | uint8 | `0xAA` |

### Board state — response to command 15 (1 byte)

| Byte | Field name | Data type | Value |
|---|---|---|---|
| 0 | Board state | int8 | 50 / 60 / 80 (see above) |

### Ready / connecting string — response to command 30 (5 bytes)

ASCII string `"BWF16"` (`0x42 0x57 0x46 0x31 0x36`), no header or trailer.

---

## 6. Scaling and units

| Signal | Raw type | Conversion | Source |
|---|---|---|---|
| ExG (EEG/EMG) | int24 BE | `V = raw × Vref / (gain × (2²³−1))`, with `Vref = 2.5 V`, `gain = 6`. In µV: `raw × 2.5 / (6 × (2²³−1)) × 1e6`. | `interface_biogapultra_eeg.py` |
| IMU accel | int16 BE | ±8 g full scale → `mg = raw × 0.244` (`lsm6dsv16bx_from_fs8_to_mg`) | `imu/driver/lsm6dsv16bx_reg.c`, `lsm6dsv16bx_sensor.c` |
| IMU gyro | int16 BE | ±2000 dps full scale → `mdps = raw × 70` (`lsm6dsv16bx_from_fs2000_to_mdps`) | `imu/driver/lsm6dsv16bx_reg.c`, `lsm6dsv16bx_sensor.c` |
| Microphone | int16 LE | 16-bit signed PCM, 16 kHz mono, no extra scaling | `mic_appl.h` |
| WULPUS RF | int16 LE | raw ADC counts; RX gain (dB) set per config `rx_gain` | `wulpus_pro/protocol.py` |
| PPG | uint16 LE | 19-bit ADC reading right-shifted by 3 (`value = adc >> 3`) | `ppg_new_appl.c` |

---

## Known discrepancies

Stale comments / constants in the code that contradict actual behaviour. The
runtime **code** and this document are authoritative; these are flagged for
cleanup.

| Location | Says | Actually |
|---|---|---|
| `ble/bluetooth.h` doc comment | EEG packet = 192 B | 211 B |
| `ble/bluetooth.h` doc comment | MIC packet = 131 B | 136 B |
| `afe/ads_defs.h` header comment | "234-byte" packet | `EXG_PCK_LNGTH` = 211 B |
| `sensors/mic/mic_appl.h` comment | "132 bytes" | macro computes 136 B |
| (resolved) ExG sample rate | older comments/GUI said 250 Hz | **Actual = 500 Hz.** The ADS1298 runs in High-Resolution mode (CONFIG1 = `0xC0` \| data-rate code 6 → DR = 110 = 500 SPS). Firmware `EEG_SAMPLE_RATE`/`EMG_SAMPLE_RATE` and the biogapultra GUI interfaces now say 500 Hz (packet rate 125/s). |

---

## Appendix: legacy formats

These coexist in the BioGUI codebase for backward compatibility with older
firmware/hardware. They are **not** produced by the current `src_NRF` firmware
and are documented only so parsers don't confuse them with the formats above.

### Legacy 234-byte ExG packet

`biogui/platforms/biogap/interface_biogap.py` and
`biogui/platforms/biogapultra/interface_biogapultra.py` use `packetSize = 234`
with old fixed byte offsets and **no timestamp field**. Incompatible with the
current 211-byte ExG layout ([3.1](#31-exg--eeg--emg-ads1298--2)).

### Legacy 128-byte IMU packet (LIS2DUXS12)

Mainboard revisions before 2026 carried a LIS2DUXS12 3-axis accelerometer
(no gyroscope) at 400 Hz. Same header/trailer (`0x56`/`0x57`) and 7-byte
prefix, but **128 bytes total**: 20 samples × 6 bytes (accel X/Y/Z int16 BE
only), trailer at byte 127. Disambiguate from the current 236-byte format by
total length.

### Legacy (non-pro) WULPUS

`biogui/platforms/wulpus/protocol.py`: `PACKAGE_LEN = 68` config, and the 400-sample
acquisition is split as `397 US + 3 IMU` when `meas_mode = 101`
(accelerometer mode). The original 8-channel WULPUS also shares one 16-bit
switch-ID space between interleaved TX/RX bits, unlike WULPUS PRO's 16
independent TX + 16 independent RX switches.
