# BLE Protocol

The BioGAP firmware uses a custom command protocol over the Nordic UART Service (NUS) for bidirectional communication. Commands are sent from the BLE peer to the device, and responses are sent back as NUS notifications.

## Command Codes

| Code | Name | Description |
|------|------|-------------|
| 12 | `SET_DEVICE_SETTINGS` | Configure device settings |
| 13 | `GET_DEVICE_SETTINGS` | Read device settings |
| 14 | `REQUEST_HARDWARE_VERSION` | Returns hardware version (major='2', minor='b') |
| 15 | `GET_BOARD_STATE` | Returns current board state |
| 17 | `REQUEST_BATTERY_STATE` | Returns battery information (7 bytes) |
| 18 | `START_EEG_STREAMING` | Start EEG acquisition and BLE streaming |
| 19 | `STOP_EEG_STREAMING` | Stop EEG acquisition |
| 20 | `SET_BOARD_STATE` | Set operating mode (Nordic/GAP9) |
| 21 | `RESET_BOARD` | Reset the board |
| 24 | `GO_TO_SLEEP` | Enter low-power sleep mode |
| 26 | `START_MIC_STREAMING` | Start PDM microphone capture |
| 27 | `STOP_MIC_STREAMING` | Stop microphone |
| 31 | `START_STREAMING_ALL` | Start all sensors with barrier synchronization |
| 32 | `STOP_STREAMING_ALL` | Stop all active sensors |
| 33 | `START_IMU_STREAMING` | Start IMU (accelerometer) streaming |
| 34 | `STOP_IMU_STREAMING` | Stop IMU streaming |
| 35 | `START_EEG_MIC_STREAMING` | Combined EEG + microphone streaming |
| 37 | `START_EMG_STREAMING` | Start EMG acquisition and BLE streaming |
| 38 | `STOP_EMG_STREAMING` | Stop EMG acquisition |
| 39 | `START_PPG_STREAMING` | Start multi-PPG streaming (`CONFIG_SENSOR_PPG_NEW`) |
| 40 | `STOP_PPG_STREAMING` | Stop multi-PPG streaming |
| 41 | `START_WULPUS_STREAMING` | Forward MSP430 config and start ultrasound (`CONFIG_SENSOR_WULPUS`) |
| 42 | `STOP_WULPUS_STREAMING` | Stop ultrasound streaming |
| 43 | `REQUEST_SYSTEM_STATUS` | Returns extended battery/system status |
| 44 | `START_MMWAVE_STREAMING` | Start radar frame streaming (`CONFIG_SENSOR_MMWAVE`) |
| 45 | `STOP_MMWAVE_STREAMING` | Stop radar streaming |
| 46 | `CONFIGURE_MMWAVE` | Write the radar register profile |
| 47 | `TURN_OFF_MMWAVE` | Cut the radar's power rail |
| 48 | `TURN_ON_MMWAVE` | Power the radar and probe it |
| 49 | `CHANGE_IFGAIN_MMWAVE` | Set radar IF gain; takes one value byte (dB) |
| 50 | `CHANGE_TXPOWER_MMWAVE` | Set radar TX power; takes one value byte (0-31) |
| 51 | `CHANGE_FPS_MMWAVE` | Set radar frame rate; takes one value byte (fps) |

The canonical list is [`ble/ble_commands.h`](../../Firmware/src_NRF/ble/ble_commands.h) — check it before
assigning a new code, since several of the codes above only exist in
shield-specific builds and are easy to miss.

## Command Flow Examples

### Start EEG Streaming

```
Peer → Device:  [18]                          (START_EEG_STREAMING)
Device → Peer:  [0x55][cnt][ts][data...][tr]   (EEG data packets @ 500 Hz)
...
Peer → Device:  [19]                          (STOP_EEG_STREAMING)
Device → Peer:  (streaming stops)
```

### Start All Sensors (Synchronized)

```
Peer → Device:  [31]                          (START_STREAMING_ALL)
                  ↓ sync barrier: all sensors start simultaneously
Device → Peer:  [0x55][...] (EEG data @ 500 Hz)
Device → Peer:  [0x56][...] (IMU data @ 400 Hz)
Device → Peer:  [0xAA][...] (MIC data @ 16 kHz)
...
Peer → Device:  [32]                          (STOP_STREAMING_ALL)
```

### Query Battery

```
Peer → Device:  [17]                          (REQUEST_BATTERY_STATE)
Device → Peer:  [17][charging][rsv][pwr][soc][vbat][temp]
```

### Combined EEG + Microphone

```
Peer → Device:  [35]                          (START_EEG_MIC_STREAMING)
                  ↓ sync barrier: EEG + MIC start simultaneously
Device → Peer:  [0x55][...] (EEG data)
Device → Peer:  [0xAA][...] (MIC data)
...
Peer → Device:  [19]                          (STOP_EEG_STREAMING)
Peer → Device:  [27]                          (STOP_MIC_STREAMING)
```

## Data Packet Headers

Streaming data packets are identified by their first byte (header):

| Header | Sensor | Trailer | See |
|--------|--------|---------|-----|
| `0x55` | EEG/EMG (EXG) | `0xAA` | [Data Formats](./data_formats.md) |
| `0x56` | IMU | `0x57` | [Data Formats](./data_formats.md) |
| `0x60` | mmWave radar | `0x61` | [Data Formats](./data_formats.md) |
| `0x70` | PPG | `0x71` | [Data Formats](./data_formats.md) |
| `0xAA` | Microphone | `0x55` | [Data Formats](./data_formats.md) |

Two further headers are in use and must be avoided when adding a sensor:
`0x10`-`0x13` mark the four chunks of a WULPUS ultrasound frame, and `0x2B`
(= 43) is the extended system-status response.

See [Data Formats](./data_formats.md) for detailed packet structure documentation.

### mmWave radar

The radar is the only sensor whose payload spans several packets, because one
frame is larger than a BLE notification:

```
[0]        0x60  header
[1:5]      frame timestamp, us, big endian (bit 0 = external sync level)
[5]        chunk index, 0-based
[6]        total number of chunks in this frame
[7:243]    payload: 12-bit packed ADC samples, zero padded in the last chunk
[243]      0x61  trailer
```

All chunks of one frame carry the same timestamp, so a host can detect a lost
chunk and discard the partial frame. With the default profile (100 fps,
32 chirps x 8 samples, 1 RX antenna) a frame is 256 samples = 384 packed bytes
= 2 packets, i.e. 200 packets/s.
