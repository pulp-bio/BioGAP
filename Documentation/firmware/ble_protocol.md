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
| `0xAA` | Microphone | `0x55` | [Data Formats](./data_formats.md) |

See [Data Formats](./data_formats.md) for detailed packet structure documentation.
