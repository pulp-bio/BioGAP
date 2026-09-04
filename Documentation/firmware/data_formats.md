# Data Formats

This document specifies the BLE packet formats for all sensor data types transmitted over the Nordic UART Service (NUS).

## Common Packet Structure

All sensor data packets follow a common pattern:

```
[Header] [Metadata] [Payload] [Trailer]
```

- **Header** (1 byte): Identifies the packet type
- **Metadata**: Counter and timestamp
- **Payload**: Sensor-specific data
- **Trailer** (1 byte): Frame end marker

## EXG Packet (EEG/EMG)

**Header**: `0x55` | **Trailer**: `0xAA` | **Size**: 211 bytes

```
Offset  Size   Field                   Description
──────  ────   ──────────────────      ───────────────────────────────────────
0       1      header                  0x55
1-2     2      packet_counter          uint16_t, little-endian, wraps at 65535
3-6     4      timestamp               uint32_t, microseconds, little-endian
7-206   200    samples[4]              4 samples x 50 bytes each (see below)
207     1      board_id                CONFIG_BOARD_ID (1-255)
208     1      sync_pulse_count        Inter-board sync pulse counter
209     1      reserved                0x00
210     1      trailer                 0xAA
```

### Sample Structure (50 bytes each)

Each sample within the EXG packet:

```
Offset  Size   Field                   Description
──────  ────   ──────────────────      ───────────────────────────────────────
0-23    24     ads_a_data[8]           ADS1298_A: 8 channels x 3 bytes (24-bit)
24-47   24     ads_b_data[8]           ADS1298_B: 8 channels x 3 bytes (24-bit)
48      1      counter_extra           Additional counter
49      1      reserved                0x00
```

### PPG Multiplexing

When `PPG_ACTIVE` is defined, the last 2 ADS1298 channels in each sample are replaced:

```
ADS1298_A channel 7 (bytes 21-23) → PPG Red LED (3 bytes)
ADS1298_A channel 8 (bytes 24-26) → PPG IR LED (3 bytes)
  (or equivalently, ADS1298_B channel 7-8 depending on mapping)
```

### Packet Counter

The packet counter (`packet_counter`) is a monotonically increasing uint16_t that wraps at 65535. It can be used to detect lost packets on the receiving end by checking for gaps in the sequence.


## IMU Packet

**Header**: `0x56` | **Trailer**: `0x57` | **Size**: 127 bytes

```
Offset  Size   Field                   Description
──────  ────   ──────────────────      ───────────────────────────────────────
0       1      header                  0x56
1       1      packet_counter          uint8_t, wraps at 255
2-5     4      timestamp               uint32_t, microseconds, little-endian
6-125   120    samples[20]             20 samples x 6 bytes each (see below)
126     1      trailer                 0x57
```

### IMU Sample Structure (6 bytes each)

Each acceleration sample:

```
Offset  Size   Field     Description
──────  ────   ──────    ───────────────────────────────────────
0-1     2      x         int16_t, big-endian, X-axis acceleration
2-3     2      y         int16_t, big-endian, Y-axis acceleration
4-5     2      z         int16_t, big-endian, Z-axis acceleration
```


## Microphone Packet

**Header**: `0xAA` | **Trailer**: `0x55` | **Size**: 136 bytes

```
Offset  Size   Field                   Description
──────  ────   ──────────────────      ───────────────────────────────────────
0       1      header                  0xAA
1-2     2      packet_counter          uint16_t, little-endian, wraps at 65535
3-6     4      timestamp               uint32_t, microseconds, little-endian
7-134   128    samples[64]             64 samples x 2 bytes each (16-bit PCM)
135     1      trailer                 0x55
```

### Audio Sample Format

Each sample is a 16-bit signed integer (little-endian):

```
Byte 0: LSB
Byte 1: MSB
```

This is standard 16-bit PCM audio at 16 kHz, mono channel.

## Battery Status Packet

**Size**: 7 bytes (sent as response to command, not a streaming packet)

```
Offset  Size   Field                   Description
──────  ────   ──────────────────      ───────────────────────────────────────
0       1      command_code            17 (REQUEST_BATTERY_STATE)
1       1      is_charging             0 = not charging, 1 = charging
2       1      reserved                0x00
3       1      power_mw                Power consumption in mW (truncated to uint8)
4       1      soc_percent             State of charge (0-100%)
5       1      voltage_mv              Battery voltage in mV (truncated to uint8)
6       1      temperature_celsius     Die temperature from IMU sensor
```

Note that `power_mw` and `voltage_mv` are truncated to uint8, so values above 255 are clipped. For full-precision readings, the raw PMIC registers must be queried directly.

---

## Packet Identification Summary

| Header | Trailer | Type | Size | Sensor | Packet Rate |
|--------|---------|------|------|--------|-------------|
| `0x55` | `0xAA` | EXG | 211 bytes | EEG/EMG (ADS1298) | 62.5 Hz |
| `0x56` | `0x57` | IMU | 127 bytes | LIS2DUXS12 | 20 Hz |
| `0xAA` | `0x55` | MIC | 136 bytes | PDM Microphone | 250 Hz |

## Timestamp

All packet types include a 32-bit microsecond timestamp (`k_cyc_to_us(k_cycle_get_32())`). The timestamp represents the time when the last sample in the packet was captured. At 32-bit width, the timestamp wraps every ~4295 seconds (~71.6 minutes).

## Receiving and Parsing

To parse the BLE data stream:

1. Read the first byte to determine packet type
2. Read the expected number of bytes for that packet type
3. Verify the trailer byte matches the expected value
4. Extract metadata (counter, timestamp)
5. Process payload samples