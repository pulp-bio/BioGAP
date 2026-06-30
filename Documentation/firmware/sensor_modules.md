# Sensor Modules

The sensor modules (`src_NRF/sensors/`) each run in a dedicated Zephyr thread and handle acquisition from a specific sensor type. All modules follow a similar pattern: wait on a start semaphore, power up the sensor, acquire data in a loop, construct BLE packets, and enqueue them for transmission.

## EEG Module (`sensors/eeg/`)

### Files

| File | Purpose |
|------|---------|
| `eeg_appl.c` | EEG streaming thread and ADS1298 control flow |
| `eeg_appl.h` | EEG configuration, states, packet format constants |

### Configuration

| Parameter | Value |
|-----------|-------|
| Sensor | 2x ADS1298 (16 channels) |
| Sample rate | 500 Hz |
| Samples per packet | 4 |
| Packet size | 211 bytes |
| Thread stack | 2048 bytes |
| Power mode | Unipolar (3.0V LDO) |

### State Machine

```
EEG_STATE_IDLE → EEG_STATE_STARTING → EEG_STATE_STREAMING → EEG_STATE_STOPPING → EEG_STATE_IDLE
```

### Acquisition Flow

1. Wait on `eeg_start_sem` (given by BLE command handler)
2. Power on ADS1298 analog rail: `power_ads_on_unipolar()`
3. First run only: verify ADS1298 ID (expects `0xD2`)
4. Initialize ADS1298 registers via `ads_init()`
5. Participate in sync barrier: `sync_wait(SYNC_SUBSYSTEM_EXG, timeout)`
6. Start conversion: `ads_start()`
7. Streaming loop: call `process_ads_data()` which:
   - Waits for DRDY interrupt
   - Reads 27 bytes from each ADS1298
   - Constructs BLE packet (4 samples per packet)
   - Enqueues via `add_data_to_send_buffer()`
8. On stop command: `ads_stop()` → power off → return to IDLE

### Configuration Structure

```c
typedef struct {
    uint8_t sample_rate;    // ADS1298 ODR index (default: 6)
    uint8_t ads_mode;       // ADS1298 mode (default: 0)
    uint8_t channel_2_func; // Channel 2 function (default: 2)
    uint8_t channel_4_func; // Channel 4 function (default: 4)
    uint8_t gain;           // PGA gain setting (default: 0)
} eeg_config_t;
```

### Kconfig

- `CONFIG_SENSOR_EEG=y` - Enable EEG mode (mutually exclusive with EMG)

---

## EMG Module (`sensors/emg/`)

### Files

| File | Purpose |
|------|---------|
| `emg_appl.c` | EMG streaming thread (mirrors EEG module) |
| `emg_appl.h` | EMG configuration, states, packet format constants |

The EMG module is architecturally identical to the EEG module with two key differences:

1. **Power configuration**: Uses bipolar mode (1.5V LDO + 2.7V SBB1) via `power_ads_on_bipolar()`
2. **Kconfig**: `CONFIG_SENSOR_EMG=y` (mutually exclusive with `CONFIG_SENSOR_EEG`)

All other parameters (sample rate, packet format, thread configuration) are the same as EEG.

### Kconfig

- `CONFIG_SENSOR_EMG=y` - Enable EMG mode (mutually exclusive with EEG)

---

## IMU Module (`sensors/imu/`)

### Files

| File | Purpose |
|------|---------|
| `imu_appl.c` | IMU streaming thread (400 Hz, 20 samples per packet) |
| `imu_appl.h` | IMU states, packet format constants |
| `lis2duxs12_sensor.c/h` | Low-level LIS2DUXS12 driver (I2C, DRDY interrupt) |
| `driver/lis2duxs12_reg.c/h` | ST platform-independent register driver (auto-generated) |

### Configuration

| Parameter | Value |
|-----------|-------|
| Sensor | LIS2DUXS12 (ST) |
| Interface | I2C (address 0x19) |
| Sample rate | 400 Hz |
| Samples per packet | 20 |
| Packet size | 127 bytes |
| Thread stack | 2048 bytes |

### DRDY Handling

The LIS2DUXS12 asserts INT1 (GPIO P0.23) when new acceleration data is available:
1. GPIO interrupt fires → `lis2duxs12_drdy_handler()`
2. Gives `lis2duxs12_drdy_sem` semaphore
3. IMU streaming thread waits on semaphore, then reads 6 bytes (X, Y, Z as int16_t)

### Acquisition Flow

1. Wait on `imu_start_sem`
2. Initialize LIS2DUXS12: set output data rate, range, bandwidth
3. Configure DRDY interrupt on INT1
4. Participate in sync barrier (if multi-sensor start)
5. Streaming loop:
   - Wait for `lis2duxs12_drdy_sem`
   - Read X, Y, Z acceleration (3 x int16_t = 6 bytes)
   - Fill packet buffer (20 samples)
   - When full, enqueue via `add_data_to_send_buffer()`
6. On stop: disable sensor, return to IDLE

### State Machine

```
IMU_STATE_IDLE → IMU_STATE_STARTING → IMU_STATE_STREAMING → IMU_STATE_STOPPING → IMU_STATE_IDLE
```

---

## Microphone Module (`sensors/mic/`)

### Files

| File | Purpose |
|------|---------|
| `mic_appl.c` | PDM microphone streaming thread |
| `mic_appl.h` | Mic states, packet format constants |

### Configuration

| Parameter | Value |
|-----------|-------|
| Sensor | PDM digital microphone |
| Interface | DMIC peripheral (NRFX PDM driver) |
| Sample rate | 16 kHz |
| Bit width | 16-bit |
| Channel | Mono (left channel) |
| Samples per packet | 64 |
| Packet size | 136 bytes |
| Thread stack | 2048 bytes |
| Memory slab | 8 blocks of 128 bytes each |

### PDM Configuration (Device Tree Overlay)

| Parameter | Value |
|-----------|-------|
| CLK pin | P0.04 |
| DIN pin | P0.12 |
| ACLK source | 12.288 MHz |

### Acquisition Flow

1. Wait on `mic_start_sem`
2. Configure PDM peripheral via `pdm_configure()`
3. Start DMIC with `dmic_start()`
4. Participate in sync barrier (if multi-sensor start)
5. Streaming loop:
   - `dmic_read()` blocks until audio block available from memory slab
   - Copy 128 bytes (64 samples x 2 bytes) into BLE packet
   - Enqueue via `add_data_to_send_buffer()`
   - Release audio block back to memory slab
6. On stop: `dmic_stop()`, return to IDLE

### Buffer Management

Audio blocks are managed via a Zephyr memory slab:

```c
K_MEM_SLAB_DEFINE_STATIC(mic_mem_slab, MAX_BLOCK_SIZE, 8, 4)
```

- 8 blocks of `MAX_BLOCK_SIZE` bytes (128 bytes each = 4ms of audio at 16 kHz)
- Blocks are acquired by the DMIC driver and released by the streaming thread after copying

### State Machine

```
MIC_STATE_IDLE → MIC_STATE_STARTING → MIC_STATE_STREAMING → MIC_STATE_STOPPING → MIC_STATE_IDLE
```

---

## PPG Module (`sensors/ppg/`)

### Files

| File | Purpose |
|------|---------|
| `ppg_appl.c` | MAX86150 PPG driver (I2C) |
| `ppg_appl.h` | MAX86150 register definitions, circular buffer |

### Configuration

| Parameter | Value |
|-----------|-------|
| Sensor | MAX86150 (Maxim) |
| Interface | I2C (address 0x5E) |
| LEDs | Red + IR |
| Sample rate | ~100 SPS |
| ADC range | 32768 nA |
| LED amplitude | 0x25 (Red and IR) |
| FIFO depth | 32 entries |

> **Note**: The PPG module is not fully integrated. The PPG thread is commented out. PPG data is currently read on-demand and multiplexed into the EXG BLE packet when `PPG_ACTIVE` is defined.

### Data Buffer

PPG samples are stored in a circular buffer:

```c
typedef struct Record {
    uint32_t red[40];   // Red LED samples (19-bit values)
    uint32_t IR[40];    // IR LED samples (19-bit values)
    uint16_t head;
    uint16_t tail;
} sense_struct;
```