# BSP Module (Board Support Package)

The BSP module (`src_NRF/bsp/`) provides board-level hardware management including power supply control, battery monitoring, and system status aggregation.

## Submodules

```
bsp/
├── pwr_bsp.c/h          # Board-level power initialization and control
├── battery/
│   ├── battery.c        # Battery monitoring thread
│   └── battery.h        # Battery status structure
├── power/
│   ├── power.c          # ADS1298 power rail control
│   └── power.h          # Power API
└── system_status/
    ├── system_status.c  # System info aggregator
    └── system_status.h  # Status API
```

## Power Management (`pwr_bsp`)

### Key Functions

| Function | Description |
|----------|-------------|
| `pwr_init()` | Initialize PMIC I2C communication |
| `pwr_bsp_start()` | Configure all PMIC rails (SBB0-2, LDO0-1) |
| `pwr_charge_enable()` | Enable battery charging (285mA input, 90mA fast-charge, 4.2V CV) |
| `pwr_bsp_soft_rst_cb()` | Soft-reset button GPIO interrupt handler |
| `gap9_pwr(on)` | Power on/off the GAP9 co-processor via I2C |



## ADS1298 Power Control (`power`)

The ADS1298 analog supply voltage depends on the electrode configuration:

### Unipolar Mode (EEG)

Used for EEG with common reference electrodes.

```
power_ads_on_unipolar()  → LDO1 set to 3.0V
power_ads_off_unipolar() → LDO1 disabled
```

### Bipolar Mode (EMG)

Used for EMG with differential electrode pairs.

```
power_ads_on_bipolar()  → LDO1 set to 1.5V, SBB1 set to 2.7V
power_ads_off_bipolar() → LDO1 disabled, SBB1 disabled
```

## Battery Monitoring (`battery`)

### Battery Thread

- **Stack**: 1024 bytes, **Priority**: 6
- **Poll interval**: 5 seconds
- Runs continuously, reading PMIC battery gas gauge data

### Battery Status Structure

```c
typedef struct {
    uint8_t soc_percent;       // State of charge (0-100%)
    uint16_t voltage_mv;       // Battery voltage in millivolts
    bool is_charging;          // Whether battery is charging
    uint16_t power_mw;         // Power consumption in milliwatts
    const char *power_source;  // "USB/External" or "Battery"
} battery_status_t;
```

### Key Functions

| Function | Description |
|----------|-------------|
| `battery_update_thread()` | Main thread loop, polls PMIC every 5 seconds |
| `get_battery_status()` | Returns current `battery_status_t` |

### Behavior During Streaming

Battery reads are **skipped** during active sensor streaming to avoid I2C/SPI interference. The battery data is only read when sensors are idle.