# Configuration

This document covers the most important build-time configuration options for the BioGAP firmware added during the refactor.

## Build System

### Environment Variable

| Variable | Description |
|----------|-------------|
| `SENSEI_SDK_ROOT` | Path to the SENSEI SDK root directory (required) |

## Kconfig Options

Custom Kconfig options are defined in the firmware's `Kconfig` file:

### Sensor Selection

| Option | Type | Default | Description |
|--------|------|---------|-------------|
| `CONFIG_SENSOR_EEG` | bool | y | Enable EEG mode (ADS1298 unipolar power) |
| `CONFIG_SENSOR_EMG` | bool | n | Enable EMG mode (ADS1298 bipolar power) |

> These two options are mutually exclusive. Only one can be enabled at a time.

### Board Synchronization

| Option | Type | Default | Description |
|--------|------|---------|-------------|
| `CONFIG_BOARD_SYNC_ROLE_STANDALONE` | bool | y | Single device, no sync |
| `CONFIG_BOARD_SYNC_ROLE_PRIMARY` | bool | n | Primary board in multi-device setup |
| `CONFIG_BOARD_SYNC_ROLE_SECONDARY` | bool | n | Secondary board in multi-device setup |
| `CONFIG_BOARD_ID` | int | 1 | Unique board identifier (1-255) |
| `CONFIG_BOARD_SYNC_PERIODIC_MS` | int | 1000 | Periodic sync pulse interval (ms) |

