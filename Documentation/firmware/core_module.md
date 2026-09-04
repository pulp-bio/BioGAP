# Core Module

The core module (`src_NRF/core/`) provides shared utilities used across the firmware: common definitions, I2C helpers, barrier synchronization, and inter-board GPIO synchronization.

## Barrier Synchronization (`sync_streaming`)

Provides a counting barrier to ensure multiple sensor subsystems start acquisition simultaneously. This is critical for temporal alignment of multi-modal data.

### API

| Function | Description |
|----------|-------------|
| `sync_begin(count)` | Initialize barrier for `count` subsystems |
| `sync_wait(subsystem_id, timeout_ms)` | Register and block until all subsystems arrive |
| `sync_reset()` | Clean up barrier state |

### Subsystem Identifiers

| ID | Name | Bitmask |
|----|------|---------|
| 0 | `SYNC_SUBSYSTEM_EXG` | bit 0 |
| 1 | `SYNC_SUBSYSTEM_MIC` | bit 1 |
| 2 | `SYNC_SUBSYSTEM_IMU` | bit 2 |

### Implementation

The barrier uses atomic operations for thread-safe registration:
1. `sync_begin(N)` sets the expected count and resets the arrival counter
2. Each `sync_wait()` atomically increments the arrival counter and sets its bitmask bit
3. The last arriving subsystem (count == N) gives the semaphore (N-1) times to unblock all others
4. Uses `atomic_t sync_ready_count` and `atomic_t sync_ready_mask` for lock-free coordination

### Timeout Handling

If a subsystem does not arrive within `timeout_ms`, the barrier is released and the waiting subsystem enters an error state, powers down its sensor, and returns to idle. This prevents deadlocks if one sensor fails to initialize.

### Usage with Board Sync

The barrier integrates with inter-board synchronization:
- In **PRIMARY** mode: the primary board's sync GPIO is asserted when the barrier completes
- In **SECONDARY** mode: the secondary board waits for the primary's GPIO signal at the barrier before proceeding

## Inter-Board Synchronization (`board_sync`)

For multi-device setups, hardware GPIO synchronization aligns sampling across multiple BioGAP boards. This is essential for synchronized multi-subject recordings or high-channel-count configurations.

> **Note**: This feature has not been tested yet and is currently theoretical. The implementation is based on standard GPIO interrupt handling and periodic pulse generation for drift correction.

### Sync Roles (Kconfig)

| Role | Kconfig | Description |
|------|---------|-------------|
| STANDALONE | `CONFIG_BOARD_SYNC_ROLE_STANDALONE` | No sync, single device (default) |
| PRIMARY | `CONFIG_BOARD_SYNC_ROLE_PRIMARY` | Outputs sync signal on GPIO |
| SECONDARY | `CONFIG_BOARD_SYNC_ROLE_SECONDARY` | Waits for PRIMARY's GPIO signal |

### Board ID

Each board has a unique ID (1-255) set via `CONFIG_BOARD_ID`. The board ID is embedded in every BLE packet for source identification.

### Synchronization Mechanism


```
PRIMARY Board                              SECONDARY Board
─────────────                              ───────────────
sync_barrier completes ──► assert GPIO ──► GPIO ISR fires
                                              │
periodic timer ───────► pulse GPIO ─────────► drift correction
                                              │
                                          sync_sem given
                                          (proceed with acquisition)
```

### PRIMARY Behavior

1. When barrier completes, assert sync GPIO output
2. Periodic timer (configured via `CONFIG_BOARD_SYNC_PERIODIC_MS`, default 1000ms) pulses the GPIO for drift correction
3. Pulse counter incremented each time and embedded in BLE packets

### SECONDARY Behavior

1. At barrier, wait for `sync_sem` (given by GPIO ISR)
2. On receiving signal, proceed with acquisition
3. Periodic pulses from PRIMARY are counted for drift correction

### Key Functions

| Function | Description |
|----------|-------------|
| `board_sync_init()` | Configure sync GPIO based on role |
| `board_sync_signal()` | (PRIMARY) Assert sync GPIO |
| `board_sync_wait(timeout_ms)` | (SECONDARY) Wait for sync signal |
| `board_sync_get_pulse_count()` | Get current pulse counter |

### Pulse Counter

The pulse counter (`sync_pulse_count`) is embedded in the BLE packet metadata (byte 208 of EXG packets). This allows post-hoc temporal alignment of data from multiple boards.

### GPIO Configuration

The sync GPIO is defined in the device tree overlay (`nrf5340_senseiv1_cpuapp.overlay`), defaulting to P0.05 (commented out by default, must be enabled for multi-board setups).
