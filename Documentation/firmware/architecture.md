# Architecture Overview

The BioGAP firmware is a multi-threaded sensor acquisition and BLE streaming system running on the **nRF5340** SoC, built on **Zephyr RTOS**.

## Dual-Core Architecture

The nRF5340 is a dual-core processor:

- **Application Core** — Runs the main firmware: sensor drivers, BLE application layer, and power management
- **Network Core** — Runs the BLE radio controller transparently; the application communicates with it via Zephyr's HCI IPC driver

## Initialization

On boot, the firmware initializes hardware in this order:

1. **Power** — Configure PMIC (MAX77654) power rails and enable battery charging
2. **Console** — Enable USB CDC ACM serial console
3. **AFE** — Configure ADS1298 SPI bus and Data Ready (DRDY) interrupt
4. **GAP9** — Power on the GAP9 co-processor
5. **BLE** — Start the BLE stack
6. **Sensors** — Initialize IMU (I2C), microphone (PDM), and AFE (EEG or EMG)
7. **Board Sync** — Configure inter-board GPIO sync (if enabled)

After initialization, the main thread sleeps and monitors the soft-reset button.

## Threading Model

Each sensor and subsystem runs in its own Zephyr thread. This allows sensors to operate independently and in parallel:

- **Sensor threads** — One per active sensor (EEG/EMG, IMU, Microphone). Each waits for a start signal, then continuously acquires data and enqueues BLE packets.
- **BLE send thread** — Dequeues packets from all sensors and transmits them over BLE.
- **BLE receive thread** — Processes incoming BLE commands (start/stop streaming, query status, etc.).
- **Battery thread** — Periodically reads battery status from the PMIC.

## Data Flow

### Sensor Streaming

Each sensor thread:
1. Waits for a start command (semaphore from BLE receive thread)
2. Powers up the sensor and configures it
3. Participates in barrier synchronization (if multi-sensor start)
4. Loops: reads samples, fills a BLE packet, enqueues it for transmission
5. On stop command: powers down and returns to idle

## Barrier Synchronization

When multiple sensors are started together, a barrier ensures they all begin at the same instant:

1. The barrier is set up for N subsystems
2. Each sensor thread registers and blocks at the barrier
3. When all N have arrived, they are all released simultaneously

This guarantees temporal alignment across different sensor modalities.
