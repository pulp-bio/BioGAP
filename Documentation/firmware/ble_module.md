# BLE Module

The BLE module (`src_NRF/ble/`) manages all Bluetooth Low Energy connectivity using the **Nordic UART Service (NUS)** for bidirectional data transfer. It is optimized for high-throughput sensor data streaming while supporting a command/response protocol for device control.

## File Overview

| File | Purpose |
|------|---------|
| `bluetooth.c/h` | BLE stack init, NUS service, connection callbacks, PHY/MTU negotiation, statistics |
| `ble_appl.c/h` | Application layer: send/receive threads, message queues, command dispatcher |
| `ble_commands.h` | Protocol command codes (see [BLE Protocol](./ble_protocol.md)) |

## Threading

### Send Thread (`ble_send_tid`)

- **Stack**: 2048 bytes
- Loops on `k_msgq_get(&send_msgq, ...)` with 100ms timeout
- Calls `send_data_ble()` → `bt_nus_send()` for each packet
- Tracks packet statistics by header byte

### Receive Thread (`ble_receive_tid`)

- **Stack**: 2048 bytes
- Waits on `ble_data_received` semaphore (signaled by NUS RX callback)
- Reads from `receive_msgq` and dispatches commands via `process_ble_rx_data()`

### Write Thread (`ble_write_thread_id`)

- **Stack**: 1024 bytes
- Waits for BLE initialization (`ble_init_ok` semaphore)
- Used for connection establishment monitoring

## Message Queues

### Send Queue (`send_msgq`)

```
K_MSGQ_DEFINE(send_msgq, sizeof(ble_packet_t), 64, 4)
```

- 64 entries of `ble_packet_t` (variable-size, max 244 bytes data)
- 4-byte alignment
- Filled by sensor streaming threads, drained by BLE send thread

### Receive Queue (`receive_msgq`)

```
K_MSGQ_DEFINE(receive_msgq, BLE_PCKT_RECEIVE_SIZE, 16, 1)
```

- 16 entries of 234 bytes each
- Filled by NUS RX callback, drained by BLE receive thread

## Public API

| Function | Description |
|----------|-------------|
| `send_data_ble(data, len)` | Send raw bytes over NUS (TX notify) |
| `add_data_to_send_buffer(pkt, len)` | Enqueue a packet for async BLE transmission |
| `get_ble_eeg_packets_sent()` | Get count of sent EEG packets |
| `get_ble_imu_packets_sent()` | Get count of sent IMU packets |
| `get_ble_mic_packets_sent()` | Get count of sent MIC packets |
| `get_ble_other_packets_sent()` | Get count of other sent packets |
| `get_ble_packets_failed()` | Get count of failed transmissions |
| `send_battery_status()` | Request battery status transmission |