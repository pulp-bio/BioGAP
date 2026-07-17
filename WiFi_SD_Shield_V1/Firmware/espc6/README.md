# WiFi/SD Shield Firmware (ESP32-C6)

ESP-IDF firmware for the BIOGAP Ultra WiFi/SD shield (ESP32-C6-MINI-1-H8). The ESP32
bridges the nRF5340 BIOGAP master to a laptop running BioGUI over WiFi: it terminates a
TCP connection from BioGUI, forwards START/STOP commands to the nRF over a shared SPI
link, and relays sensor data packets back to BioGUI. The nRF5340 is always the SPI
master; the ESP32 is always the SPI slave.

## Build / flash

Standard ESP-IDF project, target `esp32c6`.

```
idf.py set-target esp32c6      # first time only
idf.py build
idf.py -p <PORT> flash monitor
```

Project name: `wifi_sd_shield_v0_ap` (see [CMakeLists.txt](CMakeLists.txt)).

## Component layout

```
main/                       app_main(): boot sequence, task creation
components/common/          shared config, pin definitions, SPI/GPIO bring-up
components/biogap/          nRF <-> ESP32 SPI link (handshake, read, send, dummy-sensor test path)
components/wifi_ap/         WiFi SoftAP + TCP server + GUI command/data tasks
components/sdcard_spi/      SD card writer (currently disabled, see Config flags)
components/led_strip/       status LED
components/arduino/         vendored Arduino-as-component (used for led_strip only)
```

## Code workflow

### 1. Boot (`main/main.c`)

1. Configure a debug GPIO (`RTC_SCL`), set logging level.
2. Create the global event group `g_evt` (see Event bits below) and the SPI bus mutex.
3. `config_spi_nrf_master_esp_slave_pins()` — configure all NRF<->ESP GPIOs (data pins,
   DRDY, direction-control pins, see Pin reference below).
4. `init_nrf_spi_master_esp_slave_bus()` — bring up the SPI slave peripheral
   (`SPI2_HOST`, mode 1, DMA).
5. Block in a retry loop on `initial_handshake_nrf_master_esp_slave_pq()` until the nRF
   master completes the 4-byte handshake (see Handshake below). Sets `B_BIOGAP_CONECTED`.
6. `wifi_init_softap()` — bring up WiFi SoftAP (`bioserver` / see `EXAMPLE_ESP_WIFI_PASS`
   in `softap_main.h`), sets `B_WIFI_CONNECTED`.
7. Create the BIOGAP ring buffer (`biogap_ringbuf`, `RINGBUFF_SIZE` bytes).
8. `bind_to_gui()` — blocking TCP `accept()` on port `PORT_LAPTOP` (4444). Sets
   `B_GUI_SOCKET_BIND`; `node_state = STATE_IDLE`.
9. Spawn the four data-plane tasks (or three + the local generator, depending on
   `ESP_LOCAL_DUMMY_SENSOR`, see Config flags):
   - `read_from_biogap_task_nrf_master_esp_slave_prequeue` — SPI slave RX, nRF -> ESP.
   - `send_to_biogap_task_nrf_master_esp_slave` — SPI slave TX / command forwarding, ESP -> nRF.
   - `rx_from_gui` — TCP RX, BioGUI -> ESP (commands).
   - `tx_to_gui` — TCP TX, ESP -> BioGUI (data).

The main task then just idles forever; all real work happens in the tasks above.

### 2. Handshake (`biogap_common.c`)

`initial_handshake_nrf_master_esp_slave_pq()` arms one 4-byte SPI slave transaction of
`HANDSHAKE_MARKER` (`0x5A`) bytes and blocks until the nRF (master) clocks it out and
clocks back `HANDSHAKE_RESPONSE_MARKER` (`0xA5`) bytes. On a matching response it sets
`B_BIOGAP_CONECTED` and `handshake_pq_done = true`. Every other task that talks to the
nRF blocks on `B_BIOGAP_CONECTED` before doing anything.

### 3. GUI -> nRF command path (START)

1. BioGUI opens a TCP connection to `192.168.4.1:4444` (the ESP's SoftAP gateway
   address) and sends a single command byte (see `connectivity_commands.h`).
2. `rx_from_gui()` receives it and calls `parse_gui_command()` (`gui_utils.c`), which
   validates the opcode via `validate_command()` and, in `STATE_IDLE`, accepts only
   START-type commands: sets `rx_gui_data_to_fwd[0]` and `B_START_CMD_RCV`.
3. `send_to_biogap_task_nrf_master_esp_slave()` wakes on `B_START_CMD_RCV`, pulses the
   DRDY GPIO (`data_ready_pulse()`, 100 us high pulse via `NRF_ESP_DATA_READY`) to tell
   the nRF "I have something for you", then blocks in
   `send_first_start_command_to_biogap_master()` on `spi_slave_transmit()` of a 4-byte
   control frame (`ESP_SPI_HEADER 0x66, <opcode>, 0x00, ESP_SPI_TAILER 0xBB`) until the
   nRF's DRDY interrupt fires and it initiates the matching master transaction.
4. On success: `B_START_CMD_FWD_TO_BIOGAP` set, `node_state = STATE_STREAMING`.

### 4. nRF -> GUI data path (streaming)

- `read_from_biogap_task_nrf_master_esp_slave_prequeue()` (`biogap_read.c`) keeps
  `QUEUE_COUNT` (4) DMA-capable SPI slave transactions pre-queued at all times so the
  slave is never "unarmed" between nRF-initiated transactions (see the pre-queue
  pattern doc comment at the top of that file, and `notes.md` for the throughput
  numbers that motivated it). Each completed transaction is validated
  (`NRF_EXG_HEADER 0x55` / `NRF_EXG_TAILER 0xAA`, `NRF_EXG_PACKET_SIZE` = 211 bytes),
  pushed into `biogap_ringbuf` via `add_to_ringbuffer()`, and its descriptor is
  immediately re-queued.
- `tx_to_gui()` (`gui_task.c`) drains `biogap_ringbuf` and `send()`s each item straight
  to `gui_sock` whenever `node_state == STATE_STREAMING`.

### 5. STOP path

The ESP (slave) cannot push data to the nRF (master) unilaterally, so STOP is delivered
by *piggybacking* on the next regular data-transaction response instead of a dedicated
transfer:

1. GUI sends a STOP-type opcode while `node_state == STATE_STREAMING`;
   `parse_gui_command()` sets `B_STOP_CMD_RPT_PENDING`.
2. `rx_from_gui()` promotes this to `B_STOP_CMD_RCV_GUI`, then calls
   `prepare_for_restart()` (or `prepare_for_restart_local_dummy()` in local-dummy mode).
3. `read_from_biogap_task_nrf_master_esp_slave_prequeue()` sees `B_STOP_CMD_RCV_GUI` (or
   `B_STOP_CMD_RCV_FORCED`, set instead if the ring buffer overflows) and calls
   `enter_stop_quiesce_state()`: under the SPI bus lock, it stomps `sendbuf_persistent`
   and every pre-queued `tx_bufs[i]` with a STOP frame
   (`ESP_EXG_HEADER, _, ESP_STOP_COMMAND, <opcode>` at `PACKET_SZ` bytes with
   `ESP_EXG_TAILER`), drains the in-flight transaction, then sets `B_SPI_QUIESCED` and
   `node_state = STATE_IDLE`.
4. `send_to_biogap_task_nrf_master_esp_slave()` wakes on `B_SPI_QUIESCED` and sets
   `B_STOP_CMD_FWD_TO_BIOGAP`.
5. `prepare_for_restart()` (blocked on `B_STOP_CMD_FWD_TO_BIOGAP`) frees the pre-queue
   DMA buffers, re-initializes the SPI slave bus, soft-flushes `biogap_ringbuf`
   (`rb_soft_flush()`), re-allocates pre-queue resources, and clears the STOP/START
   event bits — leaving the system idle and ready for the next START from BioGUI.

### State machine (`common.h`)

`node_state_t`: `STATE_DISCONNECTED` -> `STATE_IDLE` (handshake + GUI socket bound) ->
`STATE_STREAMING` (START forwarded) -> back to `STATE_IDLE` on STOP.

### Event bits (`g_evt`, see `common.h` for the full list)

Key bits: `B_BIOGAP_CONECTED`, `B_WIFI_CONNECTED`, `B_GUI_SOCKET_BIND`,
`B_START_CMD_RCV`, `B_START_CMD_FWD_TO_BIOGAP`, `B_STOP_CMD_RCV_GUI`,
`B_STOP_CMD_RCV_FORCED`, `B_STOP_CMD_FWD_TO_BIOGAP`, `B_SPI_QUIESCED`,
`B_STOP_CMD_RPT_PENDING`, `B_RINGBUFFER_FULL`, `B_WRITING_TO_SD`.

### Framing reference

| Frame type | Header | Tailer | Size | Notes |
|---|---|---|---|---|
| ESP -> nRF control (commands) | `0x66` (`ESP_SPI_HEADER`) | `0xBB` (`ESP_SPI_TAILER`) | 4 bytes | opcode at byte[1] (START) or STOP-piggyback at byte[2]/[3] |
| nRF -> ESP data | `0x55` (`NRF_EXG_HEADER`) | `0xAA` (`NRF_EXG_TAILER`) | 211 bytes (`NRF_EXG_PACKET_SIZE`) | real ExG and dummy-sensor packets share this format |
| Handshake (ESP TX) | `0x5A` x4 (`HANDSHAKE_MARKER`) | - | 4 bytes | expects `0xA5` x4 (`HANDSHAKE_RESPONSE_MARKER`) echoed back |

Command opcodes are in `connectivity_commands.h` and must stay in sync with
`Firmware/src_NRF/core/connectivity_commands.h` on the nRF side.

## Pin reference (`components/common/include/pin_definitions.h`)

### NRF <-> ESP SPI (ESP is always SPI slave)

| Signal | GPIO | Direction (ESP side) | Notes |
|---|---|---|---|
| `NRF_ESP_MOSI` | 23 | input | SPI MOSI, level-translated via IC2 (`SN74AVC4T774RSVR`) |
| `NRF_ESP_MISO` | 19 | output | SPI MISO, via IC2 |
| `NRF_ESP_SCLK` | 20 | input | SPI clock, via IC2 |
| `NRF_ESP_CS`   | 21 | input | SPI chip-select, via IC2 |
| `NRF_ESP_DIR_CTRL` | 12 | output, driven **LOW** | Direction control for IC2's MOSI/MISO/CS/CLK channels. LOW = NRF is master (our only mode). An inverter (IC7, `SN74AUP1G04DBVR`) derives the complementary polarity needed for the MISO channel from this same control bit. |
| `NRF_ESP_DRDY_DIR_CTRL` | 8 | output, driven **HIGH**, fixed | Direction control for IC5's (`SN74AVC2T245RSWR`) dedicated DRDY channel. DRDY always flows ESP -> NRF regardless of SPI master role, so this is permanently HIGH (never toggled at runtime), unlike `NRF_ESP_DIR_CTRL`. |
| `NRF_ESP_DATA_READY` | 22 | output | DRDY line itself (-> nRF's `SPI_NRF_ESP_DRDY` / QSPI_IO2). Pulsed high for ~100 us (`NRF_DRDY_PULSE_US`) by `data_ready_pulse()` whenever the ESP has a command frame ready for the nRF to pick up. |

Both `NRF_ESP_DIR_CTRL` and `NRF_ESP_DRDY_DIR_CTRL` are configured and driven once at
boot in `config_spi_nrf_master_esp_slave_pins()` (`common.c`) and never change again at
runtime — the shield's bus direction is fixed for the "nRF is SPI master" topology this
firmware implements.

### SD card SPI (only when ESP acts as SPI master to the SD card directly; currently disabled)

| Signal | GPIO | Notes |
|---|---|---|
| `ESP_SD_MOSI` | 7 | -> SD `mSD_CMD`/CMD |
| `ESP_SD_MISO` | 2 | <- SD `mSD_DAT0`/DAT0 |
| `ESP_SD_CLK`  | 6 | SD clock |
| `ESP_SD_CS`   | 18 | -> SD `mSD_DAT3`/CS (SPI mode) |
| `ESP_SDCTRL`  | 9 | Controls whether ESP writes directly to the SD card |

### RTC

| Signal | GPIO | Notes |
|---|---|---|
| `RTC_SCL` | 15 | Also reused as a general debug/status GPIO in `main.c` |
| `RTC_SDA` | 14 | |
| `RTC_ON`  | 13 | Enable/disable the RTC |
| `RTC_FOUT` | 4 | |
| `RTC_CLKOUT` | 3 | |

## Config flags (`components/common/include/shield_config.h`)

| Flag | Default | Meaning |
|---|---|---|
| `IS_WBAN` | 0 | Multi-node WBAN accept-loop mode (not yet implemented, `main.c` has a stubbed-out call site). |
| `IS_ESP_SPI_SLAVE` | 1 | ESP is always the SPI slave, nRF the master. No other mode is currently supported end-to-end. |
| `ESP_ENABLE_SD_WRITE` | 0 | Enable the SD-card write task (`sd_card_task`), competes with `tx_to_gui()` for the same ring buffer — leave off unless actively testing SD write. |
| `ESP_LOCAL_DUMMY_SENSOR` | 0 | When 1, bypasses SPI/nRF entirely: the ESP generates synthetic dummy-sensor packets locally (`dummy_sensor_local.c`, byte-identical wire format to the nRF's `dummy_sensor_appl.c`) and streams them straight to BioGUI. Useful for testing the WiFi/GUI half in isolation with no nRF/SPI hardware attached. When 0 (real integration), the SPI/handshake bring-up in `main.c` runs and the real SPI read/send tasks are created instead. |

## WiFi / TCP

- SoftAP SSID: `bioserver` (`EXAMPLE_ESP_WIFI_SSID`, `softap_main.h`), password
  `EXAMPLE_ESP_WIFI_PASS`, channel `EXAMPLE_ESP_WIFI_CHANNEL`.
- BioGUI connects as a TCP client to `192.168.4.1:4444` (`PORT_LAPTOP`), the ESP's
  default SoftAP gateway address. Single connection only (`accept()` is called once;
  see Known limitations).
- `PORT_ESP_NODE` (3333) is reserved for the unimplemented `IS_WBAN` multi-node mode.

## Known limitations / not yet implemented

- `bind_to_gui()` accepts exactly one TCP connection with no reconnect loop; a second
  BioGUI connection after the first closes currently requires a power cycle.
- The malformed/misframed-packet path in `biogap_read.c`'s main streaming loop calls
  `break` (kills the read task) rather than logging and continuing — a single corrupted
  SPI transaction can currently take down streaming.
- `IS_WBAN` multi-node mode is not implemented (`accept_nodes_task` is only a comment).
- `tcp_server_task()` in `softap_main.c` is dead code (never spawned).

See `Firmware/docs/wifi_dummy_sensor_integration_plan.md` for the step-by-step
integration/verification plan this firmware was brought up against.
