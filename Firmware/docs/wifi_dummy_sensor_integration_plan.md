# Integration plan: dummy-sensor streaming over WiFi (BioGUI ↔ ESP32 ↔ nRF5340)

Goal: a BioGUI TCP connection triggers `START_DUMMY_STREAMING` on the nRF, whose synthetic
211-byte packets flow nRF → SPI → ESP32 → TCP → BioGUI, and can be stopped/restarted cleanly.

## Where things stand

- **nRF5340 (`Firmware/src_NRF`)**: fully ready. Shared SPI_A bus, handshake (0xA5/0x5A),
  header/tailer framing (0x66/0xBB for control frames, 0x55/0xAA for the dummy sensor's own
  211-byte packets), `command_dispatcher.c` already handles opcodes 243/244
  (`START_DUMMY_STREAMING`/`STOP_DUMMY_STREAMING`), `dummy_sensor_appl.c` already produces
  correctly-framed packets and sends them via `add_data_to_esp_send_buffer()`.
- **ESP32-C6 (`WiFi_SD_Shield_V1/Firmware/espc6`)**: SoftAP + TCP server (port 4444, SSID
  `bioserver`), SPI slave bus/GPIO init, GUI command parser, and the SPI read/send task
  bodies are all *written* and byte-compatible with the nRF side. But:
  1. The block in `main/main.c` that creates the ring buffer, calls `bind_to_gui()`, and
     spawns the four data-plane tasks is entirely commented out — nothing runs after the
     boot-time handshake today.
  2. `tx_to_gui()` (`gui_task.c`) has its `send()` call commented out and replaced with a
     hardcoded `int sent = 1;` — it would silently discard every packet instead of relaying
     it to BioGUI.
  3. `bind_to_gui()` does one blocking `accept()` with no reconnect loop, and its
     disconnect-cleanup path is commented out.
  4. `biogap_read.c`'s malformed-packet path calls `break` (kills the whole read task) rather
     than logging and continuing.
  5. There's an **uncommitted local edit** in `main.c` that already re-enables
     `wifi_init_softap()` — keep it, build on top of it.

## Step 1 — Confirm the boot-time SPI handshake still works

Nothing to re-enable yet; this just validates the baseline before touching the commented block.

- nRF: in `Firmware/src_NRF/prj.conf`, set `CONFIG_WI_FI=y` and `CONFIG_DUMMY_SENSOR=y`
  (currently `CONFIG_WI_FI=n`). Rebuild, flash.
- ESP32: build and flash as-is (working tree, with your uncommitted `wifi_init_softap()` fix).
- **Done when:** nRF RTT log shows the handshake attempt loop succeeding (`Initial handshake
  with ESP32 successful`) and ESP32's `idf.py monitor` log shows its handshake success path in
  `initial_handshake_nrf_master_esp_slave_pq()`. If this doesn't succeed, don't proceed —
  everything downstream depends on this SPI link being physically sound (check MOSI/MISO/SCLK/CS
  wiring and the DRDY GPIO edge behavior first).

## Step 2 — Enable the ESP32 data-plane tasks, GUI → nRF direction only

- In `main/main.c`, un-comment: ring buffer creation, `bind_to_gui()`, and the two SPI tasks
  (`read_biogap_task_nrf_master`, `send_to_biogap_task_nrf_master`) and the two GUI tasks
  (`rx_from_gui`, `tx_to_gui`). Leave `sd_card_task` commented (or gated off via
  `ESP_ENABLE_SD_WRITE=0`, already the default) — not needed for this goal and it competes for
  the same ring buffer as `tx_to_gui`.
- Do **not** fix `tx_to_gui()`'s stub yet — isolate the command path first.
- From BioGUI: use the **TCP client** data source (the one that dials out — `192.168.4.1:4444`,
  ESP32's SoftAP gateway address and `PORT_LAPTOP`), select the `biogapultra_dummy` interface,
  and hit start.
- **Done when:**
  - ESP log shows `accept()` succeeding and `B_GUI_SOCKET_BIND`/`gui_connected=true`.
  - ESP log shows `rx_from_gui` receiving the `START_DUMMY_STREAMING` (243) byte and
    `parse_gui_command()` setting `B_START_CMD_RCV`.
  - ESP log shows `send_to_biogap_task` pulsing DRDY and sending the `[0x66, 0xF3, 0x00, 0xBB]`
    control frame.
  - nRF log shows the DRDY interrupt firing, `process_esp_data()` validating the frame, and
    `command_dispatcher` logging `START_DUMMY_STREAMING` → `dummy_sensor_start_streaming()`.
  - ESP log shows `read_biogap_task_nrf_master` receiving valid 211-byte (`0x55`/`0xAA`) packets
    from the nRF and pushing them into `biogap_ringbuf` (packet counter incrementing). BioGUI
    itself won't show anything yet — that's Step 3.

## Step 3 — Fix `tx_to_gui()`, confirm data reaches BioGUI

- Restore the real `send(gui_sock, item, item_size, 0)` call in `gui_task.c`, remove the
  `int sent = 1;` placeholder. Keep whatever partial-send/retry handling makes sense for a
  blocking `SOCK_STREAM` socket (loop until all `item_size` bytes are sent, or bail out to the
  disconnect path on error).
- **Done when:** BioGUI's `biogapultra_dummy` plot shows the synthetic ramp waveform
  live-updating, and the counter/timestamp channels increment consistently (no gaps beyond
  what's expected from real packet loss).

## Step 4 — Reconnect / disconnect handling

- Wrap `bind_to_gui()`'s `accept()` in a loop so a second BioGUI connection is possible after
  the first one closes (rather than requiring an ESP32 power cycle between test runs).
- Restore the commented-out disconnect cleanup in `rx_from_gui()` (`shutdown`/`close` on
  `gui_sock`, reset `gui_connected`/`gui_sock`, reset `node_state` back to `STATE_IDLE`) instead
  of the current unconditional `vTaskDelete(NULL)` on `recv() == 0`.
- **Done when:** you can stop streaming from BioGUI, disconnect, reconnect, and start streaming
  again without re-flashing or power-cycling the ESP32.

## Step 5 — Harden the SPI read path

- In `biogap_read.c`, replace the `break` on a malformed/misframed packet with a log line plus
  `continue` (drop the bad packet, keep the task alive) — a single corrupted SPI transaction
  shouldn't take down the whole read task during a long streaming session.
- **Done when:** you can run a sustained streaming session (minutes, not seconds) without the
  ESP32 read task silently dying.

## Step 6 — Cleanup (optional, do last, doesn't block functionality)

- Rename/fix the misleading `IS_ESP_SPI_MASTER` macro and the "ESP is configured as SPI MASTER"
  log line (ESP is actually the SPI *slave* throughout).
- Remove genuinely dead code: `tcp_server_task()` in `softap_main.c` (never called), the
  `accept_nodes_task` comment-only reference and unused `PORT_ESP_NODE=3333` (both belong to an
  unimplemented multi-node "WBAN" mode, `IS_WBAN=0`), and the plain (non-prequeue)
  `read_from_biogap_task_nrf_master_esp_slave` declared but never defined.
- Commit the local `wifi_init_softap()` re-enable once the rest of the flow is validated, so it
  isn't sitting only in your working tree.

## Suggested verification order

Do the steps strictly in order and don't move to the next until the current step's "done when"
criteria are met on real hardware — each one isolates a specific link in the chain (SPI link →
GUI-to-nRF command path → nRF-to-GUI data path → connection lifecycle → robustness), so if
something breaks later you'll know which link to look at first.
