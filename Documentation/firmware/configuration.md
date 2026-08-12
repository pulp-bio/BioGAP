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

### mmWave Radar

The radar is enabled like any other sensor, from `prj.conf`:

```
CONFIG_SENSOR_MMWAVE=y
```

and a normal build picks it up:

```bash
west build -b nrf5340_senseiv1_cpuapp
```

The shield also needs a devicetree overlay and a shield definition, and CMake
resolves both *before* Kconfig runs — so `CMakeLists.txt` scans `prj.conf` for
that option and pulls in `SENSEI_mmWaveShield` plus `overlays/mmwave.overlay` to
match. A commented-out line does not count. `-DMMWAVE_SHIELD=ON` forces the same
thing without editing `prj.conf`, mainly for CI.

**Compiling the radar in does not power it.** The shield's rail and the radar
stay off until the host sends `TURN_ON_MMWAVE` (opcode 48), so which sensors
actually run is decided by the start command the GUI sends — an image with the
radar compiled in behaves like one without it until radar data is requested.

For a radar-only image, which swaps the SPI transport and gives SPIM4 to the
Zephyr driver:

```bash
west build -b nrf5340_senseiv1_cpuapp -- -DMMWAVE_ONLY=ON
```

That merges `overlays/mmwave_only.conf` instead of `overlays/mmwave.conf`, and
**ExG does not work in it**.

> **Radar sub-options go in `overlays/mmwave.conf`, not `prj.conf`.** They sit
> behind `if SENSOR_MMWAVE` in `Kconfig`, so an assignment placed above the line
> that enables it is evaluated while its dependency is still unmet and Kconfig
> drops it with a warning. The fragment is merged after `prj.conf`, so it is
> always a safe place for them.

### Choosing a transport

| | `CONFIG_SENSOR_MMWAVE=y` (default) | `-DMMWAVE_ONLY=ON` |
|---|---|---|
| Transport | raw nrfx (`mmWave_spi.c`) | Zephyr SPI (`mmWave_spi_zephyr.c`) |
| SPIM4 owner | BioGAP's `spi/spi_a.c`, shared | Zephyr's SPI driver, exclusive |
| Bus config | switched per transaction | set once from the devicetree |
| ExG (ADS1298) | works | **does not work** |
| WULPUS | works | disabled (SPIM2 also goes to Zephyr) |
| Microphone | disabled (pin conflict) | disabled (pin conflict) |

`CONFIG_SENSOR_MMWAVE=y` is the normal choice: ExG and the radar acquire
together, and BioGUI's `biogapultra_eeg_mmwave` interface streams both over one
BLE link. Selecting a radar-free interface in the GUI simply never powers the
radar, so the same image serves ExG-only sessions.

`-DMMWAVE_ONLY=ON` reproduces the transport of the standalone BGT60TR13C
firmware, i.e. the arrangement validated on this hardware. Use it to bring the
radar up in isolation, or when ExG is not needed. In that image
`init_spi_a_bus()` is a no-op, so the ADS1298 cannot be driven — the sources
still compile and link (other unconditionally built files reference the ExG
shield's devicetree nodes), but ExG start commands (opcodes 18/19/37/38) must
not be sent.

### Sharing SPI_A with the ADS1298

In the combined image the radar, the ADS1298 and the Wi-Fi/SD shield all sit on
SPI_A, and only one SPI dialect can be active at a time. The radar's chip-select
callback doubles as the bus lock: it takes `spi_a_mutex`, waits for any in-flight
transfer to finish, captures the current configuration with
`spi_a_save_config()`, switches the peripheral to mode 0, and puts the saved
configuration back on release. Every other consumer takes the same mutex, so they
all serialise cleanly.

It saves and restores rather than resetting to a fixed idle setting because the
clock on SPI_A belongs to whichever consumer is active: the ADS1298 runs at
`SPI_A_ADS_CMD_SAFE_FREQ_HZ` (4 MHz) while issuing commands and
`SPI_A_ADS_STREAMING_FREQ_HZ` (8 MHz) once in RDATAC, and the ESP32 switches to
`SPI_A_ESP_STREAMING_FREQ_HZ` (32 MHz) for the duration of each transfer.
Restoring a compile-time constant would silently reclock somebody else's session.

Two mechanisms coexist deliberately: `spi_a_set_frequency()` tears the peripheral
down and re-initialises it, which suits a change that lasts a whole acquisition
phase, while `spi_a_reconfigure()` rewrites only the CONFIG and FREQUENCY
registers, which is cheap enough to do per transaction — as the radar must, since
it is the only consumer that needs a different SPI *mode*.

The cost is bus-hold latency. A radar FIFO read is ~390 bytes at 8 MHz, so the
radar owns the bus for roughly 400 µs once per frame. At the default 100 fps
that is about 4% of the time, against the ADS1298's 2 ms sampling period — so an
ExG transfer occasionally waits, but well inside its budget. Lowering
`CONFIG_MMWAVE_SPI_FREQ_MHZ` or raising the radar frame rate both eat into that
margin.

> Radar bring-up in the combined image only configures **VD2**, via
> `mmwave_shield_power_on()`. The radar-only image additionally runs
> `pwr_bsp_start()`, which brings up VD0/VD1/VA0 as well. If the radar
> identifies itself in the radar-only image but not in the combined one, the
> shield needs a rail beyond VD2 and that is the first thing to extend.

Radar sub-options, available in either fragment:

| Option | Type | Default | Description |
|--------|------|---------|-------------|
| `CONFIG_SENSOR_MMWAVE` | bool | n | Enable the BGT60TR13C radar subsystem |
| `CONFIG_MMWAVE_CONF_100FPS_32C_8S` | bool | y | Register profile: 100 fps, 32 chirps x 8 samples |
| `CONFIG_MMWAVE_CONF_{25,50,100,150,200}FPS` | bool | n | Alternative frame rates, same geometry |
| `CONFIG_MMWAVE_CONF_STATIC_DISTANCE` | bool | n | 5 fps, 32 chirps x 64 samples, for distance validation |
| `CONFIG_MMWAVE_SEND_PACKED_12BIT` | bool | y | Stream samples in the radar's native 12-bit packing |
| `CONFIG_MMWAVE_TEST_PATTERN` | bool | n | Validate the FIFO against the radar's internal LFSR pattern |
| `CONFIG_MMWAVE_EXT_SYNC` | bool | n | Square-wave sync output for an external reference device |
| `CONFIG_MMWAVE_ZEPHYR_SPI` | bool | n | Use the Zephyr SPI subsystem; radar owns SPIM4, ExG stops working |
| `CONFIG_MMWAVE_SPI_FREQ_MHZ` | int | 8 | Radar SPI clock (1, 2, 4 or 8) |
| `CONFIG_MMWAVE_SPI_STATIC_MODE` | bool | n | Diagnostic: never switch the shared bus out of the radar's mode |
| `CONFIG_MMWAVE_PROBE_LOOP` | bool | n | Diagnostic: repeat the CHIP_ID read after a failed bring-up |

### Bring-up diagnostics

If the radar fails to identify itself, `mmWave_power_on()` logs `BGT60 init
error: 2` (`XENSIV_BGT60TRXX_STATUS_DEV_ERROR`) — the CHIP_ID read returned a
value the driver does not recognise. Both ID fields must read 3 for a
BGT60TR13C. The failure path then prints the raw ID, re-reads it at 4 MHz, and
dumps the SPI peripheral state captured *at the instant the last transfer
completed* (sampling it later is useless, because the chip-select callback has by
then restored the ADS1298's settings).

`RXD.AMOUNT`/`TXD.AMOUNT` in that line are the decisive numbers: non-zero counts
with an all-zero result mean the bus clocked correctly and the radar did not
drive MISO, which points at the shield rather than the firmware.

### Frame losses while streaming

A lost frame is logged and counted rather than ignored. Two things can cost one:

```
FIFO read failed at frame 4213: FSTAT 0x8003ff overflow
IRQ timeout at frame 4213 - no radar data (irq line 0)
```

The first means the burst read was refused. The BGT60TR13C reports every such
refusal through one status code, so the firmware re-reads `FSTAT` and names the
latched bit: `overflow`/`underflow` mean the reads did not keep up with the
frame rate, while `spi-burst`/`clk-num` point at the bus. The second means no
frame became available within `MMWAVE_IRQ_TIMEOUT_MS`; `irq line 0` confirms the
radar was genuinely idle rather than the interrupt having been missed — see
below for why that distinction exists.

#### The battery-ADC pin conflict (a fixed two-minute stall)

The radar used to stop dead about two minutes after boot, with `CHIP_ID
0x000000 NO ANSWER` and the data-ready line low, and no capture restart could
revive it.

The battery-monitor ADC input is `AIN3` = **P0.07**, which is also the mmWave
shield's power-enable. `mmWave_power_on()` disconnects the SAADC input and drives
that pin high as a GPIO, and `pwr_set_measurement_gate()` keeps the power thread
from measuring while the radar is up. But that gate only suppresses the SDK power
thread's *full* cycles. While it vetoes, the thread deliberately still runs
reduced **quiet cycles** — on the reasoning that AMUX/SAADC measurements are
electrically clean — once `THREAD_PWR_QUIET_UPDATE_PERIOD_MS` (120 s) has elapsed
since the last full measurement.

That reasoning holds only while the ADC pin belongs to the ADC. Zephyr's SAADC
driver re-applies the channel input on every read, which switches the pad to
analog mode and **drops the GPIO drive holding the shield on** — so the quiet
cycle powered the radar off in the middle of a stream.

The timing is what identified it. Across three runs the failure tracked uptime,
not frames or streaming time:

| Run | failed at | frames | fps |
|---|---|---|---|
| 1 | 00:02:21.49 | 11540 | 99.1 |
| 2 | 00:02:01.69 | 6088 | 98.4 |
| 3 | 00:02:01.66 | 2707 | 24.7 |

Runs 2 and 3 died at the same uptime to within 30 ms with different frame counts
and a 4× difference in frame rate; run 1 exactly one 20 s
`THREAD_PWR_UPDATE_PERIOD_MS` wakeup later, because a full cycle had run at 21.5 s
and reset the 120 s countdown. That is the quiet-cycle deadline sampled on the
20 s wakeup grid.

The fix adds `pwr_set_adc_gate()` to the SDK — a veto separate from the
measurement gate, which suppresses **any** path touching the battery ADC,
including quiet cycles. It is default-permissive, so applications that never
register one are unaffected. BioGAP registers a radar-only predicate, kept
deliberately independent of the ADS check so ExG sessions keep their quiet-cycle
telemetry.

> Any board where the battery-monitor input is multiplexed with an
> application-driven output needs this gate, not just the measurement gate.

#### Telling a stopped radar from a silent link

Every write-then-poll helper in the vendor driver reads an all-zero response as
a bit that has cleared. So on a link that returns nothing, `soft_reset()` sees
its reset bit "clear" on the first poll and `start_frame()` writes FRAME_START
into a value it read as zero — **both report success while nothing happened**.
Their return codes therefore say nothing about whether the radar is alive, and a
recovery that logs no errors is not evidence that it worked.

`mmwave_probe_link()` reads the one cheap register with a known non-zero answer:

```
Link probe (first loss): CHIP_ID 0x000303 radar answering, MAIN 0x000000, FSTAT 0x100000
Link probe (first loss): CHIP_ID 0x000000 NO ANSWER, MAIN 0x000000, FSTAT 0x000000
```

Both CHIP_ID fields must read 3. The first line means the radar is answering, so
its chirp FSM stopped while SPI is healthy — look at the device: `MAIN` shows
whether FRAME_START is still set, `FSTAT` whether the FIFO reports itself empty
(`0x100000` = EMPTY). The second means the link went silent, and the radar's own
state is unknown — look at the bus, the shield supply, or SPI_A arbitration.
An all-zero `FSTAT` is itself impossible on a healthy link, since an empty FIFO
must set its EMPTY flag; seeing it corroborates a silent link.

The probe runs at the first lost frame, before any reset disturbs the evidence,
and again before and after each restart.

Recovery escalates accordingly. The first attempt resets the FIFO only. From the
**second** attempt the whole register profile is rewritten via
`xensiv_bgt60trxx_config()`, which begins with a full software reset of the
device, then reapplies the host's runtime IF gain / TX power / frame rate on top
— since a FIFO reset cannot revive a corrupted register state, nor an FSM that
will not restart from FRAME_START alone.

#### Why the data-ready line is polled as well as interrupted

`set_fifo_limit()` writes `SFCTL.FIFO_CREF`, and the radar holds its data-ready
output asserted for as long as the FIFO fill exceeds that threshold. It is a
**level**, not a pulse — but the GPIO is armed for a rising edge
(`GPIO_INT_EDGE_TO_ACTIVE`), which only occurs when the fill crosses the limit
from below.

Normally each read drains exactly one frame, the fill drops under the limit, the
line falls, and the next frame produces a fresh edge. If a frame arrives before
the previous read completes — one scheduling delay approaching the frame period
is enough — the fill never drops below the limit, **the line never falls, and no
further edge is ever generated**. The radar keeps chirping and SPI keeps
answering, but the interrupt is gone permanently.

This is a latent race rather than the cause of any stall observed so far: the
`(0 read on level)` count in the session summary tells you whether it is being
hit at all, and `irq line 0` in a timeout message rules it out for that stall.

The streaming loop therefore tests the level before waiting on the semaphore. An
asserted line already means a full frame is waiting, so it is read immediately
instead of waiting for an edge that cannot come; a backlog drains one frame per
iteration until the line falls, and only then is the edge waited for again. The
count appears in the session summary:

```
mmWave streaming stopped after 11540 frames (37 read on level)
```

A steady non-zero count is normal under load. Before the level check, every one
of those would have ended the session.

Isolated losses only cost the host a gap in the stream. But an overflow latches
a sticky error bit that makes *every* later read fail, and no amount of waiting
clears it — so after `MMWAVE_FRAME_ERROR_LIMIT` (5) consecutive losses the
capture is rebuilt: chirp FSM stopped, FIFO reset (which clears the error bits),
pending interrupt discarded, edge interrupt re-armed, framing restarted. The
re-arm is deliberately ordered after the FIFO reset — arming while the data-ready
line is still asserted latches nothing, because the rising edge has passed.

```
Restarting radar capture (1/3) after 5 consecutive frame errors at frame 4213
```

Up to `MMWAVE_RECOVERY_LIMIT` (3) restarts are attempted before the session ends
with `Radar unrecoverable after 3 restarts; ending capture`. The teardown then
runs normally, so the radar is left configured and a fresh start command works
without power-cycling it. Because the host still believes it is streaming, its
own stop command will report `mmWave not streaming` — that message following an
`unrecoverable` line is the expected pairing, not a second fault.

A restart also re-seeds the expected test-pattern word, since the radar restarts
its LFSR whenever framing restarts; without that, `CONFIG_MMWAVE_TEST_PATTERN`
builds would report every sample as a mismatch after the first recovery.

`CONFIG_MMWAVE_PROBE_LOOP=y` re-reads CHIP_ID once a millisecond for two seconds
afterwards, so a logic analyser has traffic to trigger on — a single register
access is one ~4 µs burst tens of milliseconds after the BLE command, which is
impractical to catch in auto-trigger mode. Pair it with
`CONFIG_MMWAVE_SPI_FREQ_MHZ=1` to stretch each transfer to about 32 µs.

### Radar settings adjustable at runtime

Three radar registers can be changed without rebuilding, via BLE commands the
host sends at start-up (opcodes 49/50/51 — see
[BLE protocol](./ble_protocol.md)):

| Setting | Values | Effect |
|---|---|---|
| IF gain | 18–60 dB, 14 discrete steps | Receiver gain. Too low is noise-limited; too high clips the 12-bit ADC |
| TX power | 0–31 | Illumination, trading SNR against power draw |
| Frame rate | 25 / 50 / 100 / 150 / 200 fps | Sampling rate of the phase waveform |

BioGUI exposes all three, plus the host-side choice of which range bin the pulse
waveform is taken from, in a settings dialog on the radar interfaces — so the
usual tuning loop needs no firmware rebuild. Everything else about the radar
(chirps per frame, samples per chirp, RX count, chirp timing, sweep band) is
fixed by the compiled-in register profile.

Two signals make that tuning loop quick. `mmwave_level` reports the per-frame ADC
min and max, so reaching 0 or 4095 means the IF gain is clipping. `mmwave_amp`
reports the selected bin's magnitude in dB — roughly 0 dB for noise alone up to
72 dB for a full-scale return — which is what tells you the gain is high enough
to be worth reading at all, and whether a phase excursion had any signal behind
it. The range-time heatmap marks the selected bin, the quickest way to check that
the bin actually contains the returned energy.

> The radar shares two pins with the PDM microphone (RST = P0.04 = PDM CLK,
> IRQ = P0.12 = PDM DATA), so the two cannot be used together. The mmWave build
> disables `&pdm0` and compiles `sensors/mic/mic_unavailable.c` in place of
> `mic_appl.c`, which rejects microphone commands at runtime.
>
> The radar's power-enable pin (P0.07) is also the battery-monitor ADC input, so
> battery telemetry is frozen between `TURN_ON_MMWAVE` and `TURN_OFF_MMWAVE` —
> enforced by both `pwr_set_measurement_gate()` and `pwr_set_adc_gate()`. The
> second is essential: without it a quiet cycle steals the pin and powers the
> radar off. See [the battery-ADC pin conflict](#the-battery-adc-pin-conflict-a-fixed-two-minute-stall).
>
> The radar shares SPI_A with the ADS1298 AFEs; the bus is switched to SPI mode
> 0 at 8 MHz for each radar transaction and restored to mode 1 at 4 MHz
> afterwards, under `spi_a_mutex`, so ExG and radar can stream together. Because
> the ADS1298 driver releases that mutex before its async transfer physically
> finishes, the radar additionally waits for `spi_a_transfer_in_flight()` to
> clear before rewriting the bus configuration. Concurrent ExG + radar streaming
> has not yet been validated on hardware.

### Board Synchronization

| Option | Type | Default | Description |
|--------|------|---------|-------------|
| `CONFIG_BOARD_SYNC_ROLE_STANDALONE` | bool | y | Single device, no sync |
| `CONFIG_BOARD_SYNC_ROLE_PRIMARY` | bool | n | Primary board in multi-device setup |
| `CONFIG_BOARD_SYNC_ROLE_SECONDARY` | bool | n | Secondary board in multi-device setup |
| `CONFIG_BOARD_ID` | int | 1 | Unique board identifier (1-255) |
| `CONFIG_BOARD_SYNC_PERIODIC_MS` | int | 1000 | Periodic sync pulse interval (ms) |

