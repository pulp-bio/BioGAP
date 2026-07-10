# Getting Started

This guide covers how to set up the development environment, clone the required repositories, build the firmware, and flash it onto the BioGAP mainboard.

## Prerequisites

- **nRF Connect SDK** (NCS) v2.6.1 with Zephyr RTOS (newer SDK/toolchain versions are not currently supported)
- **Visual Studio Code** with the nRF Connect for VS Code extension (recommended)
- **SEGGER J-Link** debugger (or the BioGAP Debug Board)
- **Python 3.10+** with the `west` build tool
- Git

## Step 1: Clone the SENSEI-SDK

The firmware depends on the SENSEI-SDK, which provides the Zephyr board support package for the custom `nrf5340_senseiv1` board, the BioGAP shield definitions (ExG, PPG, WiFi/SD), and all third-party dependencies.

```bash
git clone https://github.com/pulp-bio/sensei-sdk.git
cd sensei-sdk
git submodule update --init --recursive
```

Check out the branch that adds the BioGAP board overlays and shield definitions:

```bash
git checkout feat/add-biogap-overlays
```

> There is **no manual copy-paste step anymore**. Earlier versions of this guide had you copy `Firmware/custom_dts` and `Firmware/custom_shields` into the SDK and hand-edit the board `.dts` file — that workflow is gone. Everything BioGAP-specific (the `nrf5340_senseiv1` board files, the `SENSEI_ExGShield` / `SENSEI_PPGShield` / `SENSEI_WiFi_SD_Shield_V1` shield overlays, the ADS1298 devicetree binding) now lives directly in this branch of the sensei-sdk repo.

## Step 2: Clone the BioGAP Repository

```bash
git clone https://github.com/pulp-bio/BioGAP.git
```

## Step 3: Point the Build at Your SENSEI-SDK Clone

The build reads the `SENSEI_SDK_ROOT` environment variable to locate the SDK (registered as a Zephyr module in `Firmware/src_NRF/CMakeLists.txt`). Get this wrong and CMake fails at configure time with either `SENSEI_SDK_ROOT is not defined` or `Expected Zephyr module metadata at '<path>/NRF/zephyr/module.yml'`.

### Windows (PowerShell)

Set it **persistently** (so every new shell/VS Code launch picks it up) *and* in your **current session** (so it's available immediately, without waiting for a restart):

```powershell
[Environment]::SetEnvironmentVariable("SENSEI_SDK_ROOT", "C:\path\to\sensei-sdk", "User")
$env:SENSEI_SDK_ROOT = "C:\path\to\sensei-sdk"
```

Verify:

```powershell
echo $env:SENSEI_SDK_ROOT
```

> **This is the part that actually trips people up.** `SetEnvironmentVariable(..., "User")` writes to the registry, but it only affects *new* processes started after the call — it does **not** update already-running ones. If VS Code (or a terminal, or the nRF Connect build task) was already open when you set it, it will keep using the old value no matter how many times you reopen a terminal panel or "Reload Window" inside it. **Fully quit VS Code** (all windows, confirm it's not still running in the background) and relaunch it before building again.

### Linux / macOS (bash/zsh)

```bash
export SENSEI_SDK_ROOT=/path/to/sensei-sdk
```

Add this to your shell profile (`.bashrc`/`.zshrc`) to persist it across sessions.

## Step 4: Build the Firmware

Two ways to build; the command line is the more reliable of the two in practice (see Troubleshooting) — try it first if the extension gives you trouble.

### Option A: Command line (recommended)

`west build`/`west flash` are **west extension commands** — they only exist when `west` can locate its workspace (the NCS installation's `.west` folder, e.g. `C:\ncs\v2.6.1`) by walking up from your current directory. Your BioGAP/sensei-sdk checkouts almost certainly live *outside* that workspace tree, so you must run `west` **from inside the NCS workspace root** and point it at your application explicitly with `--build-dir`/a positional source-dir argument — this is the standard, supported way Zephyr/NCS builds out-of-tree applications, not a workaround.

```powershell
# Make sure SENSEI_SDK_ROOT is set in *this* shell (see Step 3) — check with:
echo $env:SENSEI_SDK_ROOT

cd C:\ncs\v2.6.1   # your NCS workspace root — wherever `west init` set it up
west build --build-dir C:\path\to\BioGAP\Firmware\src_NRF\build C:\path\to\BioGAP\Firmware\src_NRF --board nrf5340_senseiv1_cpuapp --pristine
```

Drop `--pristine` on later incremental builds (it forces a full reconfigure/clean build — useful the first time or after changing Kconfig/CMakeLists, unnecessary otherwise, and slower).

On Linux/macOS the same idea applies with your shell's syntax (`cd ~/ncs/v2.6.1 && west build --build-dir ~/BioGAP/Firmware/src_NRF/build ~/BioGAP/Firmware/src_NRF --board nrf5340_senseiv1_cpuapp --pristine`).

### Option B: nRF Connect for VS Code Extension

1. Open the `sensei-sdk` folder in VS Code first, then use "Open an existing application" from the nRF Connect extension to open `BioGAP/Firmware/src_NRF` — opening `src_NRF` directly, without the SDK folder as the workspace root, will not resolve the SDK modules correctly.
2. In the "Applications" tab, select the application and click **Add Build Configuration**.
3. Under **CMakePreset**, select "Build for NRF5340 SENSEIV1C APP (build)".
4. Select **SDK version v2.6.1** and **Toolchain version v2.9.1** (newer versions are not currently supported).
5. Select **`nrf5340_senseiv1_cpuapp`** as the **Board Target**.
6. Click **Generate and Build**.

In practice, step 5 has been unreliable: the Board Target picker in the extension appears to build its list independently of this app's own `CMakeLists.txt` (where the SDK module is actually registered), so it may never offer `nrf5340_senseiv1_cpuapp` at all, and the field has not accepted it typed in manually either. If you hit that, fall back to Option A — the underlying build works correctly either way, since Option A is exactly what the extension itself runs under the hood.

## Step 5: Flash the Firmware

### Command line

From the same NCS-workspace-rooted shell as the build:

```powershell
cd C:\ncs\v2.6.1
west flash --build-dir C:\path\to\BioGAP\Firmware\src_NRF\build
```

This uses whichever runner the board supports by default (nrfjprog/J-Link for the nRF5340). Make sure the SEGGER J-Link (or BioGAP Debug Board) is connected before running it.

### nRF Connect for VS Code Extension

From the "Actions" tab of the build configuration, click **Flash**.

## Troubleshooting

**`CMake Error: ... is not a valid zephyr module` / `Expected Zephyr module metadata at '<path>/NRF/zephyr/module.yml'`**
`SENSEI_SDK_ROOT` doesn't point at a real sensei-sdk checkout (typo, stale/deleted clone, or a value set before you had cloned it). Re-check Step 3, and remember the VS Code restart requirement above.

**Devicetree parse error, e.g. `undefined node label 'spi_a'` in a shield overlay**
This means the build picked the *wrong board* — a generic Nordic dev-kit board (`nrf52840dk_nrf52840`, `nrf5340dk_nrf5340_cpuapp`, ...) instead of `nrf5340_senseiv1_cpuapp`. Those boards don't define the BioGAP-specific aliases (`spi_a`, `spi_b`, ...) the shield overlays reference. Double-check the `--board` value (CLI) or re-select the board target (extension) as described in Step 4.

**The board target reverted to a generic board after I deleted the `build/` folder**
The nRF Connect extension stores which board you picked for a given build configuration inside that build folder (`.vscode-nrf-connect.json`). Deleting `build/` to clear a stale `CMakeCache.txt` also deletes that memory — you'll need to re-add the build configuration and re-select `nrf5340_senseiv1_cpuapp` explicitly; it won't come back on its own. This is one reason Option A (command line) is more predictable.

**`west: unknown command "build"; do you need to run this inside a workspace?`**
You ran `west build`/`west flash` from a directory that isn't inside (or a descendant of) the NCS workspace root. `cd` into the workspace root first (e.g. `C:\ncs\v2.6.1`) and pass your app's path explicitly via `--build-dir`/the positional source-dir argument, as shown in Step 4 Option A.

**`ImportError: DLL load failed while importing _ctypes`** (usually during the MCUboot child-image build, in `imgtool.py`)
An active Conda environment (prompt shows `(base)` or similar) is shadowing the NCS toolchain's own bundled Python DLLs on `PATH`. Run `conda deactivate` (repeat if nested) in that shell before building, or use a shell that doesn't auto-activate Conda (e.g. `powershell.exe -NoProfile`, or check `$PROFILE` for a `conda init` block to disable for this session).
