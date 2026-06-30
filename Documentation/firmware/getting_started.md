# Getting Started

This guide covers how to set up the development environment, clone the required repositories, build the firmware, and flash it onto the BioGAP mainboard.

## Prerequisites

- **nRF Connect SDK** (NCS) v2.x with Zephyr RTOS
- **Visual Studio Code** with the nRF Connect for VS Code extension (recommended)
- **SEGGER J-Link** debugger (or the BioGAP Debug Board)
- **Python 3.10+** with west build tool
- Git

## Step 1: Clone the SENSEI-SDK

The firmware depends on the SENSEI-SDK, which provides the Zephyr board support package for the custom `nrf5340_senseiv1` board and all third-party dependencies.

```bash
git clone https://github.com/pulp-bio/sensei-sdk.git
cd sensei-sdk
git submodule update --init --recursive
```

Set the `SENSEI_SDK_ROOT` environment variable to point to this directory:

```bash
export SENSEI_SDK_ROOT=/path/to/sensei-sdk
```

## Step 2: Clone the BioGAP Repository

```bash
git clone https://github.com/pulp-bio/BioGAP.git
```

## Step 3: Integrate Custom Files into the SENSEI-SDK

The BioGAP firmware requires custom device tree bindings and shield definitions that are not part of the base SENSEI-SDK. These are provided in the `Firmware/` directory.

### Copy Custom Device Tree Bindings

The ADS1298 AFE binding must be added to the SDK:

```bash
cp -r Firmware/custom_dts/* $SENSEI_SDK_ROOT/NRF/dts/bindings
```

### Copy Custom Shield Definitions

The ExG and PPG shield overlays must be added to the SDK:

```bash
cp -r Firmware/custom_shields/* $SENSEI_SDK_ROOT/NRF/boards/shields
```

## Step 4: Modify the SENSEI-SDK Device Tree

The base SENSEI-SDK device tree file needs modifications to support the BioGAP hardware. Edit the file:

```
$SENSEI_SDK_ROOT/NRF/boards/arm/nrf5340_senseiv1/nrf5340_senseiv1_cpuapp.dts
```

### 4a. Comment out the UART GAP alias

Change the `aliases` block from:

```dts
aliases {
    i2ca = &i2c0;
    i2cb = &i2c1;
    uartgap = &uart_gap;
};
```

To:

```dts
aliases {
    i2ca = &i2c0;
    i2cb = &i2c1;
    // uartgap = &uart_gap;
};
```

### 4b. Add GPIO button definitions

Add the following block immediately after the `aliases` block:

```dts
buttons {
    gpio_lis2duxs12_int1: gpio_lis2duxs12_int1 {
        gpios = <&gpio0 23 GPIO_ACTIVE_HIGH>;
        label = "LIS2DUXS12_INT";
    };
    gpio_soft_rst: gpio_soft_rst {
        gpios = <&gpio0 26 GPIO_ACTIVE_HIGH>;
        label = "BUTTON_SOFT_INT";
    };
};
```

### 4c. Comment out the UART3 pin control

Comment out the `uart_gap_default` and `uart_gap_sleep` pin control groups:

```dts
// uart_gap_default: uart0_default {
//     group1 {
//         psels = <NRF_PSEL(UART_TX, 1, 0)>,
//                 <NRF_PSEL(UART_RX, 1, 1)>;
//     };
// };
//
// uart_gap_sleep: uart0_sleep {
//     group1 {
//         psels = <NRF_PSEL(UART_TX, 1, 0)>,
//                 <NRF_PSEL(UART_RX, 1, 1)>;
//         low-power-enable;
//     };
// };
```

### 4d. Comment out the UART3 node definition

Comment out the `uart_gap` node at the bottom of the file:

```dts
// uart_gap: &uart3 {
//     status = "okay";
//     current-speed = <115200>;
//     pinctrl-0 = <&uart_gap_default>;
//     pinctrl-1 = <&uart_gap_sleep>;
//     pinctrl-names = "default", "sleep";
// };
```

## Step 5: Build the Firmware

Once everything is set up, you can build the firmware. The instruction are exactly the same as the one provided in the [SENSEI-SDK repository](https://github.com/pulp-bio/sensei-sdk).