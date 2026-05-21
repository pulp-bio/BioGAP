This document provides instruction on how to get started with the firmware, including building, flashing, and running the application on the nRF5340.

# Cloning the SENSEI-SDK

In order to build the firmware, you first need to clone the SENSEI-SDK repository, which contains the necessary Zephyr board support package and other dependencies.

```bash
git clone https://github.com/pulp-bio/sensei-sdk.git
```

Then move to the `sensei-sdk` folder:

```bash
cd sensei-sdk
```

Now you need to update the submodules to clone all the necessary third-party dependencies:

```bash
git submodule update --init --recursive
```

# Cloning the BioGAP Repository
Now you need to clone the BioGAP repository, which contains the firmware source code and the custom modifications for the SENSEI-SDK.

```bash
git clone https://github.com/pulp-bio/BioGAP.git
```

Then move to the `BioGAP` folder:

```bash
cd BioGAP
```

# Adapting the SENSEI-SDK for BioGAP

The cloned SENSEI-SDK is not ready to be used for the BioGAP firmware.
For this reason, inside this repository, under `Firmware/` you will find two folders: 

- `custom_dts`: Contains a custom file for the Analog Front-End (AFE)
- `custom_shields`: Contains the custom shield definitions for the ExG (EEG/EMG), PPG and Wi-Fi/SD sensors.

You need to copy the content of these two folders into the corresponding folders in the `sensei-sdk` repository.

First move to the BioGAP directory:

```bash
cd BioGAP
```

```bash
cp -r Firmware/custom_dts/* ~/sensei-sdk/NRF/dts/bindings
cp -r Firmware/custom_shields/* ~/sensei-sdk/NRF/boards/shields
```

This will copy the custom device tree source files and the custom shield definitions into the SENSEI-SDK, allowing you to build the firmware for the BioGAP hardware.

Additionally you have to do some modification to the `sensei-sdk/NRF/boards/arm/nrf5340_senseiv1/nrf5340_senseiv1_cpuapp.dts`.

First you need to comment the alias of the UART. The following snippet of code:
```
	aliases {
		i2ca = &i2c0;
		i2cb = &i2c1;
		uartgap = &uart_gap;
	};
```

Should become:
```
    aliases {
        i2ca = &i2c0;
        i2cb = &i2c1;
        // uartgap = &uart_gap;
    };
```

Then, just under the modification you need to add the following lines:

```
	buttons{
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

So the final code should look something like this:

```
	aliases {
		i2ca = &i2c0;
		i2cb = &i2c1;
		// uartgap = &uart_gap;
	};
	buttons{
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

Then you need to comment out the following lines:

```
	uart_gap_default: uart0_default {
		group1 {
			psels = <NRF_PSEL(UART_TX, 1, 0)>,
				<NRF_PSEL(UART_RX, 1, 1)>;
		};
	};

	uart_gap_sleep: uart0_sleep {
		group1 {
			psels = <NRF_PSEL(UART_TX, 1, 0)>,
				<NRF_PSEL(UART_RX, 1, 1)>;
			low-power-enable;
		};
	};
```

Finally, you also need to comment out these lines:

```
uart_gap: &uart3{
	status = "okay";
	current-speed = <115200>;

	pinctrl-0 = <&uart_gap_default>;
	pinctrl-1 = <&uart_gap_sleep>;
	pinctrl-names = "default", "sleep";
};
```
After these modifications, the SENSEI-SDK should be ready to be used for building the BioGAP firmware.

# Building the Firmware

Once everything is set up, you can build the firmware. The instruction are exactly the same as the one provided in the [SENSEI-SDK repository](https://github.com/pulp-bio/sensei-sdk).

## Adjusting Paths
You need to export the environment variable SENSEI_SDK_ROOT

In Windows (Powershell):

```bash
cd sensei-sdk
pwd 
```
This will output the path of your sensei-sdk directory

copy the path and then export the environment variable.

```bash
[Environment]::SetEnvironmentVariable("SENSEI_SDK_ROOT", "C:\Users\giusy\OneDrive\Desktop\BIOGAP\sensei-sdk", "User")
```
Then close and reopen PowerShell / VS Code.

After reopening, check that the environment export was successfull. 

```bash
echo $env:SENSEI_SDK_ROOT
```

## Using VS Code Connect APP

1. Use the "Open an existing application" to open BioGAP/firmware/src_NRF. Important: you must open first the sensei-sdk folder in VS code and open the extension from there. 
2. In the "Applications" tab of the nRF Connect SDK extension, select the newly added application and click on "Add build configuration".
3. In the new window under "CMakePreset", select "Build for NRF5340 SENSEIV1C APP (build)"
// and select v2.6.1 as SDK version and v2.9.1 as Toolchain version
Note: Newer version of SDK and Toolchain are currently not supported. 
4. Select "nrf5340_senseiv1_cpuapp" as Board Target. If not recognized, it means you are in the wrong workspace. 
5. Click on "Generate and Build" to build the application.
In the "Actions" tab click on "Flash" to flash the application to the board.