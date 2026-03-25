This document is the most important part of the firmware documentation. It provides clear instruction on how to get started with the firmware, including building, flashing, and running the application on the nRF5340.

> [!CAUTION]
> This guide has been written on March 2026, and in the future some steps might change. 

# Cloning the SENSEI-SDK

In order to build the firmware, you first need to clone the SENSEI-SDK repository, which contains the necessary Zephyr board support package and other dependencies.

> [!CAUTION]
> Please follow the instruction in this file instead of following directly the instructions in the SENSEI-SDK repository, as we will do some modifications in the following steps of this guide.

The first step is to clone the SENSEI-SDK repository. To simplify the instructions, we will assume that you clone from your home directory (`~`).

```bash
cd ~
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
cd ~
git clone https://github.com/pulp-bio/BioGAP.git
```

Then move to the `BioGAP` folder:

```bash
cd BioGAP
```

# Adapting the SENSEI-SDK for BioGAP

The cloned SENSEI-SDK is not ready to be used for the BioGAP firmware. For this reason, inside this repository, under `Firmware/` you will find two folders: 

- `custom_dts`: Contains a custom file for the Analog Front-End (AFE)
- `custom_shields`: Contains the custom shield definitions for the ExG (EEG/EMG) and PPG sensors.

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