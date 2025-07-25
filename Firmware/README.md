
# ExG Stream Demo

This is the 

## Getting Started
Make sure to attach the e Mainboard to the ExGShield or the EMG shield.

1. Flash the NRF application by opening it in Visual Studio Code using the NRF Connect SDK extension.
```sh
cd src_NRF
west build -b nrf5340_senseiv1_cpuapp
west flash
```

Make sure your `SENSEI_SDK_ROOT` environment variable is set to the path of the SENSEI SDK.

