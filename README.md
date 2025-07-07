# BioGAP v2

This repository contains the complete design files for the **BioGAP v2**, an integrated, ultra-low-power edge-AI platform for biosensing applications.

## Overview

The **BioGAP v2** brings together modular sensing shields, robust edge processing, and flexible debugging in a single, cohesive system. It includes the main processing board and a range of specialized shields for biopotential (ExG, EMG) and PPG sensing. A dedicated debug board and a reference template shield support rapid prototyping and customization for diverse biomedical and wearable monitoring applications.

## Included PCBs

- **Mainboard**  
  High-performance core board with GAP9 and NRF5340 SoCs, optimized for edge AI and low-power wireless connectivity.

- **ExG Shield**  
  Flexible 16-channel biopotential sensing shield supporting both active and passive electrodes, fully differential or common reference configurations.

- **EMG Shield**  
  Dedicated EMG acquisition shield with 16 channels, swappable electrode-to-channel mapping PCBs, and balanced analog operation.

- **PPG Shield**  
  Compact two-channel PPG module (red & IR LED) for heart rate and SpO2 monitoring. Easily connects via cable for flexible placement.

- **Debug Board**  
  Simplifies flashing and debugging with dedicated debug connectors for all supported SoCs, signal breakouts, status LEDs, and user buttons.

- **Template Shield**  
  Reference PCB template with pre-routed connectors, serving as a starting point for custom sensing shield development.

## Key Features

- Modular stackable architecture for multi-modal physiological sensing.
- Ultra-low-power edge AI with GAP9’s parallel processing and NE16 accelerator.
- Wireless BLE connectivity via NRF5340.
- Versatile electrode configurations for a range of biosignal recordings.
- User-friendly debugging and expansion with standardized board-to-board connectors.

## Changelog

A detailed changelog is available in the [Changelog.md](Changelog.md) file, documenting major updates and design revisions for each board.

## Contributors

- **Sebastian Frey** ([sefrey@iis.ee.ethz.ch](mailto:sefrey@iis.ee.ethz.ch))

## License

This repository makes use of the following licenses:  
- for all *hardware*: [Solderpad Hardware License Version 0.51](LICENSE.hw)  
- for all *images*: [Creative Commons Attribution 4.0 International License](LICENSE.images)

For further information, please refer to the license files: `LICENSE.hw`, `LICENSE.images`.
