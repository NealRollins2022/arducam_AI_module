# ArduCAM Mega Zephyr Module (nRF5340-ready)

This repository is a Zephyr **module** that provides an ArduCAM Mega SPI camera driver (`drivers/video/arducam_mega`) and a sample (`samples/drivers/video/arducam_mega`).

## Use as a module (west)
Add this repo to your `west.yml` (manifest) and `west update`. Zephyr will discover it via `zephyr/module.yml`.

## Build the sample
```
west build -b nrf5340dk_nrf5340_cpuapp_ns samples/drivers/video/arducam_mega -p always
west flash
```

## Enable driver in your app
```
CONFIG_VIDEO=y
CONFIG_ARDUCAM_MEGA=y
```
