# ArduCAM Mega Zephyr Module for nRF5340

This module adds support for the ArduCAM Mega SPI camera in the nRF Connect SDK.

## Features
- Manual CS GPIO for precise timing
- Minimal capture demo that:
  - Triggers a capture
  - Polls until done
  - Reads FIFO length
  - Logs first 16 bytes of image (JPEG header expected)

## Setup

1. Add this module to your NCS workspace via `west.yml`:

```yaml
- name: arducam_AI_module
  remote: arducam
  revision: main
  path: modules/arducam
```

2. Update your workspace:

```bash
west update
```

3. Build the sample:

```bash
west build -b nrf5340dk_nrf5340_cpuapp_ns modules/arducam/samples/drivers/video/arducam_mega
```

4. Flash:

```bash
west flash
```

## Expected Output

On successful capture:

```
<inf> app: Starting capture...
<inf> app: Capture complete, FIFO length = 12345 bytes
<inf> app: FIFO[0:16]: 0xFF 0xD8 0xFF 0xE0 ...
```

The `FF D8 FF E0` sequence confirms a valid JPEG header.
