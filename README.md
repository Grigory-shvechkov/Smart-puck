# Smart Puck

![Platform](https://img.shields.io/badge/platform-ESP32--S3-3559C9)
![Framework](https://img.shields.io/badge/framework-Arduino%20%2F%20PlatformIO-2F8F5B)
![Control](https://img.shields.io/badge/control-Web%20Bluetooth-8A2BE2)

An ESP32-S3-based smart hockey puck: accelerometer-triggered wake/gameplay,
addressable LEDs, buzzer feedback, and BLE control from a companion web app —
no native app install required.

## Features

- **Slap-to-wake** — an accelerometer tap/click wakes the puck from deep sleep
  and doubles as the "press-to-start" gameplay gesture.
- **Gesture & proximity sensing** — an APDS-9960 sensor detects hand passes
  near the puck for touch-free interaction.
- **6x addressable RGB LEDs** — full-color feedback, controllable per-LED or
  all at once.
- **Piezo buzzer** — audible feedback for events and gameplay cues.
- **Browser-based control** — connects over Web Bluetooth directly from
  Chrome or Edge (desktop or Android), with a live 3D orientation view and
  gesture visualizer. See it in [`code/web-app/`](code/web-app/).

## Repo structure

```
code/
├── firmware/     # PlatformIO project (ESP32-S3 Arduino firmware)
└── web-app/      # Browser-based BLE control page

hardware/         # Schematic, PCB, and BOM (EasyEDA exports)
├── schematic/    # Schematic PDF exports
├── pcb/          # PCB layout export
└── bom/          # Bill of materials (pending)

docs/
└── PROJECT_HISTORY.md   # Design decisions, pin/power architecture, BOM fixes
```

Start with [`docs/PROJECT_HISTORY.md`](docs/PROJECT_HISTORY.md) for the "why"
behind the design, and each subfolder's own README for the "how" of working
with that piece:

- [`code/firmware/README.md`](code/firmware/README.md) — build/flash the
  ESP32-S3 firmware
- [`code/web-app/README.md`](code/web-app/README.md) — run the BLE control
  page
- [`hardware/README.md`](hardware/README.md) — schematic, PCB, and BOM status

## Key components

| Part | Role |
|---|---|
| ESP32-S3 | MCU |
| LIS3DH | Accelerometer (wake/tap detection) |
| APDS-9960 | Gesture / proximity sensor |
| WS2812B x6 | Addressable RGB LEDs |
| MSK-12C02 | Master power slide switch |
| LiPo + USB-C | Battery power, charged over USB-C |
