# Smart Puck

An ESP32-S3-based smart hockey puck: accelerometer-triggered wake/gameplay,
addressable LEDs, buzzer feedback, and BLE control from a companion web app.

## Repo structure

```
code/
├── firmware/     # PlatformIO project (ESP32-S3 Arduino firmware)
└── web-app/      # Browser-based BLE control page

hardware/         # Schematic, PCB, and BOM (EasyEDA exports — pending upload)

docs/
└── PROJECT_HISTORY.md   # Design decisions, pin/power architecture, BOM fixes
```

Start with `docs/PROJECT_HISTORY.md` for the "why" behind the design, and each
subfolder's own README for the "how" of working with that piece.
