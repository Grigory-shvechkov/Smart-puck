# Hardware

This folder holds the electrical design files for the Smart Puck: schematic,
PCB layout, and bill of materials.

The board was designed in EasyEDA.

```
hardware/
├── schematic/
│   ├── SCH_Schematic1_2026-09-05.pdf   # Main schematic
│   └── SCH_Schematic2_2026-09-05.pdf   # Secondary sheet
├── pcb/
│   └── PCB_PCB1_2026-09-05.pdf         # PCB layout export
├── bom/              # Bill of materials (CSV/XLSX) — pending
└── README.md
```

**Status:** schematic and PCB layout PDF exports are in place. Gerbers/drill
files and the BOM CSV are still pending. See `docs/PROJECT_HISTORY.md` for the
pin assignments, part choices, and BOM fixes worked out during schematic/PCB
review.

Key parts referenced in the design history:
- MCU: ESP32-S3
- Accelerometer: LIS3DH (I2C, `0x18`)
- Gesture/proximity: APDS-9960 (I2C, `0x39`)
- LEDs: 6x WS2812B
- Power switch: MSK-12C02 slide switch (master power, in series between `B+` and `VBAT`)
- Buttons: TS-1187A (x2, signal buttons to GPIO/GND)
- Battery: LiPo via JST connector, USB-C charging, LDO for 3V3
