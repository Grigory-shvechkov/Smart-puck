# Hardware

This folder holds the electrical design files for the Smart Puck: schematic,
PCB layout, and bill of materials.

The board was designed in EasyEDA. Suggested layout once exports are added:

```
hardware/
├── schematic/      # Schematic PDF/PNG exports, .json/.eeschema source if exported
├── pcb/             # PCB layout exports, Gerbers, drill files
├── bom/              # Bill of materials (CSV/XLSX)
└── README.md
```

**Status:** design files have not yet been uploaded to this repo — see
`docs/PROJECT_HISTORY.md` for the pin assignments, part choices, and BOM fixes
that were worked out during schematic/PCB review, pending the actual exports
(schematic PDF, Gerbers, BOM CSV) being added here from EasyEDA.

Key parts referenced in the design history:
- MCU: ESP32-S3
- Accelerometer: LIS3DH (I2C, `0x18`)
- Gesture/proximity: APDS-9960 (I2C, `0x39`)
- LEDs: 6x WS2812B
- Power switch: MSK-12C02 slide switch (master power, in series between `B+` and `VBAT`)
- Buttons: TS-1187A (x2, signal buttons to GPIO/GND)
- Battery: LiPo via JST connector, USB-C charging, LDO for 3V3
