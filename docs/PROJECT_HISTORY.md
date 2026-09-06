# Smart Puck — Project History & Design Decisions

This document compiles the design reasoning and decisions worked through in chat
sessions while designing the schematic, PCB, and firmware for the Smart Puck.
It exists so this context survives independent of any one chat thread.

> Note on pin numbers: where this doc's narrative and `src/main.cpp` differ on a
> specific GPIO, trust the code — it reflects the final wiring. The reasoning
> below (why a pin/part was chosen) still applies even if the exact IO number
> shifted during layout.

## Hardware Overview

- **MCU:** ESP32-S3
- **Sensors:** LIS3DH accelerometer (I2C, address `0x18`, SA0 grounded), APDS-9960
  gesture/proximity sensor (I2C, address `0x39`)
- **Output:** 6x WS2812B addressable LEDs, piezo buzzer (driven through a
  transistor, not directly from GPIO)
- **Power:** LiPo battery via JST connector, USB-C charging, LDO regulator for 3V3,
  hardware slide switch as master power cutoff
- **Current firmware pin map** (see `src/main.cpp` for authoritative values):
  LED data IO5, buzzer IO4, I2C SDA/SCL IO8/IO9, accelerometer INT IO6,
  gesture INT IO16, wake button IO2 (active-low, to GND)

## Key Design Decisions

### Power architecture: switch vs. sleep vs. buttons
Three distinct concerns kept coming up and were deliberately kept separate:

1. **SW1 (hardware slide switch, e.g. MSK-12C02)** sits directly in the battery
   line between `B+` (JST/battery side) and `VBAT` (everything downstream — LDO,
   LEDs, buzzer, sensors). It is the only thing that gives a true zero-current
   off state. This matters because the WS2812B LEDs draw current even when
   "off" (roughly ~1mA each, ~6mA total for six), which would otherwise drain a
   battery in a couple of weeks of idle storage. SW1 is meant to be flipped
   rarely — packing for travel, shipping, long storage — not as a daily on/off
   control.
2. **Deep sleep + accelerometer wake** is the daily on/off mechanism. The LIS3DH
   interrupt line is wired to a wake-capable GPIO so a slap/shock on the puck
   wakes the ESP32 from deep sleep in milliseconds. This doubles as the
   "press-to-start" gameplay mechanic — the same slap that wakes the device also
   starts a round, since a physical hit is a more natural start gesture for this
   product than a dedicated button.
3. **BOOT (IO0) and EN/RESET buttons** are development/recovery tools, not
   gameplay or power controls. BOOT is a strapping pin (bootloader mode when held
   low at boot); EN is chip enable (power-cycles the MCU). The standard recovery
   ritual — hold BOOT, tap RESET, release BOOT — was noted as the fallback for a
   bricked board, important since this is a sealed enclosure and hand-reflashing
   via a soldering iron is the alternative.

Explicitly rejected: powering the whole board on/off via a plain push button
(momentary — can't hold state) and a MOSFET "soft latch" power circuit (phone-
style press-to-power). The soft latch was scoped as legitimate v2 work (~5 extra
components, new firmware state machine) but decided against for v1 to avoid
adding complexity at the finish line of the first board.

### Wake-capable pins (ESP32-S3)
Only **GPIO0–GPIO21** are in the RTC domain that stays powered during deep sleep;
anything outside that range (e.g. IO35) cannot wake the chip. This constrained
where the accelerometer interrupt and wake button could be placed during
schematic/layout.

### Gameplay input
- Primary: accelerometer tap/click detection (LIS3DH built-in feature) — the
  slap that wakes the device also starts a round.
- Backup/dev input: BOOT button (IO0), readable as an ordinary GPIO outside the
  reset instant — useful for testing without a satisfying "thwack" for real play.
- A dedicated top-face button was considered and dropped: it would compete with
  the LED ring and gesture sensor window for physical space on the puck's face,
  and the accelerometer already covers the use case.

### BOM / schematic fixes made during "Convert to PCB"
EasyEDA's Convert-to-PCB step silently drops symbols that have no footprint
assigned (rather than erroring), which caused two real issues caught via BOM
review:
- **SW1 was missing from the PCB import** because its schematic symbol had no
  footprint. Fix: assign the MSK-12C02 slide switch footprint (3-pin — common
  pin to `B+`, one outer pin to `VBAT`, third pin unconnected), then re-run
  Convert to PCB and confirm SW1 appears in the import list before applying.
- **R10/R11** (the voltage-divider resistors) initially pulled in a through-hole
  axial footprint instead of the intended 0402 SMD package — comically oversized
  next to the rest of the board and not placeable by JLC's assembly service.
  Fixed by reassigning to the 0402 (C25803) footprint.
- Net naming cleanup: `B+` was consolidated/relabeled to `VBAT` in places to
  avoid an accidental permanent short once SW1 was correctly in the power path.

### Driver / firmware references used
- STMicroelectronics' official [STMems_Standard_C_drivers](https://github.com/STMicroelectronics/STMems_Standard_C_drivers)
  for the LIS3DH — platform-independent C with function pointers for I2C
  read/write, a good pattern reference even if not used verbatim.
- [Adafruit_LIS3DH](https://github.com/adafruit/Adafruit_LIS3DH) — Arduino-style
  driver with working click/tap-interrupt configuration, directly relevant to
  the slap-to-wake feature.
- [esp-idf-lib](https://github.com/UncleRus/esp-idf-lib) and Espressif's own
  `led_strip` component / [FastLED](https://github.com/FastLED/FastLED) as
  references for the WS2812B LED driving and general ESP-IDF driver structure.

## Known Trade-offs / Open Items
- Idle battery drain is dominated by the always-on LED rail (~6mA for six
  WS2812Bs) plus LDO quiescent current — expect roughly 2–3 weeks from a full
  charge if left powered on and idle. SW1 is the mitigation for storage/shipping.
- A v1.1 revision could add a P-MOSFET on the LED rail to let firmware cut LED
  power during deep sleep, removing the need to rely on the physical switch as
  often. Not implemented in v1 by design (scope control).
- Soft-latch (press-to-power) circuit remains a candidate for v2 if the physical
  switch proves annoying in practice.

---
*Compiled from prior chat sessions covering schematic review, BOM/PCB import
troubleshooting, and pin/power architecture decisions. If something here looks
stale relative to the current schematic or code, the schematic/code wins —
update this file to match.*
