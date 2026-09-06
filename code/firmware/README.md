# Firmware

PlatformIO project for the ESP32-S3 firmware.

## Build / upload

```
cd code/firmware
pio run              # build
pio run -t upload    # flash
pio device monitor    # serial monitor
```

Configured for `esp32-s3-devkitc-1` (Arduino framework). See `platformio.ini`
for board flags (16MB flash, QIO OPI PSRAM, `FastLED` library dependency).

See `docs/PROJECT_HISTORY.md` at the repo root for the reasoning behind the
pin assignments and power/sleep architecture used here.
