#include <Arduino.h>
#include <FastLED.h>
#include <Wire.h>
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>
#include "driver/rtc_io.h"

// ---- Pin map from schematic ----
#define LED_PIN      5   // LED_DATA -> IO5, through 330Ω to DIN of LED1
#define NUM_LEDS     6
#define BUZZ_PIN     4   // BUZZ -> IO4, transistor base driving buzzer
#define SDA_PIN      8   // I2C SDA -> IO8
#define SCL_PIN      9   // I2C SCL -> IO9
#define ACCEL_INT    6   // LIS3DH INT1 -> IO6
#define GEST_INT     16  // APDS-9960 INT -> IO16
#define WAKE_PIN     2   // Wake/power button -> IO2, to GND, active-low

#define LIS3DH_ADDR   0x18  // SA0 grounded
#define APDS9960_ADDR 0x39  // fixed address

// APDS-9960 proximity engine registers, used for simple gesture/motion
// detection (something moving near the sensor), not full directional
// (up/down/left/right) gesture decoding — that needs a lot more tuning.
#define APDS_ENABLE  0x80
#define APDS_ATIME   0x81
#define APDS_WTIME   0x83
#define APDS_PPULSE  0x8E
#define APDS_CONTROL 0x8F
#define APDS_CONFIG2 0x90
#define APDS_ID      0x92
#define APDS_PDATA   0x9C
#define APDS_STATUS  0x93
#define APDS_POFFSET_UR 0x9D
#define APDS_POFFSET_DL 0x9E
#define APDS_PICLEAR    0xE5
// APDS-9930 proximity data registers (16-bit)
#define APDS9930_PDATAL 0x98
#define APDS9930_PDATAH 0x99

// ---- Bluetooth Low Energy (Nordic UART Service) ----
// ESP32-S3 only supports BLE, not classic Bluetooth SPP — this is the
// standard "UART over BLE" profile most generic BLE terminal apps understand.
#define BLE_DEVICE_NAME   "Puck-Control"
#define SERVICE_UUID      "6E400001-B5A3-F393-E9A0-C91E24EC4001"
#define CHAR_UUID_RX       "6E400002-B5A3-F393-E9A0-C91E24EC4002" // app -> puck
#define CHAR_UUID_TX       "6E400003-B5A3-F393-E9A0-C91E24EC4003" // puck -> app

BLECharacteristic *txCharacteristic;
BLEServer *bleServerPtr = nullptr;
bool bleClientConnected = false;
bool bleActive = false;
bool waitingForConnection = false;
bool needsReadvertise = false;
unsigned long readvertiseAt = 0;
const unsigned long READVERTISE_DELAY_MS = 500;
unsigned long lastActivityTime = 0;
unsigned long awakeStartTime = 0;
const unsigned long SESSION_AUTO_SLEEP_MS = 3600000UL; // 1 hour safety auto-off
unsigned long lastBlinkTime = 0;
const unsigned long BLINK_INTERVAL_MS = 300;
bool blinkOn = false;

// Persists across deep sleep (but not a full power cycle/reflash) — used to
// confirm we actually intended to sleep, since esp_sleep_get_wakeup_cause()
// can report a stale EXT0 reading on a genuine cold boot right after
// reflashing, falsely making the board think the button was pressed.
RTC_DATA_ATTR bool intentionalSleep = false;

// LED state is only ever changed from loop() (the main task). BLE callbacks
// run on the BLE stack's own task — calling FastLED.show() from there (which
// briefly disables interrupts to bit-bang WS2812 timing) can stall or crash
// the BLE stack. Callbacks just set this flag; loop() applies it.
enum LedState { LED_NONE, LED_OFF, LED_GREEN };
volatile LedState pendingLedState = LED_NONE;

// Same reasoning as pendingLedState — BLE commands must not run tests
// directly from onWrite() (BLE task context). Defer to loop() instead.
volatile char pendingCommand = 0;
volatile bool requestLedSetOne = false;
volatile bool requestLedSetAll = false;
volatile bool requestLedClear = false;
volatile uint8_t ledSetIndex = 0;
volatile uint8_t ledSetR = 0;
volatile uint8_t ledSetG = 0;
volatile uint8_t ledSetB = 0;

// Live sensor streaming (accelerometer + proximity) for the web control page.
volatile bool requestStreamStart = false;
volatile bool requestStreamStop = false;
volatile bool requestDiag = false;
bool streamingActive = false;
unsigned long lastStreamSample = 0;
const unsigned long STREAM_INTERVAL_MS = 150;
const uint8_t PROX_TRIGGER_HIGH = 130;
const uint8_t PROX_TRIGGER_LOW = 95;
const uint8_t PROX_DEBOUNCE_SAMPLES = 3;
const bool PROXIMITY_ONLY_MODE = true;
bool proxAboveThreshold = false;
uint8_t proxRiseCount = 0;
uint8_t proxFallCount = 0;
bool apdsReady = false;
uint8_t lastProx = 0;
uint8_t sameProxCount = 0;
const uint8_t PROX_STUCK_LIMIT = 20;

// Direction inference for APDS-9930 systems: proximity gives near/far,
// accelerometer delta during an active gesture window gives left/right bias.
int16_t latestAccelX = 0;
int16_t latestAccelY = 0;
int16_t latestAccelZ = 0;
bool haveAccelSample = false;
bool gestureWindowActive = false;
int16_t gestureStartX = 0;
int16_t gestureStartY = 0;
int16_t prevAccelX = 0;
int16_t prevAccelY = 0;
bool havePrevAccel = false;
int32_t gestureAccumX = 0;
int32_t gestureAccumY = 0;
int32_t gestureScoreX = 0;
uint16_t gestureStepCount = 0;
int32_t accelBaseX = 0;
int32_t accelBaseY = 0;
bool accelBaseReady = false;
uint8_t gestureStartProx = 0;
uint8_t gestureMinProx = 255;
uint8_t gestureMaxProx = 0;
unsigned long gestureStartAt = 0;
unsigned long lastDirectionEmitAt = 0;
const int16_t GESTURE_ACCEL_DELTA_THRESHOLD = 700;
const int16_t GESTURE_ACCEL_STEP_DEADBAND = 50;
const int32_t GESTURE_ACCEL_ACCUM_THRESHOLD = 500;
const int16_t GESTURE_BASELINE_DELTA_THRESHOLD = 380;
const int16_t GESTURE_PROX_DELTA_THRESHOLD = 24;
const int16_t GESTURE_PROX_HYSTERESIS = 8;
const int16_t GESTURE_SCORE_THRESHOLD = 2200;
const unsigned long GESTURE_WINDOW_TIMEOUT_MS = 1800;
const unsigned long GESTURE_DIRECTION_UPDATE_MS = 250;
const unsigned long DIRECTION_RESEND_MS = 900;
String lastDirectionSent = "";
unsigned long lastDirectionSentAt = 0;

enum ApdsVariant {
  APDS_VARIANT_UNKNOWN = 0,
  APDS_VARIANT_9960,
  APDS_VARIANT_9930
};

ApdsVariant apdsVariant = APDS_VARIANT_UNKNOWN;

bool isNeutralDirection(const String& label) {
  return label == "CENTER,STABLE";
}

bool shouldSendDirection(const String& label) {
  unsigned long now = millis();
  if (label != lastDirectionSent || (now - lastDirectionSentAt) >= DIRECTION_RESEND_MS) {
    lastDirectionSent = label;
    lastDirectionSentAt = now;
    return true;
  }
  return false;
}

String inferDirectionLabel(int16_t dx, int16_t dy, int16_t dProx, int16_t baseDx, int16_t baseDy, int32_t accumX, int32_t accumY) {
  String lateral = "CENTER";

  // Primary: scored X-axis motion across the whole gesture window.
  if (abs(accumX) >= GESTURE_SCORE_THRESHOLD) {
    lateral = (accumX > 0) ? "RIGHT" : "LEFT";
  }

  if (lateral != "CENTER") {
    String depth = "STABLE";
    if (dProx > GESTURE_PROX_DELTA_THRESHOLD) depth = "FORWARD";
    else if (dProx < -GESTURE_PROX_DELTA_THRESHOLD) depth = "BACKWARD";
    return lateral + "," + depth;
  }

  if (dx > GESTURE_ACCEL_DELTA_THRESHOLD) lateral = "RIGHT";
  else if (dx < -GESTURE_ACCEL_DELTA_THRESHOLD) lateral = "LEFT";

  // Final fallback: compare X against long-term baseline, not absolute tilt.
  // This avoids a persistent right/left bias when the puck is held angled.
  if (lateral == "CENTER") {
    if (baseDx > GESTURE_BASELINE_DELTA_THRESHOLD) lateral = "RIGHT";
    else if (baseDx < -GESTURE_BASELINE_DELTA_THRESHOLD) lateral = "LEFT";
  }

  String depth = "STABLE";
  if (dProx > GESTURE_PROX_DELTA_THRESHOLD) depth = "FORWARD";
  else if (dProx < -GESTURE_PROX_DELTA_THRESHOLD) depth = "BACKWARD";

  return lateral + "," + depth;
}

CRGB leds[NUM_LEDS];

bool i2cWriteReg(uint8_t addr, uint8_t reg, uint8_t val) {
  Wire.beginTransmission(addr);
  Wire.write(reg);
  Wire.write(val);
  return (Wire.endTransmission() == 0);
}

bool i2cReadReg(uint8_t addr, uint8_t reg, uint8_t &out) {
  Wire.beginTransmission(addr);
  Wire.write(reg);
  if (Wire.endTransmission(false) != 0) return false;
  Wire.requestFrom((int)addr, 1);
  if (!Wire.available()) return false;
  out = Wire.read();
  return true;
}

bool i2cPing(uint8_t addr) {
  Wire.beginTransmission(addr);
  return (Wire.endTransmission() == 0);
}

// Forward declarations — testLIS3DH() uses this helper, but it's defined
// later in the file alongside the other sensor setup functions.
void setupAccelerometer();

void flashBlueTick() {
  if (millis() - lastBlinkTime < BLINK_INTERVAL_MS) return;
  lastBlinkTime = millis();
  blinkOn = !blinkOn;

  if (blinkOn) {
    for (int i = 0; i < NUM_LEDS; i++) leds[i] = CRGB::Blue;
  } else {
    FastLED.clear();
  }
  FastLED.show();
}

void ledsOff() {
  FastLED.clear();
  FastLED.show();
}

void ledsGreen() {
  for (int i = 0; i < NUM_LEDS; i++) leds[i] = CRGB::Green;
  FastLED.show();
}

void blinkRedThenOff() {
  // Blocking is fine here — the board is about to sleep anyway.
  for (int i = 0; i < 3; i++) {
    for (int j = 0; j < NUM_LEDS; j++) leds[j] = CRGB::Red;
    FastLED.show();
    delay(200);
    FastLED.clear();
    FastLED.show();
    delay(200);
  }
}

void logResult(const char* component, bool pass, const String& detail = "") {
  Serial.print("[");
  Serial.print(pass ? "PASS" : "FAIL");
  Serial.print("] ");
  Serial.print(component);
  if (detail.length()) {
    Serial.print(" — ");
    Serial.print(detail);
  }
  Serial.println();
}

void testLEDs() {
  Serial.println("\n-- LED chain (6x WS2812B, IO5) --");

  for (int i = 0; i < NUM_LEDS; i++) {
    FastLED.clear();
    leds[i] = CRGB::White;
    FastLED.show();
    delay(150);
  }
  FastLED.clear();
  for (int i = 0; i < NUM_LEDS; i++) leds[i] = CRGB::White;
  FastLED.show();
  delay(400);
  FastLED.clear();
  FastLED.show();

  logResult("LEDs", true, "sequenced 1-6 then all-on — visually confirm all 6 lit");
}

// Simple startup jingle — classic "power on" ascending run.
// Frequencies in Hz, 0 = rest. Durations in ms — kept short and light.
const int SONG_NOTES[] = { 523, 659, 784, 1047 }; // C5 E5 G5 C6
const int SONG_DURATIONS[] = { 60, 60, 60, 90 };
const int SONG_LENGTH = sizeof(SONG_NOTES) / sizeof(SONG_NOTES[0]);

// Bit-banged square wave — avoids the ESP32 core's tone()/LEDC peripheral,
// which can throw "LEDC is not initialized" and crash/reboot the board.
// dutyPercent controls loudness: a full square wave (50) is loudest;
// a shorter "on" pulse per cycle (e.g. 25) delivers less average power
// to the buzzer and comes out noticeably quieter.
void playTone(uint8_t pin, int freqHz, int durationMs, int dutyPercent = 50) {
  if (freqHz <= 0) {
    delay(durationMs);
    return;
  }
  unsigned long periodUs = 1000000UL / freqHz;
  unsigned long onUs = (periodUs * dutyPercent) / 100;
  unsigned long offUs = periodUs - onUs;
  unsigned long cycles = (unsigned long)durationMs * 1000UL / periodUs;

  for (unsigned long i = 0; i < cycles; i++) {
    digitalWrite(pin, HIGH);
    delayMicroseconds(onUs);
    digitalWrite(pin, LOW);
    delayMicroseconds(offUs);
  }
}

void testBuzzer() {
  Serial.println("\n-- Buzzer (IO4 -> transistor -> BUZZER2) --");
  pinMode(BUZZ_PIN, OUTPUT);
  digitalWrite(BUZZ_PIN, LOW);

  // ~25% duty instead of 50% — roughly half the average power, audibly quieter
  const int QUIET_DUTY = 25;

  for (int i = 0; i < SONG_LENGTH; i++) {
    playTone(BUZZ_PIN, SONG_NOTES[i], SONG_DURATIONS[i], QUIET_DUTY);
    digitalWrite(BUZZ_PIN, LOW);
    delay(20); // small gap between notes
  }
  digitalWrite(BUZZ_PIN, LOW);

  logResult("Buzzer", true, "played short quiet jingle — visually/audibly confirm notes");
}

void testLIS3DH() {
  Serial.println("\n-- LIS3DH accelerometer (I2C 0x18) --");
  uint8_t whoami = 0;
  bool ok = i2cReadReg(LIS3DH_ADDR, 0x0F, whoami); // WHO_AM_I
  bool idMatch = ok && (whoami == 0x33);
  logResult("LIS3DH present", ok, "addr 0x18");
  logResult("LIS3DH WHO_AM_I", idMatch, "0x" + String(whoami, HEX) + " (expect 0x33)");

  if (idMatch) {
    setupAccelerometer();
    uint8_t xl, xh, yl, yh, zl, zh;
    i2cReadReg(LIS3DH_ADDR, 0x28, xl); i2cReadReg(LIS3DH_ADDR, 0x29, xh);
    i2cReadReg(LIS3DH_ADDR, 0x2A, yl); i2cReadReg(LIS3DH_ADDR, 0x2B, yh);
    i2cReadReg(LIS3DH_ADDR, 0x2C, zl); i2cReadReg(LIS3DH_ADDR, 0x2D, zh);
    int16_t x = (int16_t)((xh << 8) | xl);
    int16_t y = (int16_t)((yh << 8) | yl);
    int16_t z = (int16_t)((zh << 8) | zl);
    Serial.print("  X="); Serial.print(x);
    Serial.print(" Y="); Serial.print(y);
    Serial.print(" Z="); Serial.println(z);
    // Sitting flat, Z should read a large nonzero value (gravity) — sanity check only
    bool plausible = (abs(x) + abs(y) + abs(z)) > 500;
    logResult("LIS3DH reading plausible", plausible, "nonzero accel data present");
  }
}

bool readAccelXYZ(int16_t &x, int16_t &y, int16_t &z) {
  uint8_t xl, xh, yl, yh, zl, zh;
  bool ok = true;
  ok &= i2cReadReg(LIS3DH_ADDR, 0x28, xl); ok &= i2cReadReg(LIS3DH_ADDR, 0x29, xh);
  ok &= i2cReadReg(LIS3DH_ADDR, 0x2A, yl); ok &= i2cReadReg(LIS3DH_ADDR, 0x2B, yh);
  ok &= i2cReadReg(LIS3DH_ADDR, 0x2C, zl); ok &= i2cReadReg(LIS3DH_ADDR, 0x2D, zh);
  x = (int16_t)((xh << 8) | xl);
  y = (int16_t)((yh << 8) | yl);
  z = (int16_t)((zh << 8) | zl);
  return ok;
}

void setupAccelerometer() {
  i2cWriteReg(LIS3DH_ADDR, 0x20, 0x57); // CTRL_REG1: normal mode, 100Hz, XYZ enabled
  delay(50);
}

void stopAccelerometer() {
  // CTRL_REG1 = 0 puts LIS3DH in power-down mode.
  i2cWriteReg(LIS3DH_ADDR, 0x20, 0x00);
}

bool setupProximity() {
  uint8_t id = 0;
  bool idRead = i2cReadReg(APDS9960_ADDR, APDS_ID, id);
  if (!idRead) {
    Serial.println("APDS ID read failed");
    apdsVariant = APDS_VARIANT_UNKNOWN;
    return false;
  }

  if (id == 0xAB || id == 0x9C || id == 0xA8) {
    apdsVariant = APDS_VARIANT_9960;
    Serial.println("APDS variant detected: 9960-family");
  } else if (id == 0x39) {
    apdsVariant = APDS_VARIANT_9930;
    Serial.println("APDS variant detected: 9930-family");
  } else {
    apdsVariant = APDS_VARIANT_UNKNOWN;
    Serial.print("APDS ID unexpected: 0x");
    Serial.println(id, HEX);
    return false;
  }

  bool ok = true;
  ok &= i2cWriteReg(APDS9960_ADDR, APDS_ENABLE, 0x00); // reset
  delay(10);
  ok &= i2cWriteReg(APDS9960_ADDR, APDS_ATIME, 0xDB);   // ~103ms integration time
  ok &= i2cWriteReg(APDS9960_ADDR, APDS_WTIME, 0xF6);   // ~27ms wait time
  ok &= i2cWriteReg(APDS9960_ADDR, APDS_PPULSE, 0x87);  // 8 pulses, 16us pulse length
  if (apdsVariant == APDS_VARIANT_9960) {
    ok &= i2cWriteReg(APDS9960_ADDR, APDS_CONTROL, 0x24); // LDRIVE 100mA, PGAIN 4x, AGAIN 1x
    ok &= i2cWriteReg(APDS9960_ADDR, APDS_CONFIG2, 0x30); // LED boost 300%
  } else {
    // APDS-9930: CONTROL layout differs (PGAIN in [3:2], LED drive in [7:6]).
    // 0x20 gives mid gain with conservative drive for stable testing.
    ok &= i2cWriteReg(APDS9960_ADDR, APDS_CONTROL, 0x20);
  }

  // Explicitly zero the proximity offset registers — some clone chips ship
  // with non-zero garbage here, which can push the analog stage into
  // permanent saturation regardless of gain or LED drive settings.
  if (apdsVariant == APDS_VARIANT_9960) {
    ok &= i2cWriteReg(APDS9960_ADDR, APDS_POFFSET_UR, 0x00);
    ok &= i2cWriteReg(APDS9960_ADDR, APDS_POFFSET_DL, 0x00);
  }

  // Explicitly clear the proximity interrupt/saturation condition — a
  // real datasheet register (write any value to clear), not previously tried.
  if (apdsVariant == APDS_VARIANT_9960) {
    ok &= i2cWriteReg(APDS9960_ADDR, APDS_PICLEAR, 0x00);
  }
  delay(15);

  // PON must stabilize the internal oscillator before other blocks are
  // enabled — setting PON and PEN in the same write can leave the proximity
  // engine started before it's actually ready, and it never produces real
  // readings after that (stuck output, exactly what we were seeing).
  ok &= i2cWriteReg(APDS9960_ADDR, APDS_ENABLE, 0x01); // PON only
  delay(15); // oscillator settle time
  ok &= i2cWriteReg(APDS9960_ADDR, APDS_ENABLE, 0x05); // PON + PEN
  delay(15);

  // Confirm the write actually stuck — if this doesn't read back 0x05,
  // the proximity engine never really turned on, no matter how correct
  // the timing sequence is.
  uint8_t enableReadback = 0;
  ok &= i2cReadReg(APDS9960_ADDR, APDS_ENABLE, enableReadback);
  Serial.print("APDS ENABLE readback: 0x");
  Serial.print(enableReadback, HEX);
  Serial.println(enableReadback == 0x05 ? " (correct)" : " (MISMATCH — write did not stick)");
  return ok && (enableReadback == 0x05);
}

uint8_t readProximity() {
  uint8_t status = 0;
  if (!i2cReadReg(APDS9960_ADDR, APDS_STATUS, status)) {
    return lastProx;
  }

  if ((status & 0x02) == 0) {
    return lastProx;
  }

  uint8_t prox8 = lastProx;

  if (apdsVariant == APDS_VARIANT_9930) {
    uint8_t lo = 0, hi = 0;
    if (!i2cReadReg(APDS9960_ADDR, APDS9930_PDATAL, lo)) return lastProx;
    if (!i2cReadReg(APDS9960_ADDR, APDS9930_PDATAH, hi)) return lastProx;
    uint16_t raw = (uint16_t)(((uint16_t)hi << 8) | lo);
    if (raw > 1023) raw = 1023;
    prox8 = (uint8_t)((raw * 255U) / 1023U);
  } else {
    uint8_t val = 0;
    if (!i2cReadReg(APDS9960_ADDR, APDS_PDATA, val)) return lastProx;
    prox8 = val;
  }

  lastProx = prox8;
  return prox8;
}

void stopProximity() {
  // Disable APDS engines (PON/PEN off) so the sensor is not left running.
  i2cWriteReg(APDS9960_ADDR, APDS_ENABLE, 0x00);
  apdsReady = false;
}

void testAPDS9960() {
  Serial.println("\n-- APDS-9960 gesture sensor (I2C 0x39) --");
  uint8_t id = 0;
  bool ok = i2cReadReg(APDS9960_ADDR, APDS_ID, id); // ID register
  // Known IDs: APDS-9960 family (0xAB/0x9C/0xA8), APDS-9930 family (0x39)
  bool idPlausible = ok && (id == 0xAB || id == 0x9C || id == 0xA8 || id == 0x39);
  logResult("APDS-9960 present", ok, "addr 0x39");
  logResult("APDS-9960 ID plausible", idPlausible, "0x" + String(id, HEX));

  if (!idPlausible) return;

  bool proxOk = setupProximity();
  logResult("APDS-9960 proximity init", proxOk, proxOk ? "engine enabled" : "init failed");
  if (!proxOk) return;

  Serial.println("  Move your hand near/far from the sensor (3s capture)...");
  bool sawGesture = false;
  bool localAboveThreshold = false;
  uint8_t localRiseCount = 0;
  uint8_t localFallCount = 0;
  unsigned long endAt = millis() + 3000;

  while (millis() < endAt) {
    uint8_t prox = readProximity();

    Serial.print("  PDATA=");
    Serial.println(prox);

    if (!localAboveThreshold) {
      if (prox >= PROX_TRIGGER_HIGH) {
        if (localRiseCount < 255) localRiseCount++;
      } else {
        localRiseCount = 0;
      }

      if (localRiseCount >= PROX_DEBOUNCE_SAMPLES) {
        localAboveThreshold = true;
        localRiseCount = 0;
        sawGesture = true;
        Serial.println("  GESTURE DETECTED");
      }
    } else {
      if (prox <= PROX_TRIGGER_LOW) {
        if (localFallCount < 255) localFallCount++;
      } else {
        localFallCount = 0;
      }

      if (localFallCount >= PROX_DEBOUNCE_SAMPLES) {
        localAboveThreshold = false;
        localFallCount = 0;
        Serial.println("  Gesture ended");
      }
    }

    delay(120);
  }

  logResult("APDS-9960 gesture threshold test", sawGesture,
            sawGesture ? "at least one detection occurred" : "no threshold crossing observed");
}

void testInterruptPins() {
  Serial.println("\n-- Interrupt lines (idle-state check only) --");
  pinMode(ACCEL_INT, INPUT_PULLUP);
  pinMode(GEST_INT, INPUT_PULLUP);
  delay(10);
  Serial.print("  ACCEL_INT (IO6) idle level: ");
  Serial.println(digitalRead(ACCEL_INT) ? "HIGH" : "LOW");
  Serial.print("  GEST_INT (IO7) idle level: ");
  Serial.println(digitalRead(GEST_INT) ? "HIGH" : "LOW");
  logResult("Interrupt pins readable", true, "not asserted-state tested, just wiring sanity");
}

void sendBLE(const String &msg) {
  if (bleClientConnected) {
    txCharacteristic->setValue(msg.c_str());
    txCharacteristic->notify();
  }
  Serial.println(msg);
}

class ServerCallbacks : public BLEServerCallbacks {
  void onConnect(BLEServer* s) override {
    bleClientConnected = true;
    waitingForConnection = false;
    lastActivityTime = millis();
    pendingLedState = LED_GREEN; // loop() will apply this — never touch FastLED here
    Serial.println("BLE client connected.");
  }
  void onDisconnect(BLEServer* s) override {
    bleClientConnected = false;
    waitingForConnection = true;
    lastActivityTime = millis(); // restart the connect-timeout clock
    pendingLedState = LED_OFF; // loop() will apply this — never touch FastLED here
    streamingActive = false;
    Serial.println("BLE client disconnected — will re-advertise shortly.");
    needsReadvertise = true;
    readvertiseAt = millis() + READVERTISE_DELAY_MS;
  }
};

class CommandCallbacks : public BLECharacteristicCallbacks {
  void onWrite(BLECharacteristic *c) override {
    String cmd = c->getValue().c_str();
    cmd.trim();
    if (cmd.length() == 0) return;

    // LED commands:
    // LC,<index>,<r>,<g>,<b>  -> set one LED
    // LCA,<r>,<g>,<b>         -> set all LEDs
    // LOFF                    -> clear all LEDs
    if (cmd.startsWith("LC,")) {
      int index = -1, r = -1, g = -1, b = -1;
      if (sscanf(cmd.c_str(), "LC,%d,%d,%d,%d", &index, &r, &g, &b) == 4 &&
          index >= 0 && index < NUM_LEDS &&
          r >= 0 && r <= 255 && g >= 0 && g <= 255 && b >= 0 && b <= 255) {
        ledSetIndex = (uint8_t)index;
        ledSetR = (uint8_t)r;
        ledSetG = (uint8_t)g;
        ledSetB = (uint8_t)b;
        requestLedSetOne = true;
        sendBLE("LED command queued: index " + String(index));
      } else {
        sendBLE("Bad LED command. Use LC,<0-" + String(NUM_LEDS - 1) + ">,<r>,<g>,<b>");
      }
      return;
    }

    if (cmd.startsWith("LCA,")) {
      int r = -1, g = -1, b = -1;
      if (sscanf(cmd.c_str(), "LCA,%d,%d,%d", &r, &g, &b) == 3 &&
          r >= 0 && r <= 255 && g >= 0 && g <= 255 && b >= 0 && b <= 255) {
        ledSetR = (uint8_t)r;
        ledSetG = (uint8_t)g;
        ledSetB = (uint8_t)b;
        requestLedSetAll = true;
        sendBLE("LED command queued: all LEDs");
      } else {
        sendBLE("Bad LED command. Use LCA,<r>,<g>,<b>");
      }
      return;
    }

    if (cmd == "LOFF") {
      requestLedClear = true;
      sendBLE("LED command queued: off");
      return;
    }

    char command = cmd.charAt(0);
    lastActivityTime = millis();

    if (command >= '1' && command <= '5') {
      pendingCommand = command; // loop() will actually run the test
      sendBLE("Command received, running...");
    } else if (command == '6') {
      requestStreamStart = true; // loop() will init sensors and start streaming
      sendBLE("Starting live view...");
    } else if (command == '0') {
      requestStreamStop = true;
      sendBLE("Stopping live view.");
    } else if (command == '7') {
      requestDiag = true; // loop() will run this once and report back
    } else {
      sendBLE("Unknown command. Send 1-7 or 0.");
    }
  }
};

void startBLE() {
  BLEDevice::init(BLE_DEVICE_NAME);
  BLEServer *bleServer = BLEDevice::createServer();
  bleServerPtr = bleServer;
  bleServer->setCallbacks(new ServerCallbacks());

  BLEService *service = bleServer->createService(SERVICE_UUID);

  txCharacteristic = service->createCharacteristic(
      CHAR_UUID_TX, BLECharacteristic::PROPERTY_NOTIFY);
  txCharacteristic->addDescriptor(new BLE2902());

  BLECharacteristic *rxCharacteristic = service->createCharacteristic(
      CHAR_UUID_RX, BLECharacteristic::PROPERTY_WRITE | BLECharacteristic::PROPERTY_WRITE_NR);
  rxCharacteristic->setCallbacks(new CommandCallbacks());

  service->start();

  // Tune connection interval preferences — default BLE timing can cause
  // some phones (notably iPhones) to rapidly reconnect/disconnect.
  bleServer->getAdvertising()->setMinPreferred(0x06);
  bleServer->getAdvertising()->setMaxPreferred(0x12);

  bleServer->getAdvertising()->start();
  bleActive = true;

  Serial.print("BLE advertising as: ");
  Serial.println(BLE_DEVICE_NAME);
  Serial.println("Connect with a BLE UART terminal app (e.g. 'Serial Bluetooth Terminal', 'nRF Connect').");
  Serial.println("Commands: 1=full test, 2=LEDs, 3=buzzer, 4=accel, 5=gesture");
}

void stopBLE() {
  if (!bleActive) return;
  Serial.println("Turning BLE off before sleep.");
  BLEDevice::deinit(true); // true = also release the controller/stack memory
  bleClientConnected = false;
  bleActive = false;
  bleServerPtr = nullptr;
  needsReadvertise = false;
}

bool buttonDown = false;
unsigned long pressStartTime = 0;
bool longPressHandled = false;
const unsigned long LONG_PRESS_MS = 2000;

// Returns true exactly once, the moment a held-down press crosses the
// long-press threshold. A quick tap and release never returns true.
bool longPressJustTriggered() {
  bool isPressed = (digitalRead(WAKE_PIN) == LOW); // active-low, pull-up enabled

  if (isPressed && !buttonDown) {
    buttonDown = true;
    pressStartTime = millis();
    longPressHandled = false;
  } else if (isPressed && buttonDown && !longPressHandled) {
    if (millis() - pressStartTime >= LONG_PRESS_MS) {
      longPressHandled = true;
      return true;
    }
  } else if (!isPressed && buttonDown) {
    buttonDown = false; // released — short taps just reset, no action
  }
  return false;
}

void printWakeReason() {
  esp_sleep_wakeup_cause_t reason = esp_sleep_get_wakeup_cause();
  if (reason == ESP_SLEEP_WAKEUP_EXT0) {
    Serial.println("Woke up: wake button (IO2) pressed");
  } else {
    Serial.println("Boot reason: power-on / reset (not a sleep wakeup)");
  }
}

void goToSleep() {
  Serial.println("-- Entering deep sleep --");
  Serial.println("Press the wake button (IO2) to wake up.");
  Serial.flush();

  requestStreamStart = false;
  requestStreamStop = false;
  requestDiag = false;
  pendingCommand = 0;
  requestLedSetOne = false;
  requestLedSetAll = false;
  requestLedClear = false;

  stopAccelerometer();
  stopProximity();
  stopBLE();
  waitingForConnection = false;
  streamingActive = false;

  // Quiesce everything before sleeping
  FastLED.clear();
  FastLED.show();
  digitalWrite(BUZZ_PIN, LOW);
  pinMode(BUZZ_PIN, INPUT);
  pinMode(LED_PIN, INPUT);
  pinMode(ACCEL_INT, INPUT);
  pinMode(GEST_INT, INPUT);

  // Shut down I2C peripheral after sensor power-down writes complete.
  Wire.end();

  pinMode(WAKE_PIN, INPUT_PULLUP);
  esp_sleep_enable_ext0_wakeup((gpio_num_t)WAKE_PIN, 0); // wake on LOW (button press)

  // Regular digital pull-ups power down during deep sleep — only the RTC
  // domain stays alive. Without this, the wake pin floats and the board
  // wakes itself instantly instead of actually staying asleep.
  rtc_gpio_pullup_en((gpio_num_t)WAKE_PIN);
  rtc_gpio_pulldown_dis((gpio_num_t)WAKE_PIN);

  intentionalSleep = true;
  delay(100);
  esp_deep_sleep_start();
}

void setup() {
  Serial.begin(115200);
  delay(1000);
  Serial.println("======================================");
  Serial.println("   BOOT");
  Serial.println("======================================");
  printWakeReason();

  pinMode(WAKE_PIN, INPUT_PULLUP);
  Wire.begin(SDA_PIN, SCL_PIN);
  Wire.setClock(100000);

  // FastLED strip registered exactly once — calling addLeds() repeatedly
  // (as earlier versions of this file did, in every LED-touching function)
  // leaks memory and eventually crashes the board.
  FastLED.addLeds<WS2812B, LED_PIN, GRB>(leds, NUM_LEDS);
  FastLED.setBrightness(80);
  FastLED.clear();
  FastLED.show();

  esp_sleep_wakeup_cause_t reason = esp_sleep_get_wakeup_cause();
  bool genuineWake = (reason == ESP_SLEEP_WAKEUP_EXT0) && intentionalSleep;
  intentionalSleep = false; // consume the flag either way

  if (!genuineWake) {
    // Cold boot (power applied / reset / just reflashed) — go straight to
    // sleep, no BLE. Also covers a stale wakeup-cause reading right after
    // flashing, which would otherwise falsely look like a button press.
    Serial.println("Cold boot — waiting for button press to turn on.");
    goToSleep();
    return; // unreachable, but explicit
  }

  // Woken by the button — turn BLE on and flash blue while waiting for
  // a connection. No test runs until a BLE command asks for one.
  startBLE();
  waitingForConnection = true;
  lastActivityTime = millis();
  awakeStartTime = millis();

  Serial.println("Waiting for a BLE connection (flashing blue)...");
}

void loop() {
  if (pendingLedState != LED_NONE) {
    if (pendingLedState == LED_GREEN) ledsGreen();
    else if (pendingLedState == LED_OFF) ledsOff();
    pendingLedState = LED_NONE;
  }

  if (requestLedClear) {
    requestLedClear = false;
    FastLED.clear();
    FastLED.show();
    sendBLE("LEDs cleared.");
  }

  if (requestLedSetAll) {
    requestLedSetAll = false;
    CRGB color(ledSetR, ledSetG, ledSetB);
    for (int i = 0; i < NUM_LEDS; i++) {
      leds[i] = color;
    }
    FastLED.show();
    sendBLE("All LEDs set.");
  }

  if (requestLedSetOne) {
    requestLedSetOne = false;
    if (ledSetIndex < NUM_LEDS) {
      leds[ledSetIndex] = CRGB(ledSetR, ledSetG, ledSetB);
      FastLED.show();
      sendBLE("LED " + String(ledSetIndex) + " set.");
    }
  }

  if (pendingCommand != 0) {
    char cmd = pendingCommand;
    pendingCommand = 0;
    switch (cmd) {
      case '1':
        testLEDs(); testBuzzer(); testLIS3DH(); testAPDS9960(); testInterruptPins();
        sendBLE("Full test complete.");
        break;
      case '2':
        testLEDs();
        sendBLE("LEDs done.");
        break;
      case '3':
        testBuzzer();
        sendBLE("Buzzer done.");
        break;
      case '4':
        testLIS3DH();
        sendBLE("Accelerometer read — see log.");
        break;
      case '5':
        testAPDS9960();
        sendBLE("Gesture sensor checked — see log.");
        break;
    }
  }

  if (requestDiag) {
    requestDiag = false;
    bool proxOk = setupProximity();

    uint8_t enableVal = 0, statusVal1 = 0, pdataVal1 = 0;
    i2cReadReg(APDS9960_ADDR, APDS_ENABLE, enableVal);
    i2cReadReg(APDS9960_ADDR, APDS_STATUS, statusVal1);
    i2cReadReg(APDS9960_ADDR, APDS_PDATA, pdataVal1);

    delay(300); // gives you a moment to move your hand between the two reads

    uint8_t statusVal2 = 0, pdataVal2 = 0;
    i2cReadReg(APDS9960_ADDR, APDS_STATUS, statusVal2);
    i2cReadReg(APDS9960_ADDR, APDS_PDATA, pdataVal2);

    bool pgsat1 = statusVal1 & 0x20;
    bool pgsat2 = statusVal2 & 0x20;
    bool changed = (pdataVal1 != pdataVal2);

    String diag = "APDS diag — ENABLE:0x" + String(enableVal, HEX) +
                  " | read1: PDATA:" + String(pdataVal1) + (pgsat1 ? " (SAT)" : "") +
                  " | read2 (300ms later): PDATA:" + String(pdataVal2) + (pgsat2 ? " (SAT)" : "") +
                  " | " + (changed ? "CHANGED — sensor is responding" : "IDENTICAL — still stuck") +
                  " (" + (proxOk ? "enable OK" : "enable FAILED") + ")";
    sendBLE(diag);
  }

  if (requestStreamStart) {
    requestStreamStart = false;
    setupAccelerometer();
    bool proxOk = setupProximity();
    apdsReady = proxOk;
    sameProxCount = 0;
    lastDirectionSent = "";
    lastDirectionSentAt = 0;

    // One-shot diagnostic dump instead of guessing register-by-register —
    // shows exactly what state the chip is actually in right now.
    uint8_t enableVal = 0, statusVal = 0, pdataVal = 0;
    i2cReadReg(APDS9960_ADDR, APDS_ENABLE, enableVal);
    i2cReadReg(APDS9960_ADDR, APDS_STATUS, statusVal);
    i2cReadReg(APDS9960_ADDR, APDS_PDATA, pdataVal);
    String diag = "APDS diag — ENABLE:0x" + String(enableVal, HEX) +
                  " STATUS:0x" + String(statusVal, HEX) +
                  " PDATA:0x" + String(pdataVal, HEX) +
                  " (" + (proxOk ? "enable OK" : "enable FAILED") + ")";
    sendBLE(diag);

    streamingActive = true;
    lastStreamSample = 0; // sample immediately on next pass
    Serial.println("Live streaming started.");
  }

  if (requestStreamStop) {
    requestStreamStop = false;
    streamingActive = false;
    stopProximity();
    proxAboveThreshold = false;
    proxRiseCount = 0;
    proxFallCount = 0;
    gestureWindowActive = false;
    apdsReady = false;
    sameProxCount = 0;
    gestureAccumX = 0;
    gestureAccumY = 0;
    gestureScoreX = 0;
    gestureStepCount = 0;
    gestureMinProx = 255;
    gestureMaxProx = 0;
    havePrevAccel = false;
    lastDirectionSent = "";
    lastDirectionSentAt = 0;
    Serial.println("Live streaming stopped.");
  }

  if (streamingActive && millis() - lastStreamSample >= STREAM_INTERVAL_MS) {
    lastStreamSample = millis();

    int16_t x, y, z;
    if (readAccelXYZ(x, y, z)) {
      if (havePrevAccel) {
        int16_t stepX = x - prevAccelX;
        int16_t stepY = y - prevAccelY;

        if (gestureWindowActive) {
          if (abs(stepX) > GESTURE_ACCEL_STEP_DEADBAND) {
            gestureAccumX += stepX;
            gestureScoreX += stepX;
            if (gestureStepCount < 65535) gestureStepCount++;
          }
          if (abs(stepY) > GESTURE_ACCEL_STEP_DEADBAND) gestureAccumY += stepY;
        }
      }
      prevAccelX = x;
      prevAccelY = y;
      havePrevAccel = true;

      latestAccelX = x;
      latestAccelY = y;
      latestAccelZ = z;
      haveAccelSample = true;

      if (!gestureWindowActive) {
        if (!accelBaseReady) {
          accelBaseX = x;
          accelBaseY = y;
          accelBaseReady = true;
        } else {
          // Slow moving baseline to remove holding-angle bias.
          accelBaseX = (accelBaseX * 63 + x) / 64;
          accelBaseY = (accelBaseY * 63 + y) / 64;
        }
      }

      sendBLE("A," + String(x) + "," + String(y) + "," + String(z));
    }

    if (!apdsReady) {
      apdsReady = setupProximity();
    }

    uint8_t prevProx = lastProx;
    uint8_t prox = readProximity();

    if (prox == prevProx) {
      if (sameProxCount < 255) sameProxCount++;
    } else {
      sameProxCount = 0;
    }

    if (sameProxCount >= PROX_STUCK_LIMIT) {
      sendBLE("APDS appears stuck (constant PDATA). Reinitializing proximity engine...");
      apdsReady = setupProximity();
      sameProxCount = 0;
    }

    sendBLE("P," + String(prox));

    if (gestureWindowActive) {
      if (prox < gestureMinProx) gestureMinProx = prox;
      if (prox > gestureMaxProx) gestureMaxProx = prox;
    }

    if (!proxAboveThreshold) {
      if (prox >= PROX_TRIGGER_HIGH) {
        if (proxRiseCount < 255) proxRiseCount++;
      } else {
        proxRiseCount = 0;
      }

      if (proxRiseCount >= PROX_DEBOUNCE_SAMPLES) {
        proxAboveThreshold = true;
        proxRiseCount = 0;
        gestureWindowActive = true;
        gestureStartAt = millis();
        lastDirectionEmitAt = 0;
        gestureAccumX = 0;
        gestureAccumY = 0;
        gestureScoreX = 0;
        gestureStepCount = 0;
        gestureStartProx = prox;
        gestureMinProx = prox;
        gestureMaxProx = prox;
        if (haveAccelSample) {
          gestureStartX = latestAccelX;
          gestureStartY = latestAccelY;
        }
        sendBLE("G,DETECTED");
      }
    } else {
      if (prox <= PROX_TRIGGER_LOW) {
        if (proxFallCount < 255) proxFallCount++;
      } else {
        proxFallCount = 0;
      }

      if (proxFallCount >= PROX_DEBOUNCE_SAMPLES) {
        proxAboveThreshold = false;
        proxFallCount = 0;
        sendBLE("G,ENDED");

        if (!PROXIMITY_ONLY_MODE && gestureWindowActive && haveAccelSample) {
          int16_t dx = latestAccelX - gestureStartX;
          int16_t dy = latestAccelY - gestureStartY;
          int16_t forwardDelta = (int16_t)gestureMaxProx - (int16_t)gestureStartProx;
          int16_t backwardDelta = (int16_t)gestureStartProx - (int16_t)gestureMinProx;
          int16_t dProx = 0;
          if (forwardDelta >= GESTURE_PROX_DELTA_THRESHOLD && forwardDelta > backwardDelta + GESTURE_PROX_HYSTERESIS) {
            dProx = forwardDelta;
          } else if (backwardDelta >= GESTURE_PROX_DELTA_THRESHOLD && backwardDelta > forwardDelta + GESTURE_PROX_HYSTERESIS) {
            dProx = -backwardDelta;
          }
          int16_t baseDx = accelBaseReady ? (latestAccelX - (int16_t)accelBaseX) : 0;
          int16_t baseDy = accelBaseReady ? (latestAccelY - (int16_t)accelBaseY) : 0;
          String label = inferDirectionLabel(dx, dy, dProx, baseDx, baseDy, gestureScoreX, gestureAccumY);
          if (!isNeutralDirection(label) && shouldSendDirection(label)) {
            sendBLE("D," + label + ",dx=" + String(dx) + ",sx=" + String((int)gestureScoreX) + ",steps=" + String(gestureStepCount) + ",bx=" + String(baseDx) + ",f=" + String(forwardDelta) + ",b=" + String(backwardDelta));
          }
        }
        gestureWindowActive = false;
      }
    }

    if (!PROXIMITY_ONLY_MODE && gestureWindowActive && (millis() - gestureStartAt > GESTURE_WINDOW_TIMEOUT_MS)) {
      if (haveAccelSample) {
        int16_t dx = latestAccelX - gestureStartX;
        int16_t dy = latestAccelY - gestureStartY;
        int16_t forwardDelta = (int16_t)gestureMaxProx - (int16_t)gestureStartProx;
        int16_t backwardDelta = (int16_t)gestureStartProx - (int16_t)gestureMinProx;
        int16_t dProx = 0;
        if (forwardDelta >= GESTURE_PROX_DELTA_THRESHOLD && forwardDelta > backwardDelta + GESTURE_PROX_HYSTERESIS) {
          dProx = forwardDelta;
        } else if (backwardDelta >= GESTURE_PROX_DELTA_THRESHOLD && backwardDelta > forwardDelta + GESTURE_PROX_HYSTERESIS) {
          dProx = -backwardDelta;
        }
        int16_t baseDx = accelBaseReady ? (latestAccelX - (int16_t)accelBaseX) : 0;
        int16_t baseDy = accelBaseReady ? (latestAccelY - (int16_t)accelBaseY) : 0;
        String label = inferDirectionLabel(dx, dy, dProx, baseDx, baseDy, gestureScoreX, gestureAccumY);
        if (!isNeutralDirection(label) && shouldSendDirection(label)) {
          sendBLE("D," + label + ",dx=" + String(dx) + ",sx=" + String((int)gestureScoreX) + ",steps=" + String(gestureStepCount) + ",bx=" + String(baseDx) + ",f=" + String(forwardDelta) + ",b=" + String(backwardDelta));
        }
      }
      gestureWindowActive = false;
    }
  }

  if (needsReadvertise && millis() >= readvertiseAt) {
    needsReadvertise = false;
    if (bleServerPtr != nullptr) {
      bleServerPtr->getAdvertising()->start();
      Serial.println("Re-advertising now.");
    }
  }

  if (waitingForConnection) {
    flashBlueTick();

    if (longPressJustTriggered()) {
      Serial.println("Long press detected — blinking red, then turning off.");
      blinkRedThenOff();
      goToSleep();
      return;
    }

    if (millis() - awakeStartTime > SESSION_AUTO_SLEEP_MS) {
      Serial.println("One-hour session timeout reached — blinking red, then sleeping.");
      blinkRedThenOff();
      goToSleep();
    }
    return;
  }

  // Connected and idle — a long press (2s hold) turns off. A quick tap
  // does nothing here; use BLE commands (1-5) to run tests while connected.
  if (longPressJustTriggered()) {
    Serial.println("Long press detected — blinking red, then turning off.");
    blinkRedThenOff();
    goToSleep();
    return;
  }

  if (millis() - awakeStartTime > SESSION_AUTO_SLEEP_MS) {
    Serial.println("One-hour session timeout reached — blinking red, then sleeping.");
    goToSleep();
  }
}