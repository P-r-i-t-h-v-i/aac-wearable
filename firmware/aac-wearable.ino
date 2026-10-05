#include <LSM6DS3.h>
#include <Wire.h>
#include <ArduinoBLE.h>
#include <PDM.h>
#include "FlashIAP.h"

// Initialize internal IMU on Wire1 (0x6A)
LSM6DS3 myIMU(I2C_MODE, 0x6A);

#define PIN_AMP_SD   D3
#define PIN_LED_BLUE LED_BUILTIN

// =========================================================
// COMMUNICATION MESSAGES
// Pain and Seizure keep dedicated always-on detectors (double tap /
// sustained shake). The other five are matched against motion templates
// the caregiver records, so each one is whatever movement they choose.
// =========================================================
enum MsgId {
  MSG_PAIN = 0, MSG_HUNGER = 1, MSG_TOILET = 2, MSG_SLEEP = 3,
  MSG_SEIZURE = 4, MSG_YES = 5, MSG_NO = 6
};
#define MSG_COUNT 7

const char* MSG_NAMES[MSG_COUNT] = {
  "PAIN", "HUNGER", "TOILET", "SLEEP", "SEIZURE", "YES", "NO"
};

// Slots driven by recorded templates (Pain/Seizure use fixed detectors)
bool isTemplateSlot(int id) {
  return id != MSG_PAIN && id != MSG_SEIZURE;
}

// =========================================================
// FLASH STORAGE MAP
// App region is 0x27000..0xED000. The sketch lives at the bottom; voice
// clips and gesture templates are reserved at the top. Keep the compiled
// sketch under AUDIO_BASE_ADDR or it will collide with stored clips.
// =========================================================
#define AUDIO_BASE_ADDR     0xA0000
#define AUDIO_SLOT_SIZE     40960        // 10 x 4KB pages, 2.5s @ 8kHz PCM16
#define AUDIO_HEADER_BYTES  8
#define AUDIO_MAGIC         0xA1C0DE01
#define AUDIO_MAX_SAMPLES   ((AUDIO_SLOT_SIZE - AUDIO_HEADER_BYTES) / 2)

#define TEMPLATE_ADDR       0xE6000      // single 4KB page holds all templates
#define TEMPLATE_MAGIC      0x6E5701

mbed::FlashIAP g_flash;
bool g_flashReady = false;

// =========================================================
// GESTURE TEMPLATES
// 64 samples @ 50Hz = 1.28s of 6-axis motion, z-normalized per axis.
// =========================================================
#define GEST_LEN   64
#define GEST_AXES  6
#define GEST_BAND  8     // Sakoe-Chiba band for DTW

struct GestureTemplate {
  uint32_t magic;
  int16_t data[GEST_LEN * GEST_AXES];
};

GestureTemplate g_templates[MSG_COUNT];
float g_gestureThreshold = 3.0;          // tune via "GT,<value>"

// Live gesture capture state
bool  g_capturing = false;
int   g_captureIdx = 0;
int16_t g_captureBuf[GEST_LEN * GEST_AXES];
unsigned long g_lastTriggerTime = 0;
const unsigned long GESTURE_COOLDOWN_MS = 2000;
const float GESTURE_TRIGGER_DPS = 120.0;  // motion energy needed to start capture

int16_t g_testAmplitude = 32767;

// =========================================================
// PAIN (DOUBLE TAP) + SEIZURE (SHAKE) DETECTORS
// =========================================================
unsigned long shakeStartTime = 0;
unsigned long lastShakeTime = 0;
const float SHAKE_THRESHOLD_DPS = 300.0;
const unsigned long REQUIRED_SHAKE_DURATION_MS = 3000;

int tapCount = 0;
unsigned long lastTapTime = 0;
const float TAP_THRESHOLD_G = 2.5;

// Audio Buffer
#define AUDIO_BUF_SIZE 1024
uint32_t i2s_buf_0[AUDIO_BUF_SIZE];
uint32_t i2s_buf_1[AUDIO_BUF_SIZE];

// =========================================================
// BLE SERVICE
// =========================================================
BLEService calService("a5e7c000-9c3f-4a4e-8e1e-1a2b3c4d5e00");
BLEStringCharacteristic commandChar("a5e7c000-9c3f-4a4e-8e1e-1a2b3c4d5e01", BLEWrite, 32);
BLEStringCharacteristic statusChar("a5e7c000-9c3f-4a4e-8e1e-1a2b3c4d5e02", BLERead | BLENotify, 200);

// =========================================================
// VOICE RECORDING (onboard PDM microphone)
// Audio is captured by the device itself rather than transferred from the
// phone. Pushing 40KB over BLE exhausted the link's buffers and dropped the
// connection mid-upload; here BLE only carries a short "RECV,<slot>" command,
// so there is no bulk transfer to fail. The board's mic makes this both
// simpler and more reliable.
// =========================================================
#define REC_MAX_SAMPLES  20000      // 2.5s @ 8kHz
#define REC_TIMEOUT_MS   4000

// nRF52 PDM gain register: 0x00..0x50, 0.5dB per step, 0x28 = 0dB. The PDM
// library's own default is 20 (-10dB), which is well below even the chip's
// default and far below the 0x50 (+20dB) ceiling - a wristband mic sits a
// good distance from the mouth, so run it near the top. Adjustable live with
// "MG,<0-80>" if it ever distorts.
// Measured on this board: at 0x4A (+17dB) quiet-room ambient alone already
// peaked at ~14800, so close speech would have clipped the ADC. 0x28 (0dB)
// puts ambient near ~2000 and leaves room for speech to approach full scale;
// the loudness normalization below does the rest.
#define PDM_GAIN_MAX     0x50
int   g_micGain = 0x28;
float REC_TARGET_RMS = 0.22f;       // tune with "MR,<0.05-0.5>"

int16_t  g_recBuf[REC_MAX_SAMPLES];
volatile uint32_t g_recCount = 0;
volatile bool g_recording = false;
int16_t  g_pdmBuf[256];
bool     g_pdmHavePending = false;
int16_t  g_pdmPending = 0;

int      g_recPendingSlot = -1;     // slot to record once its erase finishes
int      g_recSlot = -1;
unsigned long g_recStart = 0;

// Flash write state - programming 40KB in one call would block interrupts
// for ~400ms, so it is spread over loop() iterations like the erase.
bool     g_flushing = false;
uint32_t g_flushAddr = 0;
uint32_t g_flushOffset = 0;
#define  FLUSH_CHUNK_BYTES 512

// This board's mbed core has no SoftDevice (TARGET_SOFTDEVICE_NONE) - flash
// erase/program bangs the NVMC controller directly and disables interrupts
// for the whole call. A single 40KB slot erase blocks the BLE radio's
// interrupts for ~850ms (10 pages), long enough to miss connection events
// and get disconnected. Erasing one page per loop() iteration keeps each
// blocked window to ~85ms and lets BLE.poll() run in between.
bool     g_erasing = false;
uint32_t g_eraseAddr = 0;
int      g_erasePagesLeft = 0;

void logStatus(const String &msg) {
  Serial.println(msg);
  if (BLE.connected()) {
    statusChar.writeValue(msg);
  }
}

void setup() {
  Serial.begin(115200);
  pinMode(PIN_LED_BLUE, OUTPUT);
  digitalWrite(PIN_LED_BLUE, HIGH);

  pinMode(PIN_AMP_SD, OUTPUT);
  digitalWrite(PIN_AMP_SD, LOW);

  pinMode(PIN_LSM6DS3TR_C_POWER, OUTPUT);
  digitalWrite(PIN_LSM6DS3TR_C_POWER, HIGH);
  delay(20);

  Wire1.begin();
  if (myIMU.begin() != 0) {
    Serial.println("[ERROR] IMU not detected!");
    while (1);
  }

  initDirectI2S();

  if (g_flash.init() == 0) {
    g_flashReady = true;
  } else {
    Serial.println("[ERROR] Flash init failed - recordings unavailable");
  }
  loadTemplates();

  if (!BLE.begin()) {
    Serial.println("[ERROR] BLE init failed!");
  } else {
    BLE.setLocalName("AAC-Wearable");
    BLE.setAdvertisedService(calService);
    calService.addCharacteristic(commandChar);
    calService.addCharacteristic(statusChar);
    BLE.addService(calService);
    statusChar.writeValue("READY");
    BLE.advertise();
  }

  PDM.onReceive(onPDMdata);

  Serial.println("\n==================================================");
  Serial.println("  GESTURE AAC WRISTBAND ACTIVE");
  Serial.println("  * 7 messages: PAIN HUNGER TOILET SLEEP");
  Serial.println("                SEIZURE YES NO");
  Serial.println("  * Pain = two taps | Seizure = shake 3s");
  Serial.println("  * Others = recorded motion templates");
  Serial.println("  * Voice recorded on-device via onboard mic");
  Serial.println("  * BLE service advertising");
  Serial.println("==================================================");
}

void loop() {
  BLE.poll();

  if (Serial.available() > 0) {
    handleCommand(Serial.readStringUntil('\n'));
  }
  if (commandChar.written()) {
    handleCommand(commandChar.value());
  }

  // Erase one flash page per iteration (see g_erasing comment above) so
  // BLE.poll() runs between pages instead of blocking through all of them.
  if (g_erasing) {
    uint32_t sector = g_flash.get_sector_size(g_eraseAddr);
    g_flash.erase(g_eraseAddr, sector);
    g_eraseAddr += sector;
    g_erasePagesLeft--;

    if (g_erasePagesLeft <= 0) {
      g_erasing = false;
      if (g_recPendingSlot >= 0) {
        startRecording(g_recPendingSlot);
        g_recPendingSlot = -1;
      }
    }
    return;
  }

  // Recording runs off the PDM interrupt; just watch for the end condition.
  if (g_recording) {
    if (g_recCount >= REC_MAX_SAMPLES || (millis() - g_recStart) > REC_TIMEOUT_MS) {
      stopRecording();
    }
    return;
  }

  // Write the captured clip to flash a chunk at a time, same reasoning as
  // the incremental erase.
  if (g_flushing) {
    serviceFlush();
    return;
  }

  float ax = myIMU.readFloatAccelX();
  float ay = myIMU.readFloatAccelY();
  float az = myIMU.readFloatAccelZ();
  float gx = myIMU.readFloatGyroX();
  float gy = myIMU.readFloatGyroY();
  float gz = myIMU.readFloatGyroZ();

  // --- PAIN: two sharp taps ---
  float accel_magnitude = sqrt(ax*ax + ay*ay + az*az);
  if (accel_magnitude > TAP_THRESHOLD_G) {
    if (millis() - lastTapTime > 250) {
      tapCount++;
      lastTapTime = millis();
      if (tapCount == 2) {
        tapCount = 0;
        playMessage(MSG_PAIN);
        g_lastTriggerTime = millis();
        return;
      }
    }
  }
  if (tapCount == 1 && (millis() - lastTapTime > 1500)) {
    tapCount = 0;
  }

  // --- SEIZURE: sustained vigorous shaking ---
  float gyro_magnitude = abs(gx) + abs(gy) + abs(gz);
  if (gyro_magnitude > SHAKE_THRESHOLD_DPS) {
    if (shakeStartTime == 0) {
      shakeStartTime = millis();
    }
    lastShakeTime = millis();
  }
  if (shakeStartTime > 0 && (millis() - lastShakeTime > 500)) {
    shakeStartTime = 0;
  }
  if (shakeStartTime > 0 && (millis() - shakeStartTime > REQUIRED_SHAKE_DURATION_MS)) {
    shakeStartTime = 0;
    playMessage(MSG_SEIZURE);
    g_lastTriggerTime = millis();
    return;
  }

  // --- TEMPLATE GESTURES ---
  if (g_capturing) {
    int base = g_captureIdx * GEST_AXES;
    g_captureBuf[base + 0] = (int16_t)(ax * 1000);
    g_captureBuf[base + 1] = (int16_t)(ay * 1000);
    g_captureBuf[base + 2] = (int16_t)(az * 1000);
    g_captureBuf[base + 3] = (int16_t)gx;
    g_captureBuf[base + 4] = (int16_t)gy;
    g_captureBuf[base + 5] = (int16_t)gz;
    g_captureIdx++;

    if (g_captureIdx >= GEST_LEN) {
      g_capturing = false;
      digitalWrite(PIN_LED_BLUE, HIGH);
      int match = matchGesture();
      if (match >= 0) {
        playMessage(match);
        g_lastTriggerTime = millis();
      }
    }
  } else {
    bool cooling = (millis() - g_lastTriggerTime) < GESTURE_COOLDOWN_MS;
    if (!cooling && shakeStartTime == 0 && gyro_magnitude > GESTURE_TRIGGER_DPS) {
      g_capturing = true;
      g_captureIdx = 0;
      digitalWrite(PIN_LED_BLUE, LOW);
    }
  }

  delay(20);
}

// =========================================================
// COMMAND DISPATCH (shared by USB serial and BLE)
// =========================================================
void handleCommand(String input) {
  input.trim();
  if (input.length() == 0) return;

  int slot; float f;

  if (input == "LIST") {
    listMessages();
  } else if (sscanf(input.c_str(), "RECG,%d", &slot) == 1) {
    recordGesture(slot);
  } else if (sscanf(input.c_str(), "RECV,%d", &slot) == 1) {
    beginVoiceRecording(slot);
  } else if (sscanf(input.c_str(), "PLAY,%d", &slot) == 1) {
    if (slot >= 0 && slot < MSG_COUNT) playMessage(slot);
  } else if (sscanf(input.c_str(), "DEL,%d", &slot) == 1) {
    deleteMessage(slot);
  } else if (sscanf(input.c_str(), "MG,%d", &slot) == 1) {
    g_micGain = constrain(slot, 0, PDM_GAIN_MAX);
    logStatus("[OK] Mic gain = " + String(g_micGain) +
              " (" + String((g_micGain - 0x28) * 0.5f, 1) + "dB)");
  } else if (sscanf(input.c_str(), "MR,%f", &f) == 1) {
    REC_TARGET_RMS = constrain(f, 0.05f, 0.5f);
    logStatus("[OK] Target loudness = " + String(REC_TARGET_RMS, 2));
  } else if (sscanf(input.c_str(), "GT,%f", &f) == 1) {
    g_gestureThreshold = f;
    logStatus("[OK] Gesture threshold = " + String(g_gestureThreshold, 2));
  } else if (sscanf(input.c_str(), "V,%d", &slot) == 1) {
    g_testAmplitude = constrain(slot, 0, 32767);
    logStatus("[OK] Test amplitude = " + String(g_testAmplitude));
  } else if (input == "T") {
    ampOn();
    playToneI2S(440, 1000);
    ampOff();
  } else if (input == "E") {
    playMessage(MSG_SEIZURE);
  }
}

void listMessages() {
  for (int i = 0; i < MSG_COUNT; i++) {
    String line = String(i) + " " + MSG_NAMES[i];
    line += audioLength(i) > 0 ? " voice:YES" : " voice:--";
    if (isTemplateSlot(i)) {
      line += g_templates[i].magic == TEMPLATE_MAGIC ? " gesture:YES" : " gesture:--";
    } else {
      line += (i == MSG_PAIN) ? " gesture:2-TAP" : " gesture:SHAKE";
    }
    logStatus(line);
  }
}

// =========================================================
// GESTURE RECORDING + MATCHING
// =========================================================
void recordGesture(int slot) {
  if (slot < 0 || slot >= MSG_COUNT || !isTemplateSlot(slot)) {
    logStatus("[ERROR] Slot has a fixed gesture (tap/shake)");
    return;
  }

  logStatus(String("[REC] ") + MSG_NAMES[slot] + " gesture in 2s...");
  delay(2000);
  calibrateBeep(1);
  logStatus("[REC] Perform the movement now (1.3s)");
  digitalWrite(PIN_LED_BLUE, LOW);

  for (int i = 0; i < GEST_LEN; i++) {
    int base = i * GEST_AXES;
    g_templates[slot].data[base + 0] = (int16_t)(myIMU.readFloatAccelX() * 1000);
    g_templates[slot].data[base + 1] = (int16_t)(myIMU.readFloatAccelY() * 1000);
    g_templates[slot].data[base + 2] = (int16_t)(myIMU.readFloatAccelZ() * 1000);
    g_templates[slot].data[base + 3] = (int16_t)myIMU.readFloatGyroX();
    g_templates[slot].data[base + 4] = (int16_t)myIMU.readFloatGyroY();
    g_templates[slot].data[base + 5] = (int16_t)myIMU.readFloatGyroZ();
    delay(20);
  }
  g_templates[slot].magic = TEMPLATE_MAGIC;

  digitalWrite(PIN_LED_BLUE, HIGH);
  calibrateBeep(2);
  saveTemplates();
  logStatus(String("[REC] SAVED gesture for ") + MSG_NAMES[slot]);
}

// Z-normalize each axis so matching is about motion shape, not amplitude.
void normalizeWindow(const int16_t* src, float* dst) {
  for (int a = 0; a < GEST_AXES; a++) {
    float mean = 0;
    for (int i = 0; i < GEST_LEN; i++) mean += src[i * GEST_AXES + a];
    mean /= GEST_LEN;

    float var = 0;
    for (int i = 0; i < GEST_LEN; i++) {
      float d = src[i * GEST_AXES + a] - mean;
      var += d * d;
    }
    float sd = sqrt(var / GEST_LEN);
    if (sd < 1.0) sd = 1.0;

    for (int i = 0; i < GEST_LEN; i++) {
      dst[i * GEST_AXES + a] = (src[i * GEST_AXES + a] - mean) / sd;
    }
  }
}

// Banded DTW; returns mean per-step distance.
float dtwDistance(const float* a, const float* b) {
  static float prev[GEST_LEN];
  static float curr[GEST_LEN];

  for (int j = 0; j < GEST_LEN; j++) prev[j] = 1e9;
  prev[0] = 0;

  for (int i = 1; i < GEST_LEN; i++) {
    for (int j = 0; j < GEST_LEN; j++) curr[j] = 1e9;

    int lo = max(1, i - GEST_BAND);
    int hi = min(GEST_LEN - 1, i + GEST_BAND);

    for (int j = lo; j <= hi; j++) {
      float cost = 0;
      for (int k = 0; k < GEST_AXES; k++) {
        float d = a[i * GEST_AXES + k] - b[j * GEST_AXES + k];
        cost += d * d;
      }
      cost = sqrt(cost);

      float best = prev[j];
      if (prev[j - 1] < best) best = prev[j - 1];
      if (curr[j - 1] < best) best = curr[j - 1];
      curr[j] = cost + best;
    }
    for (int j = 0; j < GEST_LEN; j++) prev[j] = curr[j];
  }
  return prev[GEST_LEN - 1] / GEST_LEN;
}

int matchGesture() {
  static float liveNorm[GEST_LEN * GEST_AXES];
  static float tmplNorm[GEST_LEN * GEST_AXES];

  normalizeWindow(g_captureBuf, liveNorm);

  int bestSlot = -1;
  float bestDist = 1e9;

  for (int i = 0; i < MSG_COUNT; i++) {
    if (!isTemplateSlot(i) || g_templates[i].magic != TEMPLATE_MAGIC) continue;
    normalizeWindow(g_templates[i].data, tmplNorm);
    float d = dtwDistance(liveNorm, tmplNorm);
    if (d < bestDist) {
      bestDist = d;
      bestSlot = i;
    }
  }

  if (bestSlot >= 0) {
    logStatus(String("[GESTURE] best=") + MSG_NAMES[bestSlot] +
              " dist=" + String(bestDist, 2) +
              (bestDist <= g_gestureThreshold ? " MATCH" : " (no match)"));
    if (bestDist <= g_gestureThreshold) return bestSlot;
  }
  return -1;
}

// =========================================================
// FLASH: TEMPLATES
// =========================================================
void loadTemplates() {
  if (!g_flashReady) return;
  for (int i = 0; i < MSG_COUNT; i++) g_templates[i].magic = 0;

  const uint8_t* p = (const uint8_t*)TEMPLATE_ADDR;
  for (int i = 0; i < MSG_COUNT; i++) {
    const GestureTemplate* t = (const GestureTemplate*)(p + i * sizeof(GestureTemplate));
    if (t->magic == TEMPLATE_MAGIC) {
      memcpy(&g_templates[i], t, sizeof(GestureTemplate));
    }
  }
}

void saveTemplates() {
  if (!g_flashReady) return;
  uint32_t sector = g_flash.get_sector_size(TEMPLATE_ADDR);
  if (g_flash.erase(TEMPLATE_ADDR, sector) != 0) {
    logStatus("[ERROR] template erase failed");
    return;
  }
  uint32_t total = sizeof(GestureTemplate) * MSG_COUNT;
  if (g_flash.program(g_templates, TEMPLATE_ADDR, total) != 0) {
    logStatus("[ERROR] template write failed");
  }
}

// =========================================================
// FLASH: VOICE CLIPS
// =========================================================
uint32_t slotAddr(int slot) {
  return AUDIO_BASE_ADDR + (uint32_t)slot * AUDIO_SLOT_SIZE;
}

uint32_t audioLength(int slot) {
  if (slot < 0 || slot >= MSG_COUNT) return 0;
  const uint32_t* hdr = (const uint32_t*)slotAddr(slot);
  if (hdr[0] != AUDIO_MAGIC) return 0;
  uint32_t n = hdr[1];
  return (n > AUDIO_MAX_SAMPLES) ? 0 : n;
}

// Called from the PDM interrupt. The mic runs at 16kHz; averaging sample
// pairs decimates to the 8kHz the playback path and flash budget expect,
// and the averaging doubles as a crude anti-alias filter.
void onPDMdata() {
  int bytes = PDM.available();
  if (bytes > (int)sizeof(g_pdmBuf)) bytes = sizeof(g_pdmBuf);
  if (bytes <= 0) return;
  PDM.read(g_pdmBuf, bytes);

  if (!g_recording) return;

  int n = bytes / 2;
  for (int i = 0; i < n && g_recCount < REC_MAX_SAMPLES; i++) {
    if (!g_pdmHavePending) {
      g_pdmPending = g_pdmBuf[i];
      g_pdmHavePending = true;
    } else {
      g_recBuf[g_recCount++] = (int16_t)(((int32_t)g_pdmPending + g_pdmBuf[i]) / 2);
      g_pdmHavePending = false;
    }
  }
}

// "RECV,<slot>" - erase the slot first (spread over loop()), then record.
void beginVoiceRecording(int slot) {
  if (!g_flashReady) { logStatus("[ERROR] flash unavailable"); return; }
  if (slot < 0 || slot >= MSG_COUNT) { logStatus("[ERROR] bad slot"); return; }
  if (g_recording || g_erasing || g_flushing) {
    logStatus("[ERROR] busy, try again");
    return;
  }

  g_recPendingSlot = slot;
  g_erasing = true;
  g_eraseAddr = slotAddr(slot);
  g_erasePagesLeft = AUDIO_SLOT_SIZE / g_flash.get_sector_size(g_eraseAddr);

  logStatus(String("[REC] ") + MSG_NAMES[slot] + " - get ready...");
}

void startRecording(int slot) {
  g_recSlot = slot;
  g_recCount = 0;
  g_pdmHavePending = false;

  calibrateBeep(1);              // beep = speak now
  digitalWrite(PIN_LED_BLUE, LOW);

  // Let the speaker's beep die away before opening the mic.
  delay(150);

  PDM.setGain(g_micGain);
  if (!PDM.begin(1, 16000)) {
    logStatus("[ERROR] microphone failed to start");
    digitalWrite(PIN_LED_BLUE, HIGH);
    g_recSlot = -1;
    return;
  }

  // The PDM decimation filter emits a full-scale transient while it settles.
  // g_recording is still false here, so onPDMdata() drains and discards it;
  // without this the clip starts clipped and normalization is thrown off.
  delay(250);

  g_recStart = millis();
  g_recording = true;
  logStatus(String("[REC] SPEAK NOW for ") + MSG_NAMES[slot] + " (2.5s)");
}

void stopRecording() {
  g_recording = false;
  PDM.end();
  digitalWrite(PIN_LED_BLUE, HIGH);

  uint32_t n = g_recCount;
  if (n < 1000) {
    logStatus("[REC] too short / no audio captured");
    g_recSlot = -1;
    return;
  }

  // Normalize to a target loudness rather than just peak, so a single click
  // can't leave the speech itself quiet. Peaks past the knee are rounded off
  // smoothly - hard clipping a fricative turns it into a harsh burst.
  int32_t peak = 1;
  for (uint32_t i = 0; i < n; i++) {
    int32_t a = abs(g_recBuf[i]);
    if (a > peak) peak = a;
  }
  float toUnit = 1.0f / (float)peak;

  double acc = 0;
  for (uint32_t i = 0; i < n; i++) {
    float v = g_recBuf[i] * toUnit;
    acc += (double)v * v;
  }
  float rms = sqrt(acc / (double)n);

  float gain = (rms > 0.0001f) ? (REC_TARGET_RMS / rms) : 1.0f;
  if (gain > 60.0f) gain = 60.0f;

  const float knee = 0.5f;
  for (uint32_t i = 0; i < n; i++) {
    float v = g_recBuf[i] * toUnit * gain;
    float a = fabsf(v);
    if (a > knee) a = knee + (1.0f - knee) * tanhf((a - knee) / (1.0f - knee));
    float out = (v < 0 ? -a : a) * 32000.0f;
    if (out > 32767.0f) out = 32767.0f;
    if (out < -32768.0f) out = -32768.0f;
    g_recBuf[i] = (int16_t)out;
  }

  // Hand off to the incremental flash writer in loop().
  g_flushing = true;
  g_flushOffset = 0;
  g_flushAddr = slotAddr(g_recSlot) + AUDIO_HEADER_BYTES;

  // Raw levels, so a still-quiet recording can be diagnosed as mic-side
  // (low peak) rather than normalization-side.
  logStatus(String("[REC] captured ") + String(n) + " samples (raw peak " +
            String(peak) + ", gain x" + String(gain, 1) + "), saving...");
}

void serviceFlush() {
  uint32_t total = g_recCount * 2;
  uint32_t remaining = total - g_flushOffset;
  uint32_t chunk = remaining > FLUSH_CHUNK_BYTES ? FLUSH_CHUNK_BYTES : remaining;

  // Flash programming needs a 4-byte aligned length
  uint32_t aligned = (chunk + 3) & ~3u;

  g_flash.program(((const uint8_t*)g_recBuf) + g_flushOffset, g_flushAddr, aligned);
  g_flushAddr += aligned;
  g_flushOffset += chunk;

  if (g_flushOffset >= total) {
    uint32_t hdr[2] = { AUDIO_MAGIC, g_recCount };
    g_flash.program(hdr, slotAddr(g_recSlot), sizeof(hdr));

    g_flushing = false;
    calibrateBeep(2);            // two beeps = saved
    logStatus(String("[REC] SAVED ") + MSG_NAMES[g_recSlot] + " " +
              String(total) + " bytes");
    g_recSlot = -1;
  }
}

void deleteMessage(int slot) {
  if (!g_flashReady || slot < 0 || slot >= MSG_COUNT) return;
  g_flash.erase(slotAddr(slot), AUDIO_SLOT_SIZE);
  g_templates[slot].magic = 0;
  saveTemplates();
  logStatus(String("[OK] Cleared ") + MSG_NAMES[slot]);
}

// =========================================================
// PLAYBACK
// =========================================================
void ampOn() {
  digitalWrite(PIN_AMP_SD, HIGH);
  digitalWrite(PIN_LED_BLUE, LOW);
  delay(50);
}

void ampOff() {
  digitalWrite(PIN_AMP_SD, LOW);
  digitalWrite(PIN_LED_BLUE, HIGH);
}

void playMessage(int slot) {
  if (slot < 0 || slot >= MSG_COUNT) return;
  logStatus(String(">>> ") + MSG_NAMES[slot]);

  uint32_t n = audioLength(slot);
  ampOn();
  if (n > 0) {
    // nRF52 flash is memory-mapped, so the clip streams straight from flash
    const int16_t* pcm = (const int16_t*)(slotAddr(slot) + AUDIO_HEADER_BYTES);
    playPCMI2S(pcm, n);
  } else {
    // No recording yet - fall back to an alert pattern so it still signals
    if (slot == MSG_SEIZURE) {
      for (int i = 0; i < 6; i++) { playToneI2S(1400, 400); playToneI2S(900, 400); }
    } else if (slot == MSG_PAIN) {
      playToneI2S(1200, 2000);
    } else {
      for (int i = 0; i <= slot; i++) { playToneI2S(1000, 150); delay(100); }
    }
  }
  ampOff();
}

void calibrateBeep(int count) {
  digitalWrite(PIN_AMP_SD, HIGH);
  delay(50);
  for (int i = 0; i < count; i++) {
    playToneI2S(1800, 120);
    if (i < count - 1) delay(80);
  }
  digitalWrite(PIN_AMP_SD, LOW);
}

// =========================================================
// DIRECT HARDWARE I2S DMA (8 kHz)
// =========================================================
void initDirectI2S() {
  NRF_I2S->ENABLE = 0;
  NRF_I2S->PSEL.MCK   = 0xFFFFFFFF;
  NRF_I2S->PSEL.SCK   = (uint32_t)digitalPinToPinName(D8);
  NRF_I2S->PSEL.LRCK  = (uint32_t)digitalPinToPinName(D9);
  NRF_I2S->PSEL.SDOUT = (uint32_t)digitalPinToPinName(D10);
  NRF_I2S->PSEL.SDIN  = 0xFFFFFFFF;

  NRF_I2S->CONFIG.MODE      = (I2S_CONFIG_MODE_MODE_Master << I2S_CONFIG_MODE_MODE_Pos);
  NRF_I2S->CONFIG.RXEN      = (I2S_CONFIG_RXEN_RXEN_Disabled << I2S_CONFIG_RXEN_RXEN_Pos);
  NRF_I2S->CONFIG.TXEN      = (I2S_CONFIG_TXEN_TXEN_Enabled << I2S_CONFIG_TXEN_TXEN_Pos);
  NRF_I2S->CONFIG.MCKEN     = (I2S_CONFIG_MCKEN_MCKEN_Enabled << I2S_CONFIG_MCKEN_MCKEN_Pos);
  NRF_I2S->CONFIG.MCKFREQ   = (I2S_CONFIG_MCKFREQ_MCKFREQ_32MDIV63 << I2S_CONFIG_MCKFREQ_MCKFREQ_Pos);
  // 32MHz/63/64 = 7936Hz ~= 8kHz (same 0.8% tolerance the 16kHz setup used)
  NRF_I2S->CONFIG.RATIO     = (I2S_CONFIG_RATIO_RATIO_64X << I2S_CONFIG_RATIO_RATIO_Pos);
  NRF_I2S->CONFIG.SWIDTH    = (I2S_CONFIG_SWIDTH_SWIDTH_16Bit << I2S_CONFIG_SWIDTH_SWIDTH_Pos);
  NRF_I2S->CONFIG.ALIGN     = (I2S_CONFIG_ALIGN_ALIGN_Left << I2S_CONFIG_ALIGN_ALIGN_Pos);
  NRF_I2S->CONFIG.FORMAT    = (I2S_CONFIG_FORMAT_FORMAT_I2S << I2S_CONFIG_FORMAT_FORMAT_Pos);
  NRF_I2S->CONFIG.CHANNELS  = (I2S_CONFIG_CHANNELS_CHANNELS_Stereo << I2S_CONFIG_CHANNELS_CHANNELS_Pos);

  NRF_I2S->ENABLE = 1;
}

void playToneI2S(int frequency, int durationMs) {
  const int sample_rate = 8000;
  int total_samples = (sample_rate * durationMs) / 1000;
  int sample_idx = 0;
  int period = sample_rate / frequency;
  if (period < 2) period = 2;

  const int16_t AMPLITUDE = g_testAmplitude;

  NRF_I2S->RXTXD.MAXCNT = AUDIO_BUF_SIZE;

  for (int i = 0; i < AUDIO_BUF_SIZE; i++) {
    int16_t sample = ((sample_idx % period) < (period / 2)) ? AMPLITUDE : -AMPLITUDE;
    i2s_buf_0[i] = ((uint32_t)(uint16_t)sample << 16) | (uint16_t)sample;
    sample_idx++;
  }

  NRF_I2S->TXD.PTR = (uint32_t)i2s_buf_0;
  NRF_I2S->EVENTS_TXPTRUPD = 0;
  NRF_I2S->TASKS_START = 1;

  uint32_t* next_buf = i2s_buf_1;

  while (sample_idx < total_samples) {
    for (int i = 0; i < AUDIO_BUF_SIZE; i++) {
      int16_t sample = ((sample_idx % period) < (period / 2)) ? AMPLITUDE : -AMPLITUDE;
      next_buf[i] = ((uint32_t)(uint16_t)sample << 16) | (uint16_t)sample;
      sample_idx++;
    }

    // Service BLE while the DMA drains this buffer (~128ms at 8kHz). Without
    // this, a gesture-triggered clip starves the BLE stack long enough for the
    // central to hit supervision timeout and drop the link mid-session.
    while (NRF_I2S->EVENTS_TXPTRUPD == 0) { BLE.poll(); }
    NRF_I2S->EVENTS_TXPTRUPD = 0;

    NRF_I2S->TXD.PTR = (uint32_t)next_buf;
    next_buf = (next_buf == i2s_buf_0) ? i2s_buf_1 : i2s_buf_0;
  }
  NRF_I2S->TASKS_STOP = 1;
}

void playPCMI2S(const int16_t* samples, uint32_t total_samples) {
  uint32_t sample_idx = 0;

  NRF_I2S->RXTXD.MAXCNT = AUDIO_BUF_SIZE;

  for (int i = 0; i < AUDIO_BUF_SIZE; i++) {
    int16_t sample = (sample_idx < total_samples) ? samples[sample_idx] : 0;
    i2s_buf_0[i] = ((uint32_t)(uint16_t)sample << 16) | (uint16_t)sample;
    sample_idx++;
  }

  NRF_I2S->TXD.PTR = (uint32_t)i2s_buf_0;
  NRF_I2S->EVENTS_TXPTRUPD = 0;
  NRF_I2S->TASKS_START = 1;

  uint32_t* next_buf = i2s_buf_1;

  while (sample_idx < total_samples) {
    for (int i = 0; i < AUDIO_BUF_SIZE; i++) {
      int16_t sample = (sample_idx < total_samples) ? samples[sample_idx] : 0;
      next_buf[i] = ((uint32_t)(uint16_t)sample << 16) | (uint16_t)sample;
      sample_idx++;
    }

    // Service BLE while the DMA drains this buffer (~128ms at 8kHz). Without
    // this, a gesture-triggered clip starves the BLE stack long enough for the
    // central to hit supervision timeout and drop the link mid-session.
    while (NRF_I2S->EVENTS_TXPTRUPD == 0) { BLE.poll(); }
    NRF_I2S->EVENTS_TXPTRUPD = 0;

    NRF_I2S->TXD.PTR = (uint32_t)next_buf;
    next_buf = (next_buf == i2s_buf_0) ? i2s_buf_1 : i2s_buf_0;
  }
  NRF_I2S->TASKS_STOP = 1;
}
