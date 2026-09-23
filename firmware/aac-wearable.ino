#include <LSM6DS3.h>
#include <Wire.h>
#include <ArduinoBLE.h>
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
BLECharacteristic audioChar("a5e7c000-9c3f-4a4e-8e1e-1a2b3c4d5e03", BLEWriteWithoutResponse, 244);

// Audio upload state
bool     g_uploading = false;
int      g_uploadSlot = -1;
uint32_t g_uploadExpected = 0;
uint32_t g_uploadReceived = 0;
uint32_t g_uploadWriteAddr = 0;
uint8_t  g_uploadBuf[512];
uint32_t g_uploadBufLen = 0;
unsigned long g_uploadLastChunk = 0;
const unsigned long UPLOAD_TIMEOUT_MS = 10000;

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
    calService.addCharacteristic(audioChar);
    BLE.addService(calService);
    statusChar.writeValue("READY");
    BLE.advertise();
  }

  Serial.println("\n==================================================");
  Serial.println("  GESTURE AAC WRISTBAND ACTIVE");
  Serial.println("  * 7 messages: PAIN HUNGER TOILET SLEEP");
  Serial.println("                SEIZURE YES NO");
  Serial.println("  * Pain = two taps | Seizure = shake 3s");
  Serial.println("  * Others = recorded motion templates");
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
    g_uploadLastChunk = millis();

    if (g_erasePagesLeft <= 0) {
      g_erasing = false;
      logStatus("[UP] erased, ready to receive");
    }
    return;
  }

  if (audioChar.written()) {
    receiveAudioChunk(audioChar.value(), audioChar.valueLength());
  }

  // Suspend gesture processing while a clip is uploading. A dropped BLE
  // connection mid-transfer would otherwise wedge the device here.
  if (g_uploading) {
    if (millis() - g_uploadLastChunk > UPLOAD_TIMEOUT_MS) {
      g_uploading = false;
      g_uploadSlot = -1;
      g_uploadBufLen = 0;
      logStatus("[UP] ABORTED - timed out waiting for data");
    }
    delay(2);
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

  int slot; long bytes; float f;

  if (input == "LIST") {
    listMessages();
  } else if (sscanf(input.c_str(), "RECG,%d", &slot) == 1) {
    recordGesture(slot);
  } else if (sscanf(input.c_str(), "AUDIO,%d,%ld", &slot, &bytes) == 2) {
    beginAudioUpload(slot, (uint32_t)bytes);
  } else if (input == "AUDIOEND") {
    finishAudioUpload();
  } else if (sscanf(input.c_str(), "PLAY,%d", &slot) == 1) {
    if (slot >= 0 && slot < MSG_COUNT) playMessage(slot);
  } else if (sscanf(input.c_str(), "DEL,%d", &slot) == 1) {
    deleteMessage(slot);
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

void beginAudioUpload(int slot, uint32_t byteCount) {
  if (!g_flashReady) { logStatus("[ERROR] flash unavailable"); return; }
  if (slot < 0 || slot >= MSG_COUNT) { logStatus("[ERROR] bad slot"); return; }
  if (byteCount == 0 || byteCount > (uint32_t)AUDIO_MAX_SAMPLES * 2) {
    logStatus("[ERROR] clip too long (max 2.5s @ 8kHz)");
    return;
  }

  g_uploading = true;
  g_uploadSlot = slot;
  g_uploadExpected = byteCount;
  g_uploadReceived = 0;
  g_uploadBufLen = 0;
  g_uploadWriteAddr = slotAddr(slot) + AUDIO_HEADER_BYTES;
  g_uploadLastChunk = millis();

  // Erase spread across loop() iterations (see g_erasing declaration) -
  // this call only arms it, the actual erasing happens in loop().
  g_erasing = true;
  g_eraseAddr = slotAddr(slot);
  g_erasePagesLeft = AUDIO_SLOT_SIZE / g_flash.get_sector_size(g_eraseAddr);

  logStatus(String("[UP] ") + MSG_NAMES[slot] + " erasing " + String(g_erasePagesLeft) + " pages...");
}

void receiveAudioChunk(const uint8_t* data, int len) {
  if (!g_uploading || len <= 0) return;
  g_uploadLastChunk = millis();

  for (int i = 0; i < len && g_uploadReceived < g_uploadExpected; i++) {
    g_uploadBuf[g_uploadBufLen++] = data[i];
    g_uploadReceived++;

    if (g_uploadBufLen == sizeof(g_uploadBuf)) {
      g_flash.program(g_uploadBuf, g_uploadWriteAddr, g_uploadBufLen);
      g_uploadWriteAddr += g_uploadBufLen;
      g_uploadBufLen = 0;
    }
  }
}

void finishAudioUpload() {
  if (!g_uploading) return;

  if (g_uploadBufLen > 0) {
    // Flash programming needs a 4-byte aligned length
    while (g_uploadBufLen % 4 != 0) g_uploadBuf[g_uploadBufLen++] = 0;
    g_flash.program(g_uploadBuf, g_uploadWriteAddr, g_uploadBufLen);
    g_uploadBufLen = 0;
  }

  uint32_t hdr[2] = { AUDIO_MAGIC, g_uploadReceived / 2 };
  g_flash.program(hdr, slotAddr(g_uploadSlot), sizeof(hdr));

  logStatus(String("[UP] SAVED ") + MSG_NAMES[g_uploadSlot] + " " +
            String(g_uploadReceived) + "/" + String(g_uploadExpected) + " bytes");

  g_uploading = false;
  g_uploadSlot = -1;
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

    while (NRF_I2S->EVENTS_TXPTRUPD == 0) {}
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

    while (NRF_I2S->EVENTS_TXPTRUPD == 0) {}
    NRF_I2S->EVENTS_TXPTRUPD = 0;

    NRF_I2S->TXD.PTR = (uint32_t)next_buf;
    next_buf = (next_buf == i2s_buf_0) ? i2s_buf_1 : i2s_buf_0;
  }
  NRF_I2S->TASKS_STOP = 1;
}
