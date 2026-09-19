#include <LSM6DS3.h>
#include <Wire.h>
#include <ArduinoBLE.h>
#include "audio_chest.h"
#include "audio_head.h"
#include "audio_stomach.h"

// Initialize internal IMU on Wire1 (0x6A)
LSM6DS3 myIMU(I2C_MODE, 0x6A);

#define PIN_AMP_SD   D3
#define PIN_LED_BLUE LED_BUILTIN

enum BodyPose { POSE_REST = 0, POSE_HEAD = 1, POSE_CHEST = 2, POSE_STOMACH = 3 };
int16_t g_testAmplitude = 32767; // adjustable via "V,<0-32767>" for audio debugging

BodyPose currentPose = POSE_REST;
BodyPose lastConfirmedPose = POSE_REST;

unsigned long poseStartTime = 0;
const unsigned long DWELL_TIME_MS = 350;
bool announcementMade = false;

// =========================================================
// EMERGENCY SHAKE DETECTION VARIABLES (Seizure Alert)
// =========================================================
unsigned long shakeStartTime = 0;
unsigned long lastShakeTime = 0;
const float SHAKE_THRESHOLD_DPS = 300.0;
const unsigned long REQUIRED_SHAKE_DURATION_MS = 3000;

// =========================================================
// PAIN DETECTION (DOUBLE BANG/TAP) VARIABLES
// =========================================================
bool painModeArmed = false;
int tapCount = 0;
unsigned long lastTapTime = 0;
unsigned long painArmedTime = 0;
const float TAP_THRESHOLD_G = 2.5; // A hard bang exceeds 2.5g
const unsigned long PAIN_ARM_DURATION = 5000; // You have 5 seconds to point after double tap

// Audio Buffer
#define AUDIO_BUF_SIZE 1024
uint32_t i2s_buf_0[AUDIO_BUF_SIZE];
uint32_t i2s_buf_1[AUDIO_BUF_SIZE];

// =========================================================
// DYNAMIC CENTROID CLUSTERING
// =========================================================
// Each pose is stored as a normalized 3-axis gravity vector (full IMU
// orientation) rather than derived pitch/roll angles, which discard the
// third axis and wrap discontinuously at +/-180 degrees.
struct PoseCentroid {
  String name;
  float ax;
  float ay;
  float az;
};

PoseCentroid centroids[4] = {
  {"REST",     0.33,  -0.07,  0.94},
  {"HEAD",    -0.28,   0.09,  0.96},
  {"CHEST",    0.58,   0.79,  0.17},
  {"STOMACH",  0.23,   0.88,  0.41}
};

// Distance between unit vectors: d = 2*sin(angle/2).
// 0.45 corresponds to roughly 26 degrees of tolerance.
const float DETECTION_RADIUS = 0.45;

void normalizeVector(float &x, float &y, float &z) {
  float mag = sqrt(x * x + y * y + z * z);
  if (mag > 0.0001) {
    x /= mag;
    y /= mag;
    z /= mag;
  }
}

// =========================================================
// BLE CALIBRATION SERVICE
// =========================================================
BLEService calService("a5e7c000-9c3f-4a4e-8e1e-1a2b3c4d5e00");
BLEStringCharacteristic commandChar("a5e7c000-9c3f-4a4e-8e1e-1a2b3c4d5e01", BLEWrite, 32);
BLEStringCharacteristic statusChar("a5e7c000-9c3f-4a4e-8e1e-1a2b3c4d5e02", BLERead | BLENotify, 200);

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

  Serial.println("\n==================================================");
  Serial.println("  DYNAMIC AAC ANNOUNCER ACTIVE");
  Serial.println("  * Pose Engine Running");
  Serial.println("  * Seizure Shake Detection Active (3+ seconds)");
  Serial.println("  * Pain Detection Active (Double Bang Table -> Point)");
  Serial.println("  * BLE Calibration Service Advertising");
  Serial.println("==================================================");
}

void handleCommand(String input) {
  input.trim();
  if (input.length() == 0) return;

  int id; float vx, vy, vz;
  if (sscanf(input.c_str(), "%d,%f,%f,%f", &id, &vx, &vy, &vz) == 4) {
    if (id >= 0 && id <= 3) {
      normalizeVector(vx, vy, vz);
      centroids[id].ax = vx;
      centroids[id].ay = vy;
      centroids[id].az = vz;
      logStatus("[SUCCESS] Updated " + centroids[id].name + " -> X:" + String(vx, 3) +
                " Y:" + String(vy, 3) + " Z:" + String(vz, 3));
    }
  } else if (input == "H") {
    logStatus("[TEST] Playing HEAD tone");
    speakPose(POSE_HEAD);
  } else if (input == "C") {
    logStatus("[TEST] Playing CHEST tone");
    speakPose(POSE_CHEST);
  } else if (input == "S") {
    logStatus("[TEST] Playing STOMACH tone");
    speakPose(POSE_STOMACH);
  } else if (input == "P1") {
    logStatus("[TEST] Playing PAIN alarm (HEAD)");
    triggerPainAlarm(POSE_HEAD);
  } else if (input == "P2") {
    logStatus("[TEST] Playing PAIN alarm (CHEST)");
    triggerPainAlarm(POSE_CHEST);
  } else if (input == "P3") {
    logStatus("[TEST] Playing PAIN alarm (STOMACH)");
    triggerPainAlarm(POSE_STOMACH);
  } else if (input == "E") {
    logStatus("[TEST] Playing EMERGENCY/SEIZURE alarm");
    triggerEmergencyAlarm();
  } else if (input == "CAL") {
    calibrateCurrentPosition();
  } else if (input == "CAL0") {
    calibratePose(POSE_REST);
  } else if (input == "CAL1") {
    calibratePose(POSE_HEAD);
  } else if (input == "CAL2") {
    calibratePose(POSE_CHEST);
  } else if (input == "CAL3") {
    calibratePose(POSE_STOMACH);
  } else if (input == "SHOW") {
    showCentroids();
  } else if (input == "T") {
    logStatus("[TEST] Playing 440Hz test tone at amplitude " + String(g_testAmplitude));
    digitalWrite(PIN_AMP_SD, HIGH);
    digitalWrite(PIN_LED_BLUE, LOW);
    delay(50);
    playToneI2S(440, 1000);
    digitalWrite(PIN_AMP_SD, LOW);
    digitalWrite(PIN_LED_BLUE, HIGH);
  } else {
    int v;
    if (sscanf(input.c_str(), "V,%d", &v) == 1) {
      g_testAmplitude = constrain(v, 0, 32767);
      logStatus("[SUCCESS] Test amplitude set to " + String(g_testAmplitude));
    }
  }
}

void loop() {
  BLE.poll();

  // 1. Check for commands via USB Serial
  if (Serial.available() > 0) {
    handleCommand(Serial.readStringUntil('\n'));
  }

  // 1b. Check for commands via BLE
  if (commandChar.written()) {
    handleCommand(commandChar.value());
  }

  // 2. Read IMU Data
  float ax = myIMU.readFloatAccelX();
  float ay = myIMU.readFloatAccelY();
  float az = myIMU.readFloatAccelZ();
  float gx = myIMU.readFloatGyroX();
  float gy = myIMU.readFloatGyroY();
  float gz = myIMU.readFloatGyroZ();

  // 3. PAIN DETECTION (DOUBLE BANG/TAP) LOGIC
  float accel_magnitude = sqrt(ax*ax + ay*ay + az*az);

  if (accel_magnitude > TAP_THRESHOLD_G) {
    if (millis() - lastTapTime > 250) { // 250ms debounce to prevent false multi-taps
      tapCount++;
      lastTapTime = millis();
      Serial.print("[*] Bang detected! Count: "); Serial.println(tapCount);

      if (tapCount == 2) {
        painModeArmed = true;
        painArmedTime = millis();
        tapCount = 0;
        Serial.println("\n[!] PAIN MODE ARMED. Point to body part now...");
      }
    }
  }

  // Timeout: if they bang once but wait too long (>1.5s) for the second bang
  if (tapCount == 1 && (millis() - lastTapTime > 1500)) {
    tapCount = 0;
  }

  // Timeout: if they double bang but wait too long (>5s) to point to a body part
  if (painModeArmed && (millis() - painArmedTime > PAIN_ARM_DURATION)) {
    painModeArmed = false;
    Serial.println("[*] Pain mode timed out. Returned to normal.");
  }

  // 4. EMERGENCY SHAKE DETECTION LOGIC
  float gyro_magnitude = abs(gx) + abs(gy) + abs(gz);
  if (gyro_magnitude > SHAKE_THRESHOLD_DPS) {
    if (shakeStartTime == 0) {
      shakeStartTime = millis();
      Serial.println("\n[!] WARNING: SHAKING DETECTED...");
    }
    lastShakeTime = millis();
  }

  if (shakeStartTime > 0 && (millis() - lastShakeTime > 500)) {
    shakeStartTime = 0;
  }

  if (shakeStartTime > 0 && (millis() - shakeStartTime > REQUIRED_SHAKE_DURATION_MS)) {
    triggerEmergencyAlarm();
    shakeStartTime = 0;
    poseStartTime = millis();
    return;
  }

  // 5 & 6. Classify using the full 3-axis gravity vector
  BodyPose detectedPose = classifyPoseNearestNeighbor(ax, ay, az);

  // 7. Dwell Filter & Trigger
  if (detectedPose == currentPose) {
    if ((millis() - poseStartTime >= DWELL_TIME_MS) && !announcementMade && shakeStartTime == 0) {
      if (detectedPose != POSE_REST) {

        // CHECK IF PAIN MODE WAS ARMED BY A DOUBLE BANG
        if (painModeArmed) {
          triggerPainAlarm(detectedPose);
          painModeArmed = false; // Disarm after alarming
        } else {
          speakPose(detectedPose); // Normal behavior
        }

        announcementMade = true;
        lastConfirmedPose = detectedPose;
      }
    }
  } else {
    currentPose = detectedPose;
    poseStartTime = millis();
    if (detectedPose == POSE_REST) announcementMade = false;
  }

  delay(20);
}

// ---------------------------------------------------------
// Pain Alarm Trigger (5 Second Continuous Beep)
// ---------------------------------------------------------
void triggerPainAlarm(BodyPose pose) {
  Serial.println("\n************************************************");
  Serial.print("!!! PAIN DETECTED IN: ");
  if (pose == POSE_HEAD) Serial.print("HEAD");
  if (pose == POSE_CHEST) Serial.print("CHEST");
  if (pose == POSE_STOMACH) Serial.print("STOMACH");
  Serial.println(" !!!");
  Serial.println("************************************************\n");

  digitalWrite(PIN_AMP_SD, HIGH);
  digitalWrite(PIN_LED_BLUE, LOW);
  delay(50);

  // Play a loud, continuous 5-second long beep
  playToneI2S(1200, 5000);

  digitalWrite(PIN_AMP_SD, LOW);
  digitalWrite(PIN_LED_BLUE, HIGH);
}

// ---------------------------------------------------------
// Seizure Alarm Trigger
// ---------------------------------------------------------
void triggerEmergencyAlarm() {
  Serial.println("\n************************************************");
  Serial.println("!!! EMERGENCY DISTRESS / SEIZURE TRIGGERED !!!");
  Serial.println("************************************************\n");

  digitalWrite(PIN_AMP_SD, HIGH);
  digitalWrite(PIN_LED_BLUE, LOW);
  delay(50);

  for (int i = 0; i < 6; i++) {
    playToneI2S(1400, 400);
    playToneI2S(900, 400);
  }

  digitalWrite(PIN_AMP_SD, LOW);
  digitalWrite(PIN_LED_BLUE, HIGH);
}

// ---------------------------------------------------------
// Calibration Helpers
// ---------------------------------------------------------
void calibrateCurrentPosition() {
  logStatus("[CAL] Hold still... sampling for 3 seconds");

  const int SAMPLES = 150;
  float sum_x = 0, sum_y = 0, sum_z = 0;

  for (int i = 0; i < SAMPLES; i++) {
    sum_x += myIMU.readFloatAccelX();
    sum_y += myIMU.readFloatAccelY();
    sum_z += myIMU.readFloatAccelZ();
    delay(20);
  }

  float ax = sum_x / SAMPLES;
  float ay = sum_y / SAMPLES;
  float az = sum_z / SAMPLES;
  normalizeVector(ax, ay, az);

  BodyPose match = classifyPoseNearestNeighbor(ax, ay, az);
  logStatus("[CAL] X:" + String(ax, 3) + " Y:" + String(ay, 3) + " Z:" + String(az, 3) +
            " -> classifies as: " + centroids[(int)match].name);
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

void calibratePose(BodyPose pose) {
  int id = (int)pose;

  logStatus("[CAL] Get into position for " + centroids[id].name + " - starting in 2 seconds...");
  delay(2000);

  calibrateBeep(1);   // one beep = hold still, sampling now
  logStatus("[CAL] Sampling for 5 seconds - HOLD STILL");
  digitalWrite(PIN_LED_BLUE, LOW);

  const int SAMPLES = 250;   // 250 x 20ms = 5 seconds
  float sum_x = 0, sum_y = 0, sum_z = 0;

  for (int i = 0; i < SAMPLES; i++) {
    sum_x += myIMU.readFloatAccelX();
    sum_y += myIMU.readFloatAccelY();
    sum_z += myIMU.readFloatAccelZ();
    delay(20);
  }

  float ax = sum_x / SAMPLES;
  float ay = sum_y / SAMPLES;
  float az = sum_z / SAMPLES;
  normalizeVector(ax, ay, az);

  centroids[id].ax = ax;
  centroids[id].ay = ay;
  centroids[id].az = az;

  digitalWrite(PIN_LED_BLUE, HIGH);
  calibrateBeep(2);   // two beeps = done

  logStatus("[CAL] SAVED " + centroids[id].name + " -> X:" + String(ax, 3) +
            " Y:" + String(ay, 3) + " Z:" + String(az, 3));

  // Warn if this position is too close to an already-calibrated one to
  // be told apart reliably.
  for (int i = 0; i < 4; i++) {
    if (i == id) continue;
    float dx = ax - centroids[i].ax;
    float dy = ay - centroids[i].ay;
    float dz = az - centroids[i].az;
    float d = sqrt(dx * dx + dy * dy + dz * dz);
    if (d < DETECTION_RADIUS) {
      logStatus("[CAL] WARNING: too close to " + centroids[i].name + " (distance " +
                String(d, 3) + ", need > " + String(DETECTION_RADIUS, 2) +
                "). Tilt it differently at this spot.");
    }
  }
}

void showCentroids() {
  Serial.println("\n[INFO] Current pose targets:");
  for (int i = 0; i < 4; i++) {
    String line = String(i) + " " + centroids[i].name + " -> X:" + String(centroids[i].ax, 3) +
                  " Y:" + String(centroids[i].ay, 3) + " Z:" + String(centroids[i].az, 3);
    logStatus(line);
  }
  logStatus("Detection radius: " + String(DETECTION_RADIUS, 2));
}

// ---------------------------------------------------------
// Nearest Neighbor Classification Algorithm
// ---------------------------------------------------------
BodyPose classifyPoseNearestNeighbor(float ax, float ay, float az) {
  normalizeVector(ax, ay, az);

  float min_distance = 100000.0;
  int closest_id = 0;

  for (int i = 0; i < 4; i++) {
    float dx = ax - centroids[i].ax;
    float dy = ay - centroids[i].ay;
    float dz = az - centroids[i].az;
    float distance = sqrt((dx * dx) + (dy * dy) + (dz * dz));

    if (distance < min_distance) {
      min_distance = distance;
      closest_id = i;
    }
  }

  if (min_distance < DETECTION_RADIUS) {
    return (BodyPose)closest_id;
  }
  return POSE_REST;
}

// ---------------------------------------------------------
// Normal Audio Trigger
// ---------------------------------------------------------
void speakPose(BodyPose pose) {
  digitalWrite(PIN_AMP_SD, HIGH);
  digitalWrite(PIN_LED_BLUE, LOW);
  delay(50);

  switch (pose) {
    case POSE_HEAD:
      Serial.println("\n>>> [SPEECH OUTPUT]: \"HEAD\"");
      playPCMI2S(HEAD_PCM, HEAD_PCM_LEN);
      break;
    case POSE_CHEST:
      Serial.println("\n>>> [SPEECH OUTPUT]: \"CHEST\"");
      playPCMI2S(CHEST_PCM, CHEST_PCM_LEN);
      break;
    case POSE_STOMACH:
      Serial.println("\n>>> [SPEECH OUTPUT]: \"STOMACH\"");
      playPCMI2S(STOMACH_PCM, STOMACH_PCM_LEN);
      break;
    default:
      break;
  }

  digitalWrite(PIN_AMP_SD, LOW);
  digitalWrite(PIN_LED_BLUE, HIGH);
}

// ---------------------------------------------------------
// DIRECT HARDWARE I2S DMA
// ---------------------------------------------------------
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
  NRF_I2S->CONFIG.RATIO     = (I2S_CONFIG_RATIO_RATIO_32X << I2S_CONFIG_RATIO_RATIO_Pos);
  NRF_I2S->CONFIG.SWIDTH    = (I2S_CONFIG_SWIDTH_SWIDTH_16Bit << I2S_CONFIG_SWIDTH_SWIDTH_Pos);
  NRF_I2S->CONFIG.ALIGN     = (I2S_CONFIG_ALIGN_ALIGN_Left << I2S_CONFIG_ALIGN_ALIGN_Pos);
  NRF_I2S->CONFIG.FORMAT    = (I2S_CONFIG_FORMAT_FORMAT_I2S << I2S_CONFIG_FORMAT_FORMAT_Pos);
  NRF_I2S->CONFIG.CHANNELS  = (I2S_CONFIG_CHANNELS_CHANNELS_Stereo << I2S_CONFIG_CHANNELS_CHANNELS_Pos);

  NRF_I2S->ENABLE = 1;
}

void playToneI2S(int frequency, int durationMs) {
  const int sample_rate = 16000;
  int total_samples = (sample_rate * durationMs) / 1000;
  int sample_idx = 0;
  int period = sample_rate / frequency;

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

// ---------------------------------------------------------
// PCM Sample Playback over I2S (spoken words)
// ---------------------------------------------------------
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
