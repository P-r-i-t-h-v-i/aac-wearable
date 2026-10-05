#include <LSM6DS3.h>
#include <Wire.h>
#include <ArduinoBLE.h>
#include <PDM.h>
#include "FlashIAP.h"
#include "gesture_types.h"

// Initialize internal IMU on Wire1 (0x6A)
LSM6DS3 myIMU(I2C_MODE, 0x6A);

#define PIN_AMP_SD   D3
#define PIN_LED_BLUE LED_BUILTIN

// =========================================================
// COMMUNICATION MESSAGES
// Seizure keeps a dedicated always-on shake detector - it is safety-critical
// and must not depend on a trained model. The other six are learned from
// recordings the caregiver makes on the device (10 each).
// =========================================================
enum MsgId {
  MSG_REST = 0, MSG_HUNGER = 1, MSG_TOILET = 2, MSG_SLEEP = 3,
  MSG_SEIZURE = 4, MSG_YES = 5, MSG_NO = 6
};
#define MSG_COUNT 7

const char* MSG_NAMES[MSG_COUNT] = {
  "REST", "HUNGER", "TOILET", "SLEEP", "SEIZURE", "YES", "NO"
};

// Slots learned from recordings (Seizure uses its fixed detector)
bool isTemplateSlot(int id) {
  return id != MSG_SEIZURE;
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

mbed::FlashIAP g_flash;
bool g_flashReady = false;

// =========================================================
// LEARNED GESTURES
// Each recording is 64 samples @ 50Hz (1.28s) of 6 axes: accelerometer
// x/y/z and gyro x/y/z. The accelerometer is kept raw rather than
// mean-subtracted so the arm's orientation (which way gravity points, i.e.
// where the hand is) counts toward the match, not just the motion shape.
// Values are stored as int8: 32 units = 1g for accel, 32 units = 256dps
// for gyro, so both axes groups weigh comparably in the distance.
//
// Classification is k-nearest-neighbour (k=3) over all stored recordings
// using banded DTW, so the model learns the natural spread of how the
// person performs each gesture rather than matching one snapshot.
// =========================================================
#define GEST_BAND         8           // DTW warp window, +/-160ms
#define ACCEL_SCALE       32.0f       // units per g
#define GYRO_SCALE        (1.0f / 8)  // units per dps

// One 4KB flash page per message holds its 10 recordings (3848 bytes).
// 7 pages: 0xE6000..0xED000, the top of the app region.
#define TEMPLATE_BASE     0xE6000
#define TEMPLATE_PAGE     0x1000
#define TEMPLATE_MAGIC    0x6E5710

float g_gestureThreshold = 40.0f;     // recalibrated from training data
// Floor for the calibrated limit. Very consistent repeats can put repeats
// almost on top of each other (still recordings measured ~0), which would
// otherwise set a limit no live attempt could ever meet. In feature units:
// ~0.4g or ~3dps average deviation per axis.
const float GESTURE_THRESHOLD_MIN = 12.0f;
unsigned long g_accStartMs = 0;

// Live gesture capture state
bool  g_capturing = false;
int   g_captureIdx = 0;
int8_t g_captureBuf[WIN_BYTES];
unsigned long g_lastTriggerTime = 0;
const unsigned long GESTURE_COOLDOWN_MS = 2000;
const float GESTURE_TRIGGER_DPS = 120.0;  // motion energy needed to start capture

// Training state (10 reps, run one step per loop() so BLE stays serviced)
enum TrainPhase { TR_IDLE, TR_GAP, TR_ARMED, TR_CAPTURE };
TrainPhase g_trainPhase = TR_IDLE;
int  g_trainSlot = -1;
int  g_trainRep = 0;
int  g_trainIdx = 0;
unsigned long g_trainT = 0;
ClassTemplates g_trainPage;
const unsigned long TRAIN_FIRST_GAP_MS = 2000;
const unsigned long TRAIN_GAP_MS = 1500;
const unsigned long TRAIN_ARM_TIMEOUT_MS = 2500;

// Accuracy check (leave-one-out), also one sample per loop()
bool  g_accRunning = false;
int   g_accC = 0, g_accR = 0;
int   g_accCorrect[MSG_COUNT], g_accTotal[MSG_COUNT];
float g_accMaxSame = 0;

int16_t g_testAmplitude = 32767;

// =========================================================
// SEIZURE (SHAKE) DETECTOR
// =========================================================
unsigned long shakeStartTime = 0;
unsigned long lastShakeTime = 0;
const float SHAKE_THRESHOLD_DPS = 300.0;
const unsigned long REQUIRED_SHAKE_DURATION_MS = 3000;

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
float REC_TARGET_RMS = 0.35f;       // tune with "MR,<0.05-0.5>"

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
  Serial.println("  * 7 messages: REST HUNGER TOILET SLEEP");
  Serial.println("                SEIZURE YES NO");
  Serial.println("  * Seizure = shake 3s (fixed)");
  Serial.println("  * Others = learned from 10 recordings each");
  Serial.println("  * Voice recorded on-device via onboard mic");
  Serial.println("  * BLE service advertising");
  Serial.println("==================================================");

  // Calibrate the match threshold from whatever is already trained.
  beginAccuracy();
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
  float gyro_magnitude = abs(gx) + abs(gy) + abs(gz);

  // Training owns the IMU while it runs - no live detection, so practising
  // a gesture (or a vigorous one) can't fire a message or the seizure alarm.
  if (g_trainPhase != TR_IDLE) {
    serviceTraining(ax, ay, az, gx, gy, gz, gyro_magnitude);
    delay(20);
    return;
  }

  // --- SEIZURE: sustained vigorous shaking ---
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

  // Leave-one-out accuracy check, one recording classified per iteration.
  // Sits after the seizure check so that safety alarm keeps running while
  // the check works through every recording.
  if (g_accRunning) {
    serviceAccuracy();
    return;
  }

  // --- LEARNED GESTURES ---
  if (g_capturing) {
    encodeSample(&g_captureBuf[g_captureIdx * GEST_AXES], ax, ay, az, gx, gy, gz);
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
    beginTraining(slot);
  } else if (input == "ACC") {
    beginAccuracy();
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
      line += " gesture:" + String(trainedReps(i)) + "/" + String(REPS_PER_GESTURE);
    } else {
      line += " gesture:SHAKE";
    }
    logStatus(line);
  }
}

// =========================================================
// GESTURE FEATURES
// =========================================================
static inline int8_t q8(float v) {
  int x = (int)lroundf(v);
  if (x > 127) x = 127;
  if (x < -127) x = -127;
  return (int8_t)x;
}

void encodeSample(int8_t* out, float ax, float ay, float az,
                  float gx, float gy, float gz) {
  out[0] = q8(ax * ACCEL_SCALE);
  out[1] = q8(ay * ACCEL_SCALE);
  out[2] = q8(az * ACCEL_SCALE);
  out[3] = q8(gx * GYRO_SCALE);
  out[4] = q8(gy * GYRO_SCALE);
  out[5] = q8(gz * GYRO_SCALE);
}

const ClassTemplates* classPage(int c) {
  return (const ClassTemplates*)(TEMPLATE_BASE + (uint32_t)c * TEMPLATE_PAGE);
}

int trainedReps(int c) {
  if (c < 0 || c >= MSG_COUNT || !isTemplateSlot(c)) return 0;
  const ClassTemplates* p = classPage(c);
  if (p->magic != TEMPLATE_MAGIC) return 0;
  return (p->count > REPS_PER_GESTURE) ? REPS_PER_GESTURE : (int)p->count;
}

// Banded DTW with L1 cost over the 6 axes. Returns per-step distance in
// the int8 feature units (32 = 1g / 256dps).
// The Arduino build uses -Os; this inner loop runs ~3500 times for a full
// accuracy check, so it is worth compiling for speed instead.
__attribute__((optimize("O3")))
float dtwDistance(const int8_t* a, const int8_t* b) {
  static int32_t rowA[GEST_LEN], rowB[GEST_LEN];
  const int32_t INF = 0x3FFFFFFF;
  int32_t* prev = rowA;
  int32_t* curr = rowB;

  for (int j = 0; j < GEST_LEN; j++) prev[j] = INF;
  for (int j = 0; j <= GEST_BAND && j < GEST_LEN; j++) {
    int32_t c = 0;
    for (int k = 0; k < GEST_AXES; k++) c += abs(a[k] - b[j * GEST_AXES + k]);
    prev[j] = (j == 0) ? c : prev[j - 1] + c;
  }

  for (int i = 1; i < GEST_LEN; i++) {
    for (int j = 0; j < GEST_LEN; j++) curr[j] = INF;
    int lo = i - GEST_BAND; if (lo < 0) lo = 0;
    int hi = i + GEST_BAND; if (hi > GEST_LEN - 1) hi = GEST_LEN - 1;

    for (int j = lo; j <= hi; j++) {
      int32_t c = 0;
      for (int k = 0; k < GEST_AXES; k++) {
        c += abs(a[i * GEST_AXES + k] - b[j * GEST_AXES + k]);
      }
      int32_t best = prev[j];
      if (j > 0) {
        if (prev[j - 1] < best) best = prev[j - 1];
        if (curr[j - 1] < best) best = curr[j - 1];
      }
      curr[j] = (best >= INF) ? INF : best + c;
    }
    int32_t* t = prev; prev = curr; curr = t;
  }
  return (float)prev[GEST_LEN - 1] / (float)GEST_LEN;
}

// k=3 nearest-neighbour vote over every stored recording, skipping
// (exC, exR) so leave-one-out can test a recording against the rest.
// nearest = distance to the closest recording of any class;
// nearestSame = distance to the closest recording of sameC (if sameC >= 0).
int knnClassify(const int8_t* win, int exC, int exR, int sameC,
                float* nearest, float* nearestSame) {
  float bd[3] = {1e9f, 1e9f, 1e9f};
  int   bc[3] = {-1, -1, -1};
  float same = 1e9f;

  // DTW revisits each value ~17 times. Stored recordings live in flash,
  // where data reads are slow, so copy both sides into RAM once first.
  static int8_t query[WIN_BYTES];
  static int8_t ref[WIN_BYTES];
  memcpy(query, win, WIN_BYTES);

  for (int c = 0; c < MSG_COUNT; c++) {
    int n = trainedReps(c);
    const ClassTemplates* p = classPage(c);
    for (int r = 0; r < n; r++) {
      if (c == exC && r == exR) continue;
      memcpy(ref, p->win[r], WIN_BYTES);
      float d = dtwDistance(query, ref);
      if (c == sameC && d < same) same = d;

      if (d < bd[2]) {
        int pos = 2;
        while (pos > 0 && d < bd[pos - 1]) {
          bd[pos] = bd[pos - 1]; bc[pos] = bc[pos - 1]; pos--;
        }
        bd[pos] = d; bc[pos] = c;
      }
    }
  }

  if (nearest) *nearest = bd[0];
  if (nearestSame) *nearestSame = same;
  if (bc[0] < 0) return -1;

  // Majority of the 3; with no majority the closest one wins.
  if (bc[1] >= 0 && bc[1] == bc[2] && bc[1] != bc[0]) return bc[1];
  return bc[0];
}

int matchGesture() {
  float nearest = 1e9f;
  int pred = knnClassify(g_captureBuf, -1, -1, -1, &nearest, nullptr);
  if (pred < 0) return -1;

  bool ok = nearest <= g_gestureThreshold;
  logStatus(String("[GESTURE] ") + MSG_NAMES[pred] + " dist=" + String(nearest, 1) +
            " limit=" + String(g_gestureThreshold, 1) + (ok ? " MATCH" : " (no match)"));
  return ok ? pred : -1;
}

// =========================================================
// TRAINING: 10 recordings per gesture
// Runs as a state machine in loop() (one IMU sample per iteration). A
// blocking 10-rep loop would hold off BLE for ~30s and drop the link.
// Each rep waits for motion to start before capturing, the same trigger the
// live detector uses, so training and live windows line up the same way.
// =========================================================
bool deviceBusy() {
  return g_recording || g_erasing || g_flushing ||
         g_trainPhase != TR_IDLE || g_accRunning;
}

void beginTraining(int slot) {
  if (slot < 0 || slot >= MSG_COUNT || !isTemplateSlot(slot)) {
    logStatus("[ERROR] That message uses a fixed gesture (shake)");
    return;
  }
  if (!g_flashReady) { logStatus("[ERROR] flash unavailable"); return; }
  if (deviceBusy()) { logStatus("[ERROR] busy, try again"); return; }

  g_trainSlot = slot;
  g_trainRep = 0;
  g_trainPhase = TR_GAP;
  g_trainT = millis();
  g_capturing = false;
  logStatus(String("[TRAIN] ") + MSG_NAMES[slot] + ": do the movement " +
            String(REPS_PER_GESTURE) + " times, starting after each beep");
}

void serviceTraining(float ax, float ay, float az,
                     float gx, float gy, float gz, float gyroMag) {
  unsigned long now = millis();

  switch (g_trainPhase) {
    case TR_GAP: {
      unsigned long gap = (g_trainRep == 0) ? TRAIN_FIRST_GAP_MS : TRAIN_GAP_MS;
      if (now - g_trainT >= gap) {
        calibrateBeep(1);
        g_trainPhase = TR_ARMED;
        g_trainT = millis();
        logStatus(String("[TRAIN] ") + MSG_NAMES[g_trainSlot] + " rep " +
                  String(g_trainRep + 1) + "/" + String(REPS_PER_GESTURE) + " - go");
      }
      break;
    }
    case TR_ARMED:
      // Start on motion like the live detector; a still gesture times out
      // and is captured anyway so it can still be learned.
      if (gyroMag > GESTURE_TRIGGER_DPS || now - g_trainT > TRAIN_ARM_TIMEOUT_MS) {
        g_trainPhase = TR_CAPTURE;
        g_trainIdx = 0;
        digitalWrite(PIN_LED_BLUE, LOW);
      }
      break;

    case TR_CAPTURE:
      encodeSample(&g_trainPage.win[g_trainRep][g_trainIdx * GEST_AXES],
                   ax, ay, az, gx, gy, gz);
      if (++g_trainIdx >= GEST_LEN) {
        digitalWrite(PIN_LED_BLUE, HIGH);
        g_trainRep++;
        if (g_trainRep >= REPS_PER_GESTURE) {
          finishTraining();
        } else {
          g_trainPhase = TR_GAP;
          g_trainT = millis();
        }
      }
      break;

    default:
      break;
  }
}

void finishTraining() {
  int slot = g_trainSlot;
  g_trainPhase = TR_IDLE;
  g_trainSlot = -1;

  g_trainPage.magic = TEMPLATE_MAGIC;
  g_trainPage.count = REPS_PER_GESTURE;

  // One page: ~85ms erase + ~40ms program, short enough not to drop BLE.
  uint32_t addr = TEMPLATE_BASE + (uint32_t)slot * TEMPLATE_PAGE;
  if (g_flash.erase(addr, TEMPLATE_PAGE) != 0 ||
      g_flash.program(&g_trainPage, addr, sizeof(ClassTemplates)) != 0) {
    logStatus("[ERROR] could not save gesture");
    return;
  }

  calibrateBeep(2);
  logStatus(String("[TRAIN] SAVED ") + MSG_NAMES[slot] + " (" +
            String(REPS_PER_GESTURE) + " recordings)");
  g_lastTriggerTime = millis();
  beginAccuracy();
}

// =========================================================
// ACCURACY (leave-one-out)
// Each recording is classified using only the other recordings, so the
// score reflects how well an unseen repeat would be recognised - testing a
// recording against itself would always score 100%. The same pass sets the
// live match threshold from how far apart repeats of one gesture land.
// =========================================================
int nextTrainedClass(int from) {
  for (int c = from; c < MSG_COUNT; c++) {
    if (trainedReps(c) >= 2) return c;
  }
  return -1;
}

void beginAccuracy() {
  if (!g_flashReady) return;
  int first = nextTrainedClass(0);
  if (first < 0) return;   // nothing trained yet

  for (int i = 0; i < MSG_COUNT; i++) { g_accCorrect[i] = 0; g_accTotal[i] = 0; }
  g_accMaxSame = 0;
  g_accC = first;
  g_accR = 0;
  g_accRunning = true;
  g_accStartMs = millis();
  logStatus("[ACC] checking accuracy...");
}

void serviceAccuracy() {
  const ClassTemplates* p = classPage(g_accC);
  float nearest, nearestSame;
  int pred = knnClassify(p->win[g_accR], g_accC, g_accR, g_accC, &nearest, &nearestSame);

  g_accTotal[g_accC]++;
  if (pred == g_accC) g_accCorrect[g_accC]++;
  if (nearestSame < 1e8f && nearestSame > g_accMaxSame) g_accMaxSame = nearestSame;

  if (++g_accR >= trainedReps(g_accC)) {
    g_accR = 0;
    g_accC = nextTrainedClass(g_accC + 1);
    if (g_accC < 0) finishAccuracy();
  }
}

void finishAccuracy() {
  g_accRunning = false;

  // Accept a live gesture if it is as close to its nearest stored recording
  // as the furthest-apart repeats were, plus margin.
  if (g_accMaxSame > 0) g_gestureThreshold = g_accMaxSame * 1.5f;
  if (g_gestureThreshold < GESTURE_THRESHOLD_MIN) g_gestureThreshold = GESTURE_THRESHOLD_MIN;

  int classes = 0, correct = 0, total = 0;
  for (int c = 0; c < MSG_COUNT; c++) {
    if (g_accTotal[c] == 0) continue;
    classes++;
    correct += g_accCorrect[c];
    total += g_accTotal[c];
  }

  if (classes < 2) {
    logStatus("[ACC] Train at least 2 gestures to measure accuracy");
  } else {
    for (int c = 0; c < MSG_COUNT; c++) {
      if (g_accTotal[c] == 0) continue;
      logStatus(String("[ACC] ") + MSG_NAMES[c] + " " + String(g_accCorrect[c]) + "/" +
                String(g_accTotal[c]) + " (" +
                String(100.0f * g_accCorrect[c] / g_accTotal[c], 0) + "%)");
    }
    logStatus(String("[ACC] OVERALL ") + String(correct) + "/" + String(total) +
              " = " + String(100.0f * correct / total, 1) + "%");
  }
  logStatus("[ACC] match limit set to " + String(g_gestureThreshold, 1) +
            " (check took " + String(millis() - g_accStartMs) + "ms)");
  g_lastTriggerTime = millis();
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
  if (deviceBusy()) {
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
  if (deviceBusy()) { logStatus("[ERROR] busy, try again"); return; }

  // Gesture page: one erase (~85ms). Voice slot: 10 pages, so it goes
  // through the incremental eraser rather than one long blocking call.
  if (isTemplateSlot(slot)) {
    g_flash.erase(TEMPLATE_BASE + (uint32_t)slot * TEMPLATE_PAGE, TEMPLATE_PAGE);
  }
  g_recPendingSlot = -1;
  g_erasing = true;
  g_eraseAddr = slotAddr(slot);
  g_erasePagesLeft = AUDIO_SLOT_SIZE / g_flash.get_sector_size(g_eraseAddr);

  logStatus(String("[OK] Cleared ") + MSG_NAMES[slot]);
  beginAccuracy();
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
