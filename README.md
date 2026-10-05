# AAC Wearable

A gesture-based AAC (augmentative and alternative communication) wristband built on a Seeed XIAO nRF52840 Sense. The wearer performs a wrist movement and the device speaks the matching message in a voice recorded by their caregiver.

Gestures and voice clips are both **recorded by the caregiver** through a companion Android app over Bluetooth LE, and stored on the device's internal flash so they survive power cycles (and even firmware updates).

## Communication vocabulary

| Message | Gesture |
|---|---|
| Rest | Learned from 10 recordings |
| Seizure / Fits | Vigorous shaking for 3s *(fixed)* |
| Hunger | Learned from 10 recordings |
| Toilet | Learned from 10 recordings |
| Sleep | Learned from 10 recordings |
| Yes | Learned from 10 recordings |
| No | Learned from 10 recordings |

Seizure uses a dedicated always-on shake detector — it's safety-critical and shouldn't depend on a learned model. The other six are learned on the watch from 10 recordings each, so the caregiver picks movements that are meaningfully distinct from each other.

## Structure

- `firmware/` — Arduino sketch for the XIAO nRF52840 Sense
- `app/` — Android app (Kotlin) for recording gestures and voice clips over BLE

## How it works

**Gesture learning.** Each recording is 64 samples of 6-axis IMU data at 50Hz (1.28s): accelerometer x/y/z and gyro x/y/z, stored as int8. The accelerometer is kept raw rather than mean-subtracted, so the arm's orientation (where the hand is) counts toward a match, not just the motion shape. `RECG,<slot>` records 10 repetitions; each one waits for motion to start, the same trigger live detection uses, so training and live windows line up. Live motion is classified by 3-nearest-neighbour voting over every stored recording using banded DTW.

**Accuracy.** After training, the watch runs a leave-one-out check: each recording is classified using only the others, so the score reflects how well an unseen repeat would be recognised (testing a recording against itself would always score 100%). It reports per-gesture and overall accuracy, and sets the live match limit from how far apart repeats of the same gesture landed. Re-run any time with `ACC`. The seizure detector keeps running during the check.

**Voice recording.** Clips are recorded **on the device itself** through the XIAO Sense's onboard PDM microphone, not transferred from the phone. The mic samples at 16kHz and sample pairs are averaged down to the 8kHz the playback path uses. Clips are up to 2.5s, written to reserved internal flash via mbed `FlashIAP`. nRF52840 flash is memory-mapped, so playback streams straight from a flash pointer with no RAM buffering.

Bluetooth only carries a short `RECV,<slot>` command — there is no bulk audio transfer. An earlier design pushed the 40KB clip over BLE and the link dropped mid-upload: with no SoftDevice on this core, flash writes block interrupts, and write-without-response has no flow control, so the device's BLE buffers were exhausted. Recording locally removes that entire failure mode rather than tuning around it.

### Flash map

App region is `0x27000`–`0xED000`.

| Region | Address | Size | Use |
|---|---|---|---|
| Sketch | `0x27000`–`0xA0000` | 495KB limit | firmware (~346KB) |
| Voice clips | `0xA0000`–`0xE6000` | 7 × 40KB | one slot per message |
| Gesture recordings | `0xE6000`–`0xED000` | 7 × 4KB | 10 recordings per message, one page each |

> The compiled sketch must stay below `0xA0000` or it will collide with stored clips. The build output reports the size — check it after adding code.

Note the QSPI flash chip on the XIAO is **not** used: this core doesn't enable the QSPI peripheral (`DEVICE_QSPI` undefined, no HAL), and Adafruit_SPIFlash depends on `g_ADigitalPinMap`, which is an Adafruit-core symbol absent from the mbed core.

## Firmware

**Board:** Seeed XIAO nRF52840 Sense · **Core:** `Seeeduino:mbed` · **Libraries:** `Seeed Arduino LSM6DS3`, `ArduinoBLE`

```bash
arduino-cli compile --fqbn Seeeduino:mbed:xiaonRF52840Sense firmware
arduino-cli upload -p /dev/ttyACM0 --fqbn Seeeduino:mbed:xiaonRF52840Sense firmware
```

### Serial / BLE commands (115200 baud)

Both interfaces share one dispatcher, so every command works over USB serial and Bluetooth alike.

| Command | Effect |
|---|---|
| `LIST` | Show all 7 messages and whether each has a voice clip / gesture |
| `RECG,<slot>` | Learn a gesture from 10 recordings (beep before each, two beeps when saved), then report accuracy |
| `ACC` | Re-run the accuracy check |
| `PLAY,<slot>` | Play a message's voice clip |
| `DEL,<slot>` | Erase a message's clip and gesture |
| `GT,<value>` | Override the gesture match limit (normally set automatically by `ACC`) |
| `RECV,<slot>` | Record a voice clip from the onboard mic (one beep = speak, two beeps = saved) |
| `V,<0-32767>` | Test tone amplitude |
| `T` | Play a test tone |

Slots: `0` Rest · `1` Hunger · `2` Toilet · `3` Sleep · `4` Seizure · `5` Yes · `6` No

### BLE service

- Service: `a5e7c000-9c3f-4a4e-8e1e-1a2b3c4d5e00`
- Command (write): `…5e01`
- Status (notify): `…5e02`

Clients should negotiate a larger MTU — the default 23-byte ATT MTU truncates status messages to ~20 bytes.

## Android app

Kotlin, minSdk 26, no Compose. One row per message with **Gesture / Voice / Play / Delete**. Each button sends a short command; the device does the recording and storage itself, so the app needs no microphone permission and transfers no audio.

```bash
cd app
./gradlew assembleDebug
```

Output: `app/build/outputs/apk/debug/app-debug.apk` (debug-signed, sideloadable).

## Not covered here

The source specification also calls for a status display, a dedicated physical emergency button, an activation lock, rechargeable battery with all-day runtime, and waterproof charging. Those are hardware requirements and aren't addressed by this firmware/app.

## Recent changes

- **2026-10-05** — Pain replaced by Rest; gestures learned on-device from 10 recordings each (k-NN over raw accel x/y/z + gyro) with leave-one-out accuracy reported per gesture.

- **2026-10-05** — Voice is now recorded on-device via the onboard PDM mic instead of being uploaded over BLE, which removes the mid-upload disconnects entirely.

- **2026-09-23** — Replaced body-part poses with the 7-message gesture vocabulary; added recordable gestures (DTW matching) and caregiver-recorded voice clips stored in internal flash and uploaded over BLE.
- **2026-09-19** — Created public GitHub repo, combined firmware and Android app into one repository.
