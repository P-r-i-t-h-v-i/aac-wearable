# AAC Wearable

A gesture-based AAC (augmentative and alternative communication) wristband built on a Seeed XIAO nRF52840 Sense. The wearer performs a wrist movement and the device speaks the matching message in a voice recorded by their caregiver.

Gestures and voice clips are both **recorded by the caregiver** through a companion Android app over Bluetooth LE, and stored on the device's internal flash so they survive power cycles (and even firmware updates).

## Communication vocabulary

| Message | Gesture |
|---|---|
| Pain | Two sharp taps of the wrist against a surface *(fixed)* |
| Seizure / Fits | Vigorous shaking for 3s *(fixed)* |
| Hunger | Recorded by caregiver |
| Toilet | Recorded by caregiver |
| Sleep | Recorded by caregiver |
| Yes | Recorded by caregiver |
| No | Recorded by caregiver |

Pain and Seizure use dedicated always-on detectors — they're safety-critical and shouldn't depend on template matching. The other five are matched against motion templates the caregiver records, which also means the caregiver picks movements that are meaningfully distinct from each other.

## Structure

- `firmware/` — Arduino sketch for the XIAO nRF52840 Sense
- `app/` — Android app (Kotlin) for recording gestures and voice clips over BLE

## How it works

**Gesture matching.** A recorded gesture is 64 samples of 6-axis IMU data at 50Hz (1.28s), z-normalised per axis so matching depends on the shape of the movement rather than how hard it was performed. Live motion above a trigger threshold is captured the same way and scored against every stored template using banded DTW (Dynamic Time Warping); the closest match under a distance threshold fires. Tune the threshold with `GT,<value>` and watch reported distances in the status log.

**Voice storage.** Clips are 8kHz mono PCM16, up to 2.5s each, written to reserved internal flash via mbed `FlashIAP`. nRF52840 flash is memory-mapped, so playback streams straight from a flash pointer with no RAM buffering.

### Flash map

App region is `0x27000`–`0xED000`.

| Region | Address | Size | Use |
|---|---|---|---|
| Sketch | `0x27000`–`0xA0000` | 495KB limit | firmware (~346KB) |
| Voice clips | `0xA0000`–`0xE6000` | 7 × 40KB | one slot per message |
| Gesture templates | `0xE6000` | 4KB | all templates in one page |

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
| `RECG,<slot>` | Record a gesture template (2s to get ready, one beep, 1.3s capture, two beeps = saved) |
| `PLAY,<slot>` | Play a message's voice clip |
| `DEL,<slot>` | Erase a message's clip and gesture |
| `GT,<value>` | Set the gesture match threshold (default 3.0) |
| `AUDIO,<slot>,<bytes>` … `AUDIOEND` | Voice clip upload (used by the app) |
| `V,<0-32767>` | Test tone amplitude |
| `T` | Play a test tone |

Slots: `0` Pain · `1` Hunger · `2` Toilet · `3` Sleep · `4` Seizure · `5` Yes · `6` No

### BLE service

- Service: `a5e7c000-9c3f-4a4e-8e1e-1a2b3c4d5e00`
- Command (write): `…5e01`
- Status (notify): `…5e02`
- Audio data (write w/o response): `…5e03`

Clients should negotiate a larger MTU — the default 23-byte ATT MTU truncates status messages to ~20 bytes and makes clip upload very slow.

## Android app

Kotlin, minSdk 26, no Compose. One row per message with **Gesture / Voice / Play / Delete**. Voice is captured with `AudioRecord` (raw PCM16 @ 8kHz), peak-normalised, then streamed to the device in MTU-sized chunks with flow control driven by `onCharacteristicWrite`.

```bash
cd app
./gradlew assembleDebug
```

Output: `app/build/outputs/apk/debug/app-debug.apk` (debug-signed, sideloadable).

## Not covered here

The source specification also calls for a status display, a dedicated physical emergency button, an activation lock, rechargeable battery with all-day runtime, and waterproof charging. Those are hardware requirements and aren't addressed by this firmware/app.

## Recent changes

- **2026-09-23** — Replaced body-part poses with the 7-message gesture vocabulary; added recordable gestures (DTW matching) and caregiver-recorded voice clips stored in internal flash and uploaded over BLE.
- **2026-09-19** — Created public GitHub repo, combined firmware and Android app into one repository.
