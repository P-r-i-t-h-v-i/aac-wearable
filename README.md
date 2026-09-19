# AAC Wearable

A wearable AAC (augmentative and alternative communication) device built on a Seeed XIAO nRF52840 Sense. It detects which body part it's being held against (via the onboard IMU) and speaks the corresponding word, plus a double-tap "pain" alert and a shake-triggered seizure alarm. Calibration can be done over USB serial or wirelessly via a companion Android app over BLE.

## Structure

- `firmware/` — Arduino sketch (`aac-wearable.ino`) for the XIAO nRF52840 Sense. Handles pose detection, I2S audio playback, pain/seizure detection, and a BLE calibration service.
- `app/` — Native Android app (Kotlin) for calibrating the device over Bluetooth Low Energy.

## Firmware

**Board:** Seeed XIAO nRF52840 Sense
**Core:** `Seeeduino:mbed`
**Dependencies:** `Seeed Arduino LSM6DS3`, `ArduinoBLE`

Build/flash with `arduino-cli`:

```bash
arduino-cli compile --fqbn Seeeduino:mbed:xiaonRF52840Sense firmware
arduino-cli upload -p /dev/ttyACM0 --fqbn Seeeduino:mbed:xiaonRF52840Sense firmware
```

### Calibration (USB serial, 115200 baud)

| Command | Effect |
|---|---|
| `CAL1` / `CAL2` / `CAL3` / `CAL0` | Calibrate HEAD / CHEST / STOMACH / REST (2s to position, one beep, 5s hold, two beeps = saved) |
| `SHOW` | Print current calibration |
| `H` / `C` / `S` | Test-play HEAD / CHEST / STOMACH audio |
| `E` | Test-play seizure alarm |
| `P1` / `P2` / `P3` | Test-play pain alarm for HEAD / CHEST / STOMACH |
| `V,<0-32767>` | Set test tone amplitude |

The same commands work over the BLE calibration service (see below), so the Android app is optional — everything is also reachable from the Arduino serial monitor.

### BLE service

- Service UUID: `a5e7c000-9c3f-4a4e-8e1e-1a2b3c4d5e00`
- Command characteristic (write): `a5e7c000-9c3f-4a4e-8e1e-1a2b3c4d5e01` — same command strings as the serial interface
- Status characteristic (notify): `a5e7c000-9c3f-4a4e-8e1e-1a2b3c4d5e02` — live status/results

## Android App

Native Kotlin app (no Compose), minSdk 26. Connects to the device over BLE and exposes calibration + test-playback buttons.

Build:

```bash
cd app
./gradlew assembleDebug
```

Output: `app/app/build/outputs/apk/debug/app-debug.apk` (debug-signed, sideloadable).

## Recent changes

- **2026-09-19** — Created public GitHub repo, combined firmware and Android app into one repository.
