# MuxSensor

Custom Library for reading sensor inputs from 16CMIRSA

---

## Table of contents

- [Overview](#overview)
- [Installation](#installation)
- [Constants](#constants)
- [Enums](#enums)
- [Constructor](#constructor)
- [API reference](#api-reference)
- [Calibration details](#calibration-details)
- [Digital thresholding](#digital-thresholding)
- [Channel select table](#channel-select-table)
- [Example sketch](#example-sketch)
- [Debug snippet](#debug-snippet)
- [Notes and caveats](#notes-and-caveats)

---

## Overview

`MuxSensor` controls a 16-to-1 analog multiplexer using four digital select lines (S0–S3) and one analog input (COM). It reads each of the 16 channels in sequence, runs a live calibration pass to record each sensor's min/max ADC extremes, and converts raw 10-bit ADC values to digital 0/1 signals using per-channel hysteresis windows.

Designed for **line-following robots** and **barcode scanners** where a bank of IR sensors (e.g. TCRT5000) feeds a single ADC pin through a mux.

---

## Installation

Copy `MuxSensor.h` and `MuxSensor.cpp` into your sketch folder or Arduino libraries directory, then include the header:

```cpp
#include "MuxSensor.h"
```

---

## Constants

| Macro | Value | Description |
|---|---|---|
| `MUX_NUM_CHANNELS` | `16` | Total channels on the multiplexer. |
| `MUX_CALIB_MARGIN` | `30` | ±ADC counts around each calibrated extreme forming the hysteresis window. Reduces noise-induced toggling. |
| `MUX_SETTLE_US` | `200` | Microseconds to wait after switching the mux channel before sampling. Allows the analog signal to settle. |

---

## Enums

### `MuxPolarity`

Describes how the sensor's ADC output maps to surface colour.

| Value | Description |
|---|---|
| `POLARITY_DARK_LOW` | Black surface → low ADC reading. Typical for TCRT5000 and most reflective IR sensors. **Default.** |
| `POLARITY_DARK_HIGH` | Black surface → high ADC reading. Use when your sensor circuit is inverted. |

---

## Constructor

```cpp
MuxSensor(uint8_t pinS0, uint8_t pinS1, uint8_t pinS2, uint8_t pinS3,
          uint8_t pinCOM,
          MuxPolarity polarity = POLARITY_DARK_LOW);
```

Creates a `MuxSensor` instance. Does **not** configure any pins — call `begin()` after construction.

| Parameter | Type | Description |
|---|---|---|
| `pinS0`–`pinS3` | `uint8_t` | Digital output pins connected to the mux select lines S0, S1, S2, S3. |
| `pinCOM` | `uint8_t` | Analog input pin connected to the mux common output (Z pin). |
| `polarity` | `MuxPolarity` | Optional. Sensor polarity. Defaults to `POLARITY_DARK_LOW`. |

All calibration arrays are initialised to `_calMin = 0`, `_calMax = 1023`, and `_calibrated = false`.

---

## API reference

### `begin()`

```cpp
void begin();
```

Sets S0–S3 as `OUTPUT` and selects channel 0. Call once in `setup()`.

> **Note:** Does not call `pinMode` on the COM pin. Make sure the COM pin is not configured as a digital output elsewhere in your sketch.

---

### `calibrate()`

```cpp
bool calibrate(uint32_t durationMs = 12000UL);
```

Sweeps all 16 channels continuously for `durationMs` milliseconds, recording the minimum and maximum ADC value seen on each channel. Move the sensor array over both black and white surfaces during this window.

| Parameter | Type | Description |
|---|---|---|
| `durationMs` | `uint32_t` | Calibration window in milliseconds. Default `12000` ms. |

**Returns** `true` if every channel has sufficient contrast — i.e. `calMin[i] + MUX_CALIB_MARGIN < calMax[i] - MUX_CALIB_MARGIN`. Returns `false` if any channel's range is too narrow, but still sets `_calibrated = true` so subsequent reads can proceed.

> **Warning:** This call **blocks the main loop** for the full duration. Use a non-blocking state machine if that is a concern.

---

### `isCalibrated()`

```cpp
bool isCalibrated() const;
```

Returns `true` after a successful `calibrate()` call, regardless of whether contrast was adequate on all channels.

---

### `getRawAnalogValues()`

```cpp
void getRawAnalogValues(uint16_t out[MUX_NUM_CHANNELS]);
```

Reads all 16 channels in order (ch0 → ch15) and writes the raw 10-bit ADC value (0–1023) into `out`. Does not require prior calibration.

| Parameter | Type | Description |
|---|---|---|
| `out` | `uint16_t[16]` | Caller-allocated array of 16 elements. Filled in channel order. |

---

### `getDigital()`

```cpp
bool getDigital(uint8_t out[MUX_NUM_CHANNELS]);
```

Reads all channels, applies the calibrated hysteresis windows, and writes a digital `0` or `1` per channel into `out`.

| Parameter | Type | Description |
|---|---|---|
| `out` | `uint8_t[16]` | Caller-allocated array. Each element is `0` (white / no surface) or `1` (black / surface detected), depending on polarity. |

**Returns** `false` immediately — leaving `out` unchanged — if `_calibrated` is `false`. Returns `true` on success.

If the current reading falls outside both the black and white zones, the previous digital value for that channel is retained (`_lastDigital[ch]`). This prevents glitching in mid-transition regions.

---

### `getCalibMin()`

```cpp
uint16_t getCalibMin(uint8_t ch) const;
```

Returns the minimum ADC value recorded for channel `ch` during the last calibration. Returns `0` for out-of-range channel indices.

---

### `getCalibMax()`

```cpp
uint16_t getCalibMax(uint8_t ch) const;
```

Returns the maximum ADC value recorded for channel `ch` during the last calibration. Returns `0` for out-of-range channel indices.

---

### `getThreshold()`

```cpp
uint16_t getThreshold(uint8_t ch) const;
```

Returns the midpoint `(calMin[ch] + calMax[ch]) / 2` for channel `ch`. Useful as a simple fixed threshold for debugging. Returns `0` for out-of-range channel indices.

---

## Calibration details

During `calibrate()`, the library:

1. Resets `_calMin[i]` to `1023` and `_calMax[i]` to `0` for all channels.
2. Polls all 16 channels in a tight loop (with 1 ms delay per sweep) until the deadline is reached.
3. After the sweep, checks whether each channel has enough contrast: `_calMin[i] + MUX_CALIB_MARGIN < _calMax[i] - MUX_CALIB_MARGIN`.
4. Sets `_calibrated = true` unconditionally and returns the contrast check result.

For reliable calibration, ensure the sensor passes over a clear black surface **and** a clear white surface during the calibration window.

---

## Digital thresholding

`getDigital()` defines two zones per channel based on calibrated extremes and `MUX_CALIB_MARGIN`:

```
blackLo = calMin[ch] - MUX_CALIB_MARGIN   (clamped to 0)
blackHi = calMin[ch] + MUX_CALIB_MARGIN

whiteLo = calMax[ch] - MUX_CALIB_MARGIN   (clamped to 0)
whiteHi = calMax[ch] + MUX_CALIB_MARGIN
```

The mapping of zones to digital output depends on polarity:

| Zone | `POLARITY_DARK_LOW` output | `POLARITY_DARK_HIGH` output |
|---|---|---|
| Reading in black zone | `1` | `0` |
| Reading in white zone | `0` | `1` |
| Reading in neither zone | retain previous value | retain previous value |

---

## Channel select table

The CD74HC4067 select lines S3, S2, S1, S0 address channels 0–15 as follows:

| Channel | S3 | S2 | S1 | S0 |
|---|---|---|---|---|
| 0  | 0 | 0 | 0 | 0 |
| 1  | 0 | 0 | 0 | 1 |
| 2  | 0 | 0 | 1 | 0 |
| 3  | 0 | 0 | 1 | 1 |
| 4  | 0 | 1 | 0 | 0 |
| 5  | 0 | 1 | 0 | 1 |
| 6  | 0 | 1 | 1 | 0 |
| 7  | 0 | 1 | 1 | 1 |
| 8  | 1 | 0 | 0 | 0 |
| 9  | 1 | 0 | 0 | 1 |
| 10 | 1 | 0 | 1 | 0 |
| 11 | 1 | 0 | 1 | 1 |
| 12 | 1 | 1 | 0 | 0 |
| 13 | 1 | 1 | 0 | 1 |
| 14 | 1 | 1 | 1 | 0 |
| 15 | 1 | 1 | 1 | 1 |

---

## Example sketch

```cpp
#include "MuxSensor.h"

// S0=4, S1=5, S2=6, S3=7, COM=A0
MuxSensor mux(4, 5, 6, 7, A0, POLARITY_DARK_LOW);

void setup() {
  Serial.begin(115200);
  mux.begin();

  Serial.println("Calibrating — move over black and white...");
  bool ok = mux.calibrate(5000);  // 5-second window
  Serial.println(ok ? "Calibrated OK" : "Low contrast on some channels");
}

void loop() {
  uint8_t bits[MUX_NUM_CHANNELS];
  if (mux.getDigital(bits)) {
    for (uint8_t i = 0; i < MUX_NUM_CHANNELS; i++) {
      Serial.print(bits[i]);
    }
    Serial.println();
  }
  delay(20);
}
```

---

## Debug snippet

Print calibration data for every channel after `calibrate()`:

```cpp
for (uint8_t i = 0; i < MUX_NUM_CHANNELS; i++) {
  Serial.print("ch");    Serial.print(i);
  Serial.print("  min="); Serial.print(mux.getCalibMin(i));
  Serial.print("  max="); Serial.print(mux.getCalibMax(i));
  Serial.print("  thr="); Serial.println(mux.getThreshold(i));
}
```

---

## Notes and caveats

- `calibrate()` always sets `_calibrated = true` even when it returns `false` (poor contrast). Reads via `getDigital()` will proceed — results may be unreliable on affected channels.
- `getThreshold()` is declared in the header but may not be present in all versions of the `.cpp`. Verify it is implemented before use.
- `MUX_SETTLE_US` (200 µs) is conservative. On fast MCUs you may be able to reduce it if your mux and sensor settle faster, saving time during full sweeps.
- Total scan time per full `getDigital()` call is approximately `16 × (MUX_SETTLE_US + analogRead time)` — roughly 3–5 ms on a standard Arduino Uno at default ADC prescaler.
