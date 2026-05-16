#pragma once
#include <Arduino.h>

#define MUX_NUM_CHANNELS   16
#define MUX_CALIB_MARGIN   30      // ±30 raw-ADC counts around the calibrated edge
#define MUX_CALIB_MS       12000  // default calibration window: 12 s

enum MuxPolarity { POLARITY_DARK_LOW, POLARITY_DARK_HIGH };

class MuxSensor {
public:
  // ── constructor ────────────────────────────────────────────────────────────
  MuxSensor(uint8_t pinS0, uint8_t pinS1, uint8_t pinS2, uint8_t pinS3,
            uint8_t pinCOM,
            MuxPolarity polarity = POLARITY_DARK_LOW);

  // ── initialisation
  void begin();

  // ── calibration ────────────────────────────────────────────────────────────
  bool calibrate(uint32_t durationMs = MUX_CALIB_MS);

  // returns a bool if the sensor is calibrated or not
  bool isCalibrated() const { return _calibrated; }

  // ── raw readings ───────────────────────────────────────────────────────────

  void getRawAnalogValues(uint16_t readings[MUX_NUM_CHANNELS]);

  // ── digital readings ───────────────────────────────────────────────────────
  //   1 →  (black)
  //   0 →  (white)
  // needs calibrate() to run first
  bool getDigital(uint8_t digital[MUX_NUM_CHANNELS]);

  // ── calibration inspection ────────────────────────────────────────────────
  uint16_t getCalibMin(uint8_t ch) const;
  uint16_t getCalibMax(uint8_t ch) const;

private:
  //[channel][S3,S2,S1,S0]
  static const uint8_t _muxTable[MUX_NUM_CHANNELS][4];

  uint8_t    _pinS0, _pinS1, _pinS2, _pinS3;
  uint8_t    _pinCOM;
  MuxPolarity _polarity;
  bool       _calibrated;

  uint16_t   _calMin[MUX_NUM_CHANNELS];   // per-channel observed minimum
  uint16_t   _calMax[MUX_NUM_CHANNELS];   // per-channel observed maximum
  uint16_t   _threshold[MUX_NUM_CHANNELS]; // midpoint

  void _selectChannel(uint8_t ch);
  uint16_t _readChannel(uint8_t ch);
};