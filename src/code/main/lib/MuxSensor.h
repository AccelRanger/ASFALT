#pragma once
#include <Arduino.h>

#define MUX_NUM_CHANNELS   16
#define MUX_CALIB_MARGIN   30 // calib margin in ADC
#define MUX_CALIB_MS       12000  // calibration duration in ms

enum MuxPolarity { POLARITY_DARK_LOW, POLARITY_DARK_HIGH };

class MuxSensor {
public:
  MuxSensor(uint8_t pinS0, uint8_t pinS1, uint8_t pinS2, uint8_t pinS3,
            uint8_t pinCOM,
            MuxPolarity polarity = POLARITY_DARK_LOW);
 // initialization
  void begin();
 
  // ── calibration ────────────────────────────────────────────────────────────
  bool calibrate(uint32_t durationMs = 12000UL);
 
  bool isCalibrated() const { return _calibrated; }
 
  // ── raw readings ───────────────────────────────────────────────────────────
  void getRawAnalogValues(uint16_t out[MUX_NUM_CHANNELS]);
 
  // ── digital readings ──────────────────────────────────────────────────────
  bool getDigital(uint8_t out[MUX_NUM_CHANNELS]);
 
  // Debug helpers
  uint16_t getCalibMin(uint8_t ch) const;
  uint16_t getCalibMax(uint8_t ch) const;
  uint16_t getThreshold(uint8_t ch) const;
 
private:
  static const uint8_t _muxTable[MUX_NUM_CHANNELS][4]; // [ch][S3,S2,S1,S0]
 
  uint8_t     _pinS0, _pinS1, _pinS2, _pinS3;
  uint8_t     _pinCOM;
  MuxPolarity _polarity;
  bool        _calibrated;
 
  uint16_t _calMin[MUX_NUM_CHANNELS];
  uint16_t _calMax[MUX_NUM_CHANNELS];
  uint16_t _threshold[MUX_NUM_CHANNELS];
 
  void     _selectChannel(uint8_t ch);
  uint16_t _readChannel(uint8_t ch);
};