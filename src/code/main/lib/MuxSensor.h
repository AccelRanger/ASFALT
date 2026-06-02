#pragma once
#include <Arduino.h>

#define MUX_NUM_CHANNELS  16
#define MUX_CALIB_MARGIN  30    // ± ADC counts around each calibrated extreme
#define MUX_SETTLE_US     200   // µs to settle after switching MUX channel

enum MuxPolarity {
  POLARITY_DARK_LOW,    // black → LOW  reading  (typical TCRT5000)
  POLARITY_DARK_HIGH    // black → HIGH reading
};

class MuxSensor {
public:
  MuxSensor(uint8_t pinS0, uint8_t pinS1, uint8_t pinS2, uint8_t pinS3,
            uint8_t pinCOM,
            MuxPolarity polarity = POLARITY_DARK_LOW);

  void begin();

  bool calibrate(uint32_t durationMs = 12000UL);

  bool isCalibrated() const { return _calibrated; }

  // Raw 10-bit ADC values for all 16 channels.
  void getRawAnalogValues(uint16_t out[MUX_NUM_CHANNELS]);
  bool getDigital(uint8_t out[MUX_NUM_CHANNELS]);

  // Debug
  uint16_t getCalibMin(uint8_t ch) const;
  uint16_t getCalibMax(uint8_t ch) const;

private:
  static const uint8_t _muxTable[MUX_NUM_CHANNELS][4];

  uint8_t     _pinS0, _pinS1, _pinS2, _pinS3, _pinCOM;
  MuxPolarity _polarity;
  bool        _calibrated;

  uint16_t _calMin[MUX_NUM_CHANNELS];
  uint16_t _calMax[MUX_NUM_CHANNELS];
  uint8_t  _lastDigital[MUX_NUM_CHANNELS];

  void     _selectChannel(uint8_t ch);
  uint16_t _readChannel(uint8_t ch);
};