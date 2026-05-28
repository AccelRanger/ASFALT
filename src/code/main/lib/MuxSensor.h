#pragma once
#include <Arduino.h>

#define MUX_NUM_CHANNELS  16
#define MUX_CALIB_MARGIN  30    // hysteresis band around cal min/max (ADC counts)
#define MUX_SETTLE_US     10    // microseconds to wait after channel select

enum MuxPolarity { POLARITY_DARK_LOW, POLARITY_DARK_HIGH };

class MuxSensor {
public:
  MuxSensor(uint8_t pinS0, uint8_t pinS1, uint8_t pinS2, uint8_t pinS3,
            uint8_t pinCOM,
            MuxPolarity polarity = POLARITY_DARK_LOW);

  void begin();

  // Sweep sensor over black & white for durationMs, records min/max per channel.
  // Returns true if every channel saw enough contrast.
  bool calibrate(uint32_t durationMs = 12000UL);

  bool isCalibrated() const { return _calibrated; }

  // Fill out[16] with 1 = on line, 0 = off line.
  // Returns false if not yet calibrated.
  bool getDigital(uint8_t out[MUX_NUM_CHANNELS]);

  // Raw ADC reads (0-1023) for all channels.
  void getRawAnalogValues(uint16_t out[MUX_NUM_CHANNELS]);

  uint16_t getCalibMin(uint8_t ch) const;
  uint16_t getCalibMax(uint8_t ch) const;
  uint16_t getThreshold(uint8_t ch) const;

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
