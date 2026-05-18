#include "MuxSensor.h"

const uint8_t MuxSensor::_muxTable[MUX_NUM_CHANNELS][4] = {
  {0,0,0,0}, {0,0,0,1}, {0,0,1,0}, {0,0,1,1},
  {0,1,0,0}, {0,1,0,1}, {0,1,1,0}, {0,1,1,1},
  {1,0,0,0}, {1,0,0,1}, {1,0,1,0}, {1,0,1,1},
  {1,1,0,0}, {1,1,0,1}, {1,1,1,0}, {1,1,1,1},
};

MuxSensor::MuxSensor(uint8_t pinS0, uint8_t pinS1, uint8_t pinS2,
                     uint8_t pinS3, uint8_t pinCOM, MuxPolarity polarity)
  : _pinS0(pinS0), _pinS1(pinS1), _pinS2(pinS2), _pinS3(pinS3),
    _pinCOM(pinCOM), _polarity(polarity), _calibrated(false)
{
  for (uint8_t i = 0; i < MUX_NUM_CHANNELS; i++) {
    _calMin[i]      = 0;
    _calMax[i]      = 1023;
    _lastDigital[i] = 0;
  }
}

void MuxSensor::begin() {
  pinMode(_pinS0, OUTPUT);
  pinMode(_pinS1, OUTPUT);
  pinMode(_pinS2, OUTPUT);
  pinMode(_pinS3, OUTPUT);
  _selectChannel(0);
}

void MuxSensor::_selectChannel(uint8_t ch) {
  digitalWrite(_pinS3, _muxTable[ch][0]);
  digitalWrite(_pinS2, _muxTable[ch][1]);
  digitalWrite(_pinS1, _muxTable[ch][2]);
  digitalWrite(_pinS0, _muxTable[ch][3]);
}

uint16_t MuxSensor::_readChannel(uint8_t ch) {
  _selectChannel(ch);
  delayMicroseconds(MUX_SETTLE_US);
  return (uint16_t)analogRead(_pinCOM);
}

void MuxSensor::getRawAnalogValues(uint16_t out[MUX_NUM_CHANNELS]) {
  for (uint8_t ch = 0; ch < MUX_NUM_CHANNELS; ch++) {
    out[ch] = _readChannel(ch);
  }
}

bool MuxSensor::calibrate(uint32_t durationMs) {
  for (uint8_t i = 0; i < MUX_NUM_CHANNELS; i++) {
    _calMin[i] = 1023;
    _calMax[i] = 0;
  }

  uint32_t deadline = millis() + durationMs;
  while ((int32_t)(millis() - deadline) < 0) {
    for (uint8_t ch = 0; ch < MUX_NUM_CHANNELS; ch++) {
      uint16_t v = _readChannel(ch);
      if (v < _calMin[ch]) _calMin[ch] = v;
      if (v > _calMax[ch]) _calMax[ch] = v;
    }
    delay(1);
  }

  for (uint8_t i = 0; i < MUX_NUM_CHANNELS; i++) {
    _lastDigital[i] = 0;
  }

  bool allOk = true;
  for (uint8_t i = 0; i < MUX_NUM_CHANNELS; i++) {
    uint16_t blackTop   = _calMin[i] + MUX_CALIB_MARGIN;
    uint16_t whiteBottom = (_calMax[i] >= MUX_CALIB_MARGIN)
                             ? _calMax[i] - MUX_CALIB_MARGIN : 0;
    if (blackTop >= whiteBottom) allOk = false;
  }

  _calibrated = true;
  return allOk;
}

bool MuxSensor::getDigital(uint8_t out[MUX_NUM_CHANNELS]) {
  if (!_calibrated) return false;

  uint16_t raw[MUX_NUM_CHANNELS];
  getRawAnalogValues(raw);

  for (uint8_t ch = 0; ch < MUX_NUM_CHANNELS; ch++) {
    uint16_t v = raw[ch];

    uint16_t blackLo = (_calMin[ch] >= MUX_CALIB_MARGIN) ? _calMin[ch] - MUX_CALIB_MARGIN : 0;
    uint16_t blackHi = _calMin[ch] + MUX_CALIB_MARGIN;
    uint16_t whiteLo = (_calMax[ch] >= MUX_CALIB_MARGIN) ? _calMax[ch] - MUX_CALIB_MARGIN : 0;
    uint16_t whiteHi = _calMax[ch] + MUX_CALIB_MARGIN;

    if (_polarity == POLARITY_DARK_LOW) {
      if (v >= blackLo && v <= blackHi) {
        _lastDigital[ch] = 1;   // inside black zone
      } else if (v >= whiteLo && v <= whiteHi) {
        _lastDigital[ch] = 0;   // inside white zone
      }
    } else {
      // POLARITY_DARK_HIGH: high reading = black
      if (v >= whiteLo && v <= whiteHi) {
        _lastDigital[ch] = 1;   // high = black
      } else if (v >= blackLo && v <= blackHi) {
        _lastDigital[ch] = 0;   // low = white
      }
    }

    out[ch] = _lastDigital[ch];
  }
  return true;
}

uint16_t MuxSensor::getCalibMin(uint8_t ch) const {
  return ch < MUX_NUM_CHANNELS ? _calMin[ch] : 0;
}
uint16_t MuxSensor::getCalibMax(uint8_t ch) const {
  return ch < MUX_NUM_CHANNELS ? _calMax[ch] : 0;
}