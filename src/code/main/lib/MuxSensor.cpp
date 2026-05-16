#include "MuxSensor.h"

const uint8_t MuxSensor::_muxTable[MUX_NUM_CHANNELS][4] = {
  {0,0,0,0}, {0,0,0,1}, {0,0,1,0}, {0,0,1,1},
  {0,1,0,0}, {0,1,0,1}, {0,1,1,0}, {0,1,1,1},
  {1,0,0,0}, {1,0,0,1}, {1,0,1,0}, {1,0,1,1},
  {1,1,0,0}, {1,1,0,1}, {1,1,1,0}, {1,1,1,1},
};

// ── constructor ──────────────────────────────────────────────────────────────
MuxSensor::MuxSensor(uint8_t pinS0, uint8_t pinS1, uint8_t pinS2,
                     uint8_t pinS3, uint8_t pinCOM, MuxPolarity polarity)
  : _pinS0(pinS0), _pinS1(pinS1), _pinS2(pinS2), _pinS3(pinS3),
    _pinCOM(pinCOM), _polarity(polarity), _calibrated(false)
{
  for (uint8_t i = 0; i < MUX_NUM_CHANNELS; i++) {
    _calMin[i]    = 1023;
    _calMax[i]    = 0;
    _threshold[i] = 512;
  }
}

// ── begin ────────────────────────────────────────────────────────────────────
void MuxSensor::begin() {
  pinMode(_pinS0, OUTPUT);
  pinMode(_pinS1, OUTPUT);
  pinMode(_pinS2, OUTPUT);
  pinMode(_pinS3, OUTPUT);
  _selectChannel(0); // default to channel 0
}

// ── private: drive the 4 select lines ────────────────────────────────────────
void MuxSensor::_selectChannel(uint8_t ch) {
  digitalWrite(_pinS3, _muxTable[ch][0]);
  digitalWrite(_pinS2, _muxTable[ch][1]);
  digitalWrite(_pinS1, _muxTable[ch][2]);
  digitalWrite(_pinS0, _muxTable[ch][3]);
}

// ── private: select + settle + sample ────────────────────────────────────────
uint16_t MuxSensor::_readChannel(uint8_t ch) {
  _selectChannel(ch);
  delayMicroseconds(MUX_SETTLE_US);
  return (uint16_t)analogRead(_pinCOM);
}

// ── getRawAnalogValues ────────────────────────────────────────────────────────
void MuxSensor::getRawAnalogValues(uint16_t out[MUX_NUM_CHANNELS]) {
  for (uint8_t ch = 0; ch < MUX_NUM_CHANNELS; ch++) {
    out[ch] = _readChannel(ch);
  }
}

// ── calibrate ────────────────────────────────────────────────────────────────
bool MuxSensor::calibrate(uint32_t durationMs) {
  // Reset
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

  bool allOk = true;
  for (uint8_t i = 0; i < MUX_NUM_CHANNELS; i++) {
    _calMin[i] = (_calMin[i] >= MUX_CALIB_MARGIN)
                   ? _calMin[i] - MUX_CALIB_MARGIN : 0;
    _calMax[i] = (_calMax[i] + MUX_CALIB_MARGIN <= 1023)
                   ? _calMax[i] + MUX_CALIB_MARGIN : 1023;

    if (_calMax[i] - _calMin[i] < 2 * MUX_CALIB_MARGIN) {
      allOk = false; // channel never saw meaningful contrast
    }
    _threshold[i] = (_calMin[i] + _calMax[i]) / 2;
  }

  _calibrated = true;
  return allOk;
}

// ── getDigital ────────────────────────────────────────────────────────────────
bool MuxSensor::getDigital(uint8_t out[MUX_NUM_CHANNELS]) {
  if (!_calibrated) return false;

  uint16_t raw[MUX_NUM_CHANNELS];
  getRawAnalogValues(raw);

  for (uint8_t ch = 0; ch < MUX_NUM_CHANNELS; ch++) {
    bool onLine = (_polarity == POLARITY_DARK_LOW)
                    ? (raw[ch] <= _threshold[ch])   // low = black = line
                    : (raw[ch] >= _threshold[ch]);  // high = black = line
    out[ch] = onLine ? 1 : 0;
  }
  return true;
}

// ── debug accessors ───────────────────────────────────────────────────────────
uint16_t MuxSensor::getCalibMin(uint8_t ch) const {
  return ch < MUX_NUM_CHANNELS ? _calMin[ch] : 0;
}
uint16_t MuxSensor::getCalibMax(uint8_t ch) const {
  return ch < MUX_NUM_CHANNELS ? _calMax[ch] : 0;
}
uint16_t MuxSensor::getThreshold(uint8_t ch) const {
  return ch < MUX_NUM_CHANNELS ? _threshold[ch] : 0;
}