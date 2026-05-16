#include "MuxSensor.h"

const uint8_t MuxSensor::_muxTable[MUX_NUM_CHANNELS][4] = {
  // S3  S2  S1  S0
  {  0,  0,  0,  0 }, //  0
  {  0,  0,  0,  1 }, //  1
  {  0,  0,  1,  0 }, //  2
  {  0,  0,  1,  1 }, //  3
  {  0,  1,  0,  0 }, //  4
  {  0,  1,  0,  1 }, //  5
  {  0,  1,  1,  0 }, //  6
  {  0,  1,  1,  1 }, //  7
  {  1,  0,  0,  0 }, //  8
  {  1,  0,  0,  1 }, //  9
  {  1,  0,  1,  0 }, // 10
  {  1,  0,  1,  1 }, // 11
  {  1,  1,  0,  0 }, // 12
  {  1,  1,  0,  1 }, // 13
  {  1,  1,  1,  0 }, // 14
  {  1,  1,  1,  1 }, // 15
};

// ── constructor
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

// ── begin
void MuxSensor::begin() {
  pinMode(_pinS0, OUTPUT);
  pinMode(_pinS1, OUTPUT);
  pinMode(_pinS2, OUTPUT);
  pinMode(_pinS3, OUTPUT);
  _selectChannel(0);
}

// ── private: select one MUX channel
void MuxSensor::_selectChannel(uint8_t ch) {
  digitalWrite(_pinS3, _muxTable[ch][0]);
  digitalWrite(_pinS2, _muxTable[ch][1]);
  digitalWrite(_pinS1, _muxTable[ch][2]);
  digitalWrite(_pinS0, _muxTable[ch][3]);
}

// ── private: select channel, settle, sample
uint16_t MuxSensor::_readChannel(uint8_t ch) {
  _selectChannel(ch);
  delayMicroseconds(200); // 200ms settle time
  return analogRead(_pinCOM);
}

// ── getRawAnalogValues
void MuxSensor::getRawAnalogValues(uint16_t readings[MUX_NUM_CHANNELS]) {
  for (uint8_t ch = 0; ch < MUX_NUM_CHANNELS; ch++) {
    readings[ch] = _readChannel(ch);
  }
}

// ── calibrate
bool MuxSensor::calibrate(uint32_t durationMs) {
  // Reset previous calibration
  for (uint8_t i = 0; i < MUX_NUM_CHANNELS; i++) {
    _calMin[i] = 1023;
    _calMax[i] = 0;
  }

  uint32_t start = millis();
  while (millis() - start < durationMs) {
    for (uint8_t ch = 0; ch < MUX_NUM_CHANNELS; ch++) {
      uint16_t val = _readChannel(ch);
      if (val < _calMin[ch]) _calMin[ch] = val;
      if (val > _calMax[ch]) _calMax[ch] = val;
    }
    delay(1);
  }

  bool ok = true;
  for (uint8_t i = 0; i < MUX_NUM_CHANNELS; i++) {
    // Apply the ±30 margin
    _calMin[i] = (_calMin[i] > MUX_CALIB_MARGIN)
                   ? _calMin[i] - MUX_CALIB_MARGIN : 0;
    _calMax[i] = (_calMax[i] + MUX_CALIB_MARGIN < 1023)
                   ? _calMax[i] + MUX_CALIB_MARGIN : 1023;

    if (_calMax[i] - _calMin[i] < 2 * MUX_CALIB_MARGIN) {
        // if calibration failed return false
      ok = false;
    }
    _threshold[i] = (_calMin[i] + _calMax[i]) / 2;
  }

  _calibrated = true;
  return ok;
}

// ── getDigital
bool MuxSensor::getDigital(uint8_t digital[MUX_NUM_CHANNELS]) {
  if (!_calibrated) return false;

  uint16_t raw[MUX_NUM_CHANNELS];
  getRawAnalogValues(raw);

  for (uint8_t ch = 0; ch < MUX_NUM_CHANNELS; ch++) {
    bool overLine;
    if (_polarity == POLARITY_DARK_LOW) {
      // LOW reading
      overLine = (raw[ch] <= _threshold[ch]);
    } else {
      // HIGH reading
      overLine = (raw[ch] >= _threshold[ch]);
    }
    digital[ch] = overLine ? 1 : 0;
  }
  return true;
}

// ── calibration inspection
uint16_t MuxSensor::getCalibMin(uint8_t ch) const {
  return (ch < MUX_NUM_CHANNELS) ? _calMin[ch] : 0;
}
uint16_t MuxSensor::getCalibMax(uint8_t ch) const {
  return (ch < MUX_NUM_CHANNELS) ? _calMax[ch] : 0;
}