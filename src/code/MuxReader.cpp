#include "MuxReader.h"

// ──────────────────────────────────────────────────────────────
//  Constructor
// ──────────────────────────────────────────────────────────────
MuxReader::MuxReader(uint8_t pinA,
                     uint8_t pinB,
                     uint8_t pinC,
                     uint8_t pinD,
                     uint8_t sigPin)
{
    _pinA             = pinA;
    _pinB             = pinB;
    _pinC             = pinC;
    _pinD             = pinD;
    _sigPin           = sigPin;
    _settleTime       = 10;
    _invert           = false;
    _noLineThreshold  = 50;
}

// ──────────────────────────────────────────────────────────────
//  begin()
// ──────────────────────────────────────────────────────────────
void MuxReader::begin()
{
    pinMode(_pinA,   OUTPUT);
    pinMode(_pinB,   OUTPUT);
    pinMode(_pinC,   OUTPUT);
    pinMode(_pinD,   OUTPUT);
    pinMode(_sigPin, INPUT);
    _selectChannel(0);
}

// ──────────────────────────────────────────────────────────────
//  readSum()
//  Returns the plain sum of all 16 ADC readings (0–16368).
// ──────────────────────────────────────────────────────────────
int MuxReader::readSum()
{
    int total = 0;
    for (uint8_t ch = 0; ch < MUX_CHANNELS; ch++) {
        int val = readChannel(ch);
        total  += _invert ? (1023 - val) : val;
    }
    return total;
}

// ──────────────────────────────────────────────────────────────
//  readPosition()
//  Weighted-average position of the line across all 16 sensors.
//
//  Formula:
//    position = SUM(ch * 1000 * adc) / SUM(adc)
//
//  Range  : 0 (line at ch0) … 15000 (line at ch15)
//  Center : 7500
//  No line: MUX_NO_LINE (-1) when total < _noLineThreshold
// ──────────────────────────────────────────────────────────────
int MuxReader::readPosition()
{
    long weightedSum = 0;
    long totalSum    = 0;

    for (uint8_t ch = 0; ch < MUX_CHANNELS; ch++) {
        int val = readChannel(ch);
        if (_invert) val = 1023 - val;
        weightedSum += (long)ch * 1000L * val;
        totalSum    += val;
    }

    if (totalSum < _noLineThreshold) {
        return MUX_NO_LINE;
    }

    return (int)(weightedSum / totalSum);   // 0 … 15000
}

// ──────────────────────────────────────────────────────────────
//  readChannel(ch)
// ──────────────────────────────────────────────────────────────
int MuxReader::readChannel(uint8_t channel)
{
    if (channel >= MUX_CHANNELS) return 0;
    _selectChannel(channel);
    delayMicroseconds(_settleTime);
    return analogRead(_sigPin);
}

// ──────────────────────────────────────────────────────────────
//  readAll(results)
// ──────────────────────────────────────────────────────────────
void MuxReader::readAll(int results[MUX_CHANNELS])
{
    for (uint8_t ch = 0; ch < MUX_CHANNELS; ch++) {
        results[ch] = readChannel(ch);
    }
}

// ──────────────────────────────────────────────────────────────
//  Setters
// ──────────────────────────────────────────────────────────────
void MuxReader::setInvert(bool invert)
{
    _invert = invert;
}

void MuxReader::setNoLineThreshold(int threshold)
{
    _noLineThreshold = threshold;
}

void MuxReader::setSettleTime(uint16_t microseconds)
{
    _settleTime = microseconds;
}

// ──────────────────────────────────────────────────────────────
//  _selectChannel
// ──────────────────────────────────────────────────────────────
void MuxReader::_selectChannel(uint8_t channel)
{
    digitalWrite(_pinA, (channel >> 0) & 0x01);  // S0
    digitalWrite(_pinB, (channel >> 1) & 0x01);  // S1
    digitalWrite(_pinC, (channel >> 2) & 0x01);  // S2
    digitalWrite(_pinD, (channel >> 3) & 0x01);  // S3
}
