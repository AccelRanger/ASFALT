#ifndef MuxReader_h
#define MuxReader_h

#include <Arduino.h>

// ──────────────────────────────────────────────────────────────
//  MuxReader  –  16-channel analog multiplexer library
//  Sensor    : TCRT5000 IR reflectance sensors
//  MCU       : Arduino Nano / ATmega328P
//
//  Wiring (default, overridable in constructor):
//    PORT A (S0) → D8
//    PORT B (S1) → D10
//    PORT C (S2) → D11
//    PORT D (S3) → D12
//    OUT (SIG)   → A0
//
//  readSum()      → sum of all 16 readings, 0-16368
//  readPosition() → weighted line position, 0-15000 (7500=center)
//                   returns MUX_NO_LINE (-1) if no line detected
// ──────────────────────────────────────────────────────────────

#define MUX_CHANNELS  16
#define MUX_MAX_SUM   (MUX_CHANNELS * 1023)   // 16368
#define MUX_NO_LINE   -1

class MuxReader {
public:
    MuxReader(uint8_t pinA   = 8,
              uint8_t pinB   = 10,
              uint8_t pinC   = 11,
              uint8_t pinD   = 12,
              uint8_t sigPin = A0);

    void begin();

    int readSum();

    int readPosition();
    int  readChannel(uint8_t channel);

    void readAll(int results[MUX_CHANNELS]);

    void setInvert(bool invert);

    void setNoLineThreshold(int threshold);

    void setSettleTime(uint16_t microseconds);

private:
    uint8_t  _pinA, _pinB, _pinC, _pinD;
    uint8_t  _sigPin;
    uint16_t _settleTime;
    bool     _invert;
    int      _noLineThreshold;

    void _selectChannel(uint8_t channel);
};

#endif
