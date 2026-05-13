#include "CD74HC4067.h"

CD74HC4067::CD74HC4067(uint8_t s0, uint8_t s1, uint8_t s2, uint8_t s3, uint8_t com_pin)
  : _s0(s0), _s1(s1), _s2(s2), _s3(s3), _com(com_pin) {}

void CD74HC4067::setup() {
    pinMode(_s0, OUTPUT);
    pinMode(_s1, OUTPUT);
    pinMode(_s2, OUTPUT);
    pinMode(_s3, OUTPUT);
}

void CD74HC4067::selectChannel(uint8_t channel) {
    // Ensure channel is 0-15
    channel &= 0x0F;

    digitalWrite(_s0, (channel & 0x01) ? HIGH : LOW);
    digitalWrite(_s1, (channel & 0x02) ? HIGH : LOW);
    digitalWrite(_s2, (channel & 0x04) ? HIGH : LOW);
    digitalWrite(_s3, (channel & 0x08) ? HIGH : LOW);
}

uint16_t CD74HC4067::readChannel(uint8_t channel) {
    selectChannel(channel);
    delayMicroseconds(10);
    return analogRead(_com);
}
