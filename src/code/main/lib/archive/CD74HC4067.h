#ifndef CD74HC4067_h
#define CD74HC4067_h

#include "Arduino.h"

class CD74HC4067 {
  public:
    CD74HC4067(uint8_t s0, uint8_t s1, uint8_t s2, uint8_t s3, uint8_t com_pin);
    uint16_t readChannel(uint8_t channel);
    void selectChannel(uint8_t channel);
    void setup();

  private:
    uint8_t _s0, _s1, _s2, _s3, _com;
};

#endif
