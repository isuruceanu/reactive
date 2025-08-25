#ifndef SHIFT_REGISTER_H
#define SHIFT_REGISTER_H

#include <Arduino.h>

class ShiftRegister {
  public:
    ShiftRegister(uint8_t dataPin, uint8_t clockPin, uint8_t latchPin, uint8_t ledCount = 24);
    
    void begin();
    void next(bool turnOffPrevious = true);
    void previous(bool turnOffCurrent = true);
    void set(uint8_t index, bool state);
    void clearAll();

  private:
    void updateShiftRegister();
    
    uint8_t _dataPin, _clockPin, _latchPin;
    uint8_t _ledCount;
    int _currentIndex;
    uint32_t _ledState;  // up to 32 bits, enough for 24 LEDs
};

#endif