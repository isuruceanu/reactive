#ifndef COMPARATOR_H
#define COMPARATOR_H

#include <Arduino.h>

enum ZeroCrossType {
  NO_CROSS,
  ZC_RISING,
  ZC_FALLING
};

class Comparator {
  public:
    Comparator(uint8_t analogPin, int threshold, int samplesRequired = 2, float frequesncyHz = 50.0);
    
    void setThreshold(int threshold);
    IRAM_ATTR ZeroCrossType detect(unsigned long timeoutMs = 20);
    int calibrateZeroOffset(int samples = 1000);
  protected:
    uint8_t _analogPin;
    float _freq;
    int _threshold;
  private:
    int _samplesRequired;
    portMUX_TYPE _mux = portMUX_INITIALIZER_UNLOCKED;
};

#endif