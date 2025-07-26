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
    Comparator(uint8_t analogPin, int threshold, int samplesRequired = 3, float frequesncyHz = 50.0);
    void setFrequency(float freqHz);
    IRAM_ATTR ZeroCrossType detect(unsigned long timeoutMs = 20);

  private:
    uint8_t _analogPin;
    int _threshold;
    int _samplesRequired;
    float _freq;
    portMUX_TYPE _mux = portMUX_INITIALIZER_UNLOCKED;
    int _sampleDelayUs;
    void updateSamplingDelay();
};

#endif