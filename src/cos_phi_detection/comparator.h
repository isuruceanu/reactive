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
  

    Comparator(uint8_t analogPin, int threshold, int samplesRequired = 3, unsigned long timeoutMs = 20);

    ZeroCrossType read();  // Call this from loop()
    int getMinVal();
    int getMaxVal();
    void resetValues();

  private:
    uint8_t _analogPin;
    int _threshold;
    int _samplesRequired;
    unsigned long _timeoutMs;

    void setValues(int val);
    
    int minVal = 60000;
    int maxVal = 0;
};

#endif