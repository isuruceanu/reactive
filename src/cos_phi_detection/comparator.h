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
    typedef void (*CallbackFunction)(ZeroCrossType);

    Comparator(uint8_t analogPin, int threshold, CallbackFunction cb, int samplesRequired = 3);
    void start();
    void loop(bool stopOnFire = false);  // Call this from loop()
    void stop();

  private:
    uint8_t _analogPin;
    int _threshold;
    int _samplesRequired;
    CallbackFunction _callback;
    bool _running = false;
    int _prevState = 0;
};

#endif