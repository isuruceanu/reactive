#include "comparator.h"

Comparator::Comparator(uint8_t analogPin, int threshold, CallbackFunction cb, int samplesRequired) {
  _analogPin = analogPin;
  _threshold = threshold;
  _samplesRequired = samplesRequired;
  _callback = cb;
}

void Comparator::start() {
  int prevReading = analogRead(_analogPin);
  _prevState = (prevReading > _threshold) ? 1 : 0;
  _running = true;

}

void Comparator::stop() {
  _running = false;
}

void Comparator::loop(bool stopOnFire) {

  if (!_running) return;

  int consistentState = 0;
  int stableCount = 0;

  for (int i = 0; i < _samplesRequired; i++) {
    int val = analogRead(_analogPin);
    int currentState = (val > _threshold) ? 1 : 0;

    if (currentState == consistentState) {
      stableCount++;
    } else {
      consistentState = currentState;
      stableCount = 1;
    }

    //delayMicroseconds(1);  // Small delay between samples
  }

  if (stableCount >= _samplesRequired) {
    if (_prevState == 0 && consistentState == 1) {
      if (stopOnFire) stop();
      _callback(ZC_RISING);
      return;
    } else if (_prevState == 1 && consistentState == 0) {
      if (stopOnFire) stop();
      _callback(ZC_FALLING);
      return;
    }
    _prevState = consistentState;
  }
}