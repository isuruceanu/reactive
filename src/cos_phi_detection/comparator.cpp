#include "comparator.h"

Comparator::Comparator(uint8_t analogPin, int threshold, int samplesRequired, unsigned long timeoutMs) {
  _analogPin = analogPin;
  _threshold = threshold;
  _samplesRequired = samplesRequired;
  _timeoutMs = timeoutMs;
}

int Comparator::getMaxVal() {
  return maxVal;
}

int Comparator::getMinVal() {
  return minVal;
}

void Comparator::setValues(int val) {
  if (val > maxVal) {
    maxVal = val;
    return;
  }
  if (val < minVal) {
    minVal = val;
  }
}

void Comparator::resetValues() {
  maxVal = 0;
  minVal = 6000;
}

ZeroCrossType Comparator::read() {

  int prevReading = analogRead(_analogPin);
  int prevState = (prevReading > _threshold) ? 1 : 0;
  Serial.print("PrevRead:"); Serial.println(prevReading);
  unsigned long startTime = millis();
  
  while (millis() - startTime < _timeoutMs) {
    int consistentState = 0;
    int stableCount = 0;

    for (int i = 0; i < _samplesRequired; i++) {
      int val = analogRead(_analogPin);
      Serial.print("CurrentRead["); Serial.print(i); Serial.print("]="); Serial.println(val);
      setValues(val);
      int currentState = (val > _threshold) ? 1 : 0;

      if (currentState == consistentState) {
        stableCount++;
      } else {
        consistentState = currentState;
        stableCount = 1;
      }

      delayMicroseconds(100);  // Small delay between samples
    }

    if (stableCount >= _samplesRequired) {
      if (prevState == 0 && consistentState == 1) {
        return ZC_RISING;
      } else if (prevState == 1 && consistentState == 0) {
        return ZC_FALLING;
      }
      prevState = consistentState;
    }
    return NO_CROSS;
  }
}