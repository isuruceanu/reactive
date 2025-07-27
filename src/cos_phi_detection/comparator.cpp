#include "comparator.h"
#include "const.h"

Comparator::Comparator(uint8_t analogPin, int threshold, int samplesRequired, float frequencyHz) {
  _analogPin = analogPin;
  _samplesRequired = samplesRequired;
  _freq = frequencyHz;
  _threshold = threshold;

  pinMode(_analogPin, INPUT_PULLDOWN);
}

void Comparator::setThreshold(int threshold) {
  _threshold = threshold;
}

IRAM_ATTR ZeroCrossType Comparator::detect(unsigned long timeoutMs) {
  
  int prevState = (analogRead(_analogPin) > _threshold) ? 1 : 0;
  unsigned long startTime = millis();
  while (millis() - startTime < timeoutMs) {
    int consistentState = 0;
    int stableCount = 0;
    portENTER_CRITICAL(&_mux);
    for (int i = 0; i < _samplesRequired; i++) {
      int val = analogRead(_analogPin);
      int currentState = (val > _threshold) ? 1 : 0;

      if (currentState == consistentState) {
        stableCount++;
      } else {
        consistentState = currentState;
        stableCount = 1;
      }

     delayMicroseconds(10);  // Small delay between samples
    }
    portEXIT_CRITICAL(&_mux);

    if (stableCount >= _samplesRequired) {
      if (prevState == 0 && consistentState == 1) {
        return ZC_RISING;
      } else if (prevState == 1 && consistentState == 0) {
        return ZC_FALLING;
      }
      prevState = consistentState;
    }
  }

  return NO_CROSS;
}

int Comparator::calibrateZeroOffset(int samples) {
  long sum = 0;

  for (int i = 0; i < samples; i++) {
    sum += analogRead(_analogPin);
    delayMicroseconds(50);  // 20kHz sampling
  }

  float avgADC = sum / (float)samples;
  return (int)avgADC;
}



// void Comparator::loop(bool stopOnFire) {

//   if (!_running) return;
//   //int prevReading = analogRead(_analogPin);
//   //_prevState = (prevReading > _threshold) ? 1 : 0;

//   int consistentState = 0;
//   int stableCount = 0;

//   for (int i = 0; i < _samplesRequired; i++) {
//     int val = analogRead(_analogPin);
//     Serial.print("Val["); Serial.print(i); Serial.print("]="); Serial.println(val);
//     int currentState = (val > _threshold) ? 1 : 0;

//     if (currentState == consistentState) {
//       stableCount++;
//     } else {
//       consistentState = currentState;
//       stableCount = 1;
//     }

//     delayMicroseconds(10);  // Small delay between samples
//   }

//   if (stableCount >= _samplesRequired) {
//     if (_prevState == 0 && consistentState == 1) {
//       if (stopOnFire) stop();
//       _callback(ZC_RISING);
//       return;
//     } else if (_prevState == 1 && consistentState == 0) {
//       if (stopOnFire) stop();
//       _callback(ZC_FALLING);
//       return;
//     }
//     _prevState = consistentState;
//   }
// }