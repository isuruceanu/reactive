#include <Arduino.h>
#include "comparator.h"

Comparator* zeroCrossDetector;

const int zeroCrossPin = 35;
const int outputPin = 27;
const int ledPin       = 2;
const int threshold = 0;  

bool ledState = LOW;
volatile unsigned long lastInterruptTime = 0; // For debounce

unsigned long timeoutMs = 20;
int debounceSamples = 2;

bool outputState = true;

const unsigned long interval = 500; // Blink every 500 ms


void setup() {
  pinMode(zeroCrossPin, INPUT_PULLDOWN);
  pinMode(outputPin, OUTPUT);
  digitalWrite(outputPin, HIGH);  // Start LOW

  pinMode(ledPin, OUTPUT);
  digitalWrite(ledPin, LOW);
  
  zeroCrossDetector = new Comparator(zeroCrossPin, threshold, timeoutMs);

  Serial.begin(115200);
  Serial.println("Zero-cross test started.");
   
}

ZeroCrossType detectZeroCrossD1(int _analogPin, int _threshold = 10,unsigned long timeoutMs=20, int _samplesRequired =2) {
  
  int prevState = (analogRead(_analogPin) > _threshold) ? 1 : 0;
  unsigned long startTime = millis();

  while (millis() - startTime < timeoutMs) {
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

      delayMicroseconds(10);  // Small delay between samples
    }

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


void loop() {
  
  // unsigned long currentMillis = millis();

  // if (currentMillis - previousMillis >= interval) {
  //   previousMillis = currentMillis;
  //   ledState = !ledState;
  //   digitalWrite(ledPin, ledState);
    
  // }
  //zeroCrossDetector->loop();
  
  //ZeroCrossType zc = zeroCrossDetector-> detectZeroCross();
  ZeroCrossType zc = detectZeroCrossD1(zeroCrossPin, threshold, timeoutMs);
  digitalWrite(outputPin, zc == ZC_RISING);
}
