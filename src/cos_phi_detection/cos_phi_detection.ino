#include <Arduino.h>
#include "comparator.h"
// #include <Wire.h>
// #include <LiquidCrystal_I2C.h>


// LiquidCrystal_I2C lcd(0x27, 16, 2);

const int volt0CrossPin = 35;
const int acs725Pin = 14;
const int outputPin = 27;
const int ledPin       = 2;
const int voltageThreshold = 0;  
const int currentThreshold = 2048;  

bool ledState = LOW;

unsigned long timeoutMs = 20;
int debounceSamples = 2;

bool outputState = true;

const unsigned long interval = 500; // Blink every 500 ms

Comparator voltageZC(volt0CrossPin, voltageThreshold, debounceSamples); 
Comparator currentZC(acs725Pin, currentThreshold, 1); 


void setup() {
  pinMode(volt0CrossPin, INPUT_PULLDOWN);
  pinMode(acs725Pin, INPUT_PULLDOWN);
  pinMode(outputPin, OUTPUT);
  digitalWrite(outputPin, HIGH);  // Start LOW

  pinMode(ledPin, OUTPUT);
  digitalWrite(ledPin, LOW);
  
  Serial.begin(115200);
  Serial.println("Zero-cross test started.");
  // lcd.init();         // Initialize LCD
  // lcd.backlight();    // Turn on backlight
  // lcd.setCursor(0, 0);
  // lcd.print("Hello ESP32!");
   
}

ZeroCrossType detectZeroCross(int _analogPin, int _threshold, unsigned long timeoutMs=20, int _samplesRequired =2) {
  
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


float calculatePowerFactor(unsigned long voltageZC, unsigned long currentZC, float frequencyHz) {
  const float periodSec = 1.0 / frequencyHz;

  long deltaT_us = (long)(currentZC - voltageZC);  // time difference in microseconds (can be negative)
  float deltaT_sec = deltaT_us / 1e6;              // convert to seconds

  float phaseAngleRad = 2 * PI * frequencyHz * deltaT_sec;

  // Optional: reject if phase shift exceeds ±90 degrees (π/2 radians)
  if (fabs(phaseAngleRad) > PI / 2) return -1.0; // invalid

  float pf = cos(phaseAngleRad);
  return fabs(pf); // PF is always a positive magnitude
}

float readCurrentRMS(int analogPin, float zeroCurrentVoltage = 1.65, float sensitivity_mV_per_A = 264.0, int sampleCount = 1000) {
  const float ADC_REF_VOLTAGE = 3.3;    // ESP32 ADC reference
  const int ADC_RESOLUTION = 4095;      // 12-bit ADC resolution

  float sumSquared = 0;
 // portENTER_CRITICAL(&_mux);
  for (int i = 0; i < sampleCount; i++) {
    int adcValue = analogRead(analogPin);
    float voltage = (adcValue * ADC_REF_VOLTAGE) / ADC_RESOLUTION;

    // Subtract zero-current voltage offset (~1.65V at 3.3V supply)
    float centeredVoltage = voltage - zeroCurrentVoltage;
    sumSquared += centeredVoltage * centeredVoltage;

    delayMicroseconds(100);  // ~10kHz sampling rate
  }
 // portEXIT_CRITICAL(&_mux);

  float meanSquared = sumSquared / sampleCount;
  float rmsVoltage = sqrt(meanSquared);

  // Convert to Amps using sensitivity in mV/A
  float currentRMS = (rmsVoltage * 1000.0) / sensitivity_mV_per_A;
  return currentRMS;
}


void loop() {
  
  // unsigned long currentMillis = millis();

  // if (currentMillis - previousMillis >= interval) {
  //   previousMillis = currentMillis;
  //   ledState = !ledState;
  //   digitalWrite(ledPin, ledState);
    
  // }
  
  
  
  //ZeroCrossType zc = detectZeroCrossD1(zeroCrossPin, threshold, timeoutMs);
  //ZeroCrossType zc = voltageZC.detect(timeoutMs);

  digitalWrite(outputPin, zc == ZC_RISING);
  // float current = readCurrentRMS(acs725Pin);
  // Serial.print("Current: "); Serial.println(current);

  // delay(1000);
}
