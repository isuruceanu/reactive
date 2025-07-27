#include <Arduino.h>
#include "const.h"
#include "comparator.h"
#include "acs725.h"

// #include <Wire.h>
// #include <LiquidCrystal_I2C.h>


// LiquidCrystal_I2C lcd(0x27, 16, 2);

const int outputPin = 27;
const int ledPin       = 2;


bool ledState = LOW;

unsigned long timeoutMs = 20;
int debounceSamples = 2;

bool outputState = true;

const unsigned long interval = 500; // Blink every 500 ms

Comparator voltageZC(VOLTAGE_ZC_PIN, 0, debounceSamples); 
Acs725 acs725(ACS725_PIN); 

void setup() {
  
  pinMode(outputPin, OUTPUT);
  digitalWrite(outputPin, HIGH);  // Start LOW

  pinMode(ledPin, OUTPUT);
  digitalWrite(ledPin, LOW);
  analogSetAttenuation(ADC_11db);
  Serial.begin(115200);
  Serial.println("Zero-cross test started.");
  

  // lcd.init();         // Initialize LCD
  // lcd.backlight();    // Turn on backlight
  // lcd.setCursor(0, 0);
  // lcd.print("Hello ESP32!");
   
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




void loop() {
  
  
  ZeroCrossType volt_zc = voltageZC.detect();
  unsigned long volt_time = millis();
  unsigned long current_time = volt_time;
  float current = 0.0;
  if (volt_zc == ZC_RISING) {
    ZeroCrossType current_zc = acs725.detect();
    current_time = millis();
    if (current_zc != NO_CROSS) {
      current = acs725.readRMS();
    }

  } else if (volt_zc == NO_CROSS) {
    Serial.println("NO CROSS");
  }

  Serial.print("Delta t="); Serial.println(volt_time - current_time );
  Serial.print("Current = "); Serial.println(current);

  delay(1000);
}
