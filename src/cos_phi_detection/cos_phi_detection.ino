#include <Arduino.h>
#include "const.h"
#include "comparator.h"
#include "acs725.h"
#include "shift_register.h"

#include <LiquidCrystal_I2C.h>


LiquidCrystal_I2C lcd(0x27, 20, 4);
ShiftRegister ledBar(DATA_PIN, CLOCK_PIN, LATCH_PIN, LED_COUNT);


unsigned long timeoutMs = 20;
int debounceSamples = 2;
int idx = 0;
int direction = 1;


bool outputState = true;

const unsigned long interval = 500;  // Blink every 500 ms

Comparator voltageZC(VOLTAGE_ZC_PIN, 0, debounceSamples);
Acs725 acs725(ACS725_PIN);



void setup() {

  pinMode(OUTPUT_PIN, OUTPUT);
  digitalWrite(OUTPUT_PIN, HIGH);  // Start LOW

  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, LOW);
  analogSetAttenuation(ADC_11db);

  Serial.begin(115200);
  Serial.println("Zero-cross test started.");

  acs725.setup();
  ledBar.begin();

  lcd.init();  // initialize the lcd
  lcd.backlight();

  lcd.setCursor(0, 0);             // move cursor the first row
  lcd.print("LCD 20x4");           // print message at the first row
  lcd.setCursor(0, 1);             // move cursor to the second row
  lcd.print("I2C Address: 0x27");  // print message at the second row
  lcd.setCursor(0, 2);             // move cursor to the third row
  lcd.print("DIYables");           // print message at the third row
  delay(5000);
  lcd.clear();
}




float calculatePowerFactor(unsigned long voltageZC, unsigned long currentZC, float frequencyHz) {
  const float periodSec = 1.0 / frequencyHz;

  long deltaT_us = (long)(currentZC - voltageZC);  // time difference in microseconds (can be negative)
  float deltaT_sec = deltaT_us / 1e6;              // convert to seconds

  float phaseAngleRad = 2 * PI * frequencyHz * deltaT_sec;

  // Optional: reject if phase shift exceeds ±90 degrees (π/2 radians)
  if (fabs(phaseAngleRad) > PI / 2) return -1.0;  // invalid

  float pf = cos(phaseAngleRad);
  return fabs(pf);  // PF is always a positive magnitude
}


bool detectVoltRising() {
  while (true) {
    ZeroCrossType volt_zc = voltageZC.detect();
    switch (volt_zc) {
      case NO_CROSS:
        return false;
        break;
      case ZC_RISING:
        return true;
        break;
      default:
        delay(5);
        break;
    }
  }
}



void loop() {

  ledBar.set(idx, direction == 1);
  if (idx >= LED_COUNT) direction = -1;
  if (idx <= -1) direction = 1;

  idx += direction;


  unsigned long volt_time = millis();
  unsigned long current_time = volt_time;
  float current = 0.0;


  if (!detectVoltRising()) {
    Serial.println("NO CROSS");
  }


  Serial.println("Current detect");
  ZeroCrossType current_zc = acs725.detect();
  current_time = millis();
  Serial.print("Current read: "); Serial.println(current_zc);
  if (current_zc != NO_CROSS) {
    Serial.println("Read current");
    current = acs725.readRMS();
  }


  Serial.print("Delta t=");
  Serial.println(volt_time - current_time);
  Serial.print("Current = ");
  Serial.println(current);

  lcd.setCursor(0, 0);  // Line 2, position 1
  lcd.print("RMS Current: ");
  lcd.print(current, 2);  // Print with 2 decimals
  lcd.print(" A");


  delay(1000);
}
