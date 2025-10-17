#include <Arduino.h>
#include "const.h"
#include "comparator.h"
#include "acs725.h"
#include "shift_register.h"
#include "circular_shift.h"

#include <LiquidCrystal_I2C.h>


LiquidCrystal_I2C lcd(0x27, 20, 4);
ShiftRegister ledBar(DATA_PIN, CLOCK_PIN, LATCH_PIN, LED_COUNT);
SwitchOnOffStrategy *circularStrategy;

unsigned long timeoutMs = 20;
int debounceSamples = 2;

Comparator voltageZC(VOLTAGE_ZC_PIN, VOLT_THRESHOLD, debounceSamples); //TODO: revert back to 0 on real data
Acs725 acs725(ACS725_PIN);


//Simulation, sine generation params
#define TABLE_SIZE 256     // sine table samples
#define SAMPLE_RATE (FREQ * TABLE_SIZE)   // Hz

#define SAMPLE_INTERVAL_US (1000000.0f / (SAMPLE_RATE))
#define INTERVAL_PERIOD ((FREQ * 360) / 1e6)

esp_timer_handle_t waveform_timer;
portMUX_TYPE timerMux = portMUX_INITIALIZER_UNLOCKED;

volatile int idx = 0;
volatile float phaseShift = 0.0f;  // radians
volatile int phaseOffset = 0;

uint8_t sineTable[TABLE_SIZE];

void IRAM_ATTR onTimer(void* arg) {
  portENTER_CRITICAL_ISR(&timerMux);

  // Compute indices with phase shift
  int idxA = idx;
  int idxB = (idx + phaseOffset + TABLE_SIZE) % TABLE_SIZE;

  // Output DAC values
  dacWrite(DAC_PIN_A, sineTable[idxA]);
  dacWrite(DAC_PIN_B, sineTable[idxB]);

  // Increment index
  idx = (idx + 1) % TABLE_SIZE;

  portEXIT_CRITICAL_ISR(&timerMux);
}


void setup() {
  
 
  // Build sine lookup table (0–255)
  for (int i = 0; i < TABLE_SIZE; i++) {
    sineTable[i] = (uint8_t)((sin((TWO_PI * i) / TABLE_SIZE) + 1.0) * 127.5);
  }


  pinMode(POT_PIN, INPUT);
  pinMode(DAC_PIN_A, OUTPUT);
  pinMode(DAC_PIN_B, OUTPUT);

  // Set up a hardware timer to generate the waveform
  esp_timer_create_args_t timer_args = {
    .callback = &onTimer,
    .name = "waveform_timer"
  };
  esp_timer_create(&timer_args, &waveform_timer);
  
  // Start the timer to trigger the DAC updates
  esp_timer_start_periodic(waveform_timer, (uint64_t)SAMPLE_INTERVAL_US);


  Serial.println("Wave generator using hardware timer started.");
  
  circularStrategy = new CircularShift(8);
  pinMode(OUTPUT_PIN, OUTPUT);
  digitalWrite(OUTPUT_PIN, LOW);  // Start LOW

  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, LOW);
  //analogSetAttenuation(ADC_11db);

  Serial.begin(115200);
  Serial.println("Zero-cross test started.");

  acs725.setup();
 // ledBar.begin();

  // lcd.init();  // initialize the lcd
  // lcd.backlight();

  // lcd.setCursor(0, 0);             // move cursor the first row
  // lcd.print("LCD 20x4");           // print message at the first row
  // lcd.setCursor(0, 1);             // move cursor to the second row
  // lcd.print("I2C Address: 0x27");  // print message at the second row
  // lcd.setCursor(0, 2);             // move cursor to the third row
  // lcd.print("DIYables");           // print message at the third row
  // delay(5000);
  // lcd.clear();


}


// check_tolerance - check if delta t and current is big/small enough to switch on/off capacitors
bool check_tolerance(float current, unsigned long dt) {
  //TODO: implement this
  return true;
}

float getPhaseShift() {
  ZeroCrossType volt_zc = voltageZC.detect();
  if (volt_zc == ZC_RISING) { // detect voltage rising zero cross
    unsigned long volt_time = micros();
    ZeroCrossType current_zc = acs725.detect(); // detect current cross
    unsigned long current_time = micros(); // read current time (when current cross detected)
    auto dt = current_time - volt_time; // calculate time difference between current and volt corss detection
    float phaseAngle = dt * INTERVAL_PERIOD;
    if (current_zc == ZC_FALLING) {
      phaseAngle = phaseAngle - 180.0;
    }
    return phaseAngle;
  } 

  return 0.0; // TODO: check this, why 0(zero) maybe return a stuct with error and value?
}

void loop() {
  static unsigned long lastPrint = 0;
  // read current in A, we need it anyway to display

  int potVal = analogRead(POT_PIN);
  phaseShift = map(potVal, 0, 4095, -90, 90) * PI / 180.0f;

  portENTER_CRITICAL(&timerMux);
  phaseOffset = (int)((phaseShift / (2.0 * PI)) * TABLE_SIZE);
  portEXIT_CRITICAL(&timerMux);
 
  // run  - calculate the delta t between voltage and current 
  float realAngle = getPhaseShift();

  float current = 0.0;
  current = acs725.readRMS();

  if (millis() - lastPrint > 2000 && realAngle != 0.0) {
    float degrees = phaseShift * 180.0f / PI;
    float powerFactor = cos(phaseShift);


    float rPF = cos(realAngle * PI/180.0);
    Serial.printf("Phase: %.2f°, PF: %.2f \n", degrees, powerFactor);
    Serial.printf("Real degree: %.2f°, PF: %.2f, AMPs: %.2f(A) \n", realAngle, rPF, current);
    lastPrint = millis();
  }



  // lcd.setCursor(0, 0);  // Line 2, position 1
  // lcd.print("RMS Current: ");
  // lcd.print(current, 2);  // Print with 2 decimals
  // lcd.print(" A");

  
  delay(1000);

}
