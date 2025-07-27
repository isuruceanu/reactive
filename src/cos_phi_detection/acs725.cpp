#include "acs725.h"
#include "const.h"

Acs725::Acs725(uint8_t analogPin, int samplesRequired, float frequencyHz)
  : Comparator(analogPin, samplesRequired, frequencyHz) {
}

void Acs725::setup(void) {
  
  int calibratedThreshold = calibrateZeroOffset();
  setThreshold(calibratedThreshold);
}


IRAM_ATTR float Acs725::readRMS() {
    const int maxSamples = 1000;
    float samples[maxSamples];
    int sampleCount = 0;

    float periodMicros = 1e6f / _freq;

    
    unsigned long startMicros = micros();
    while ((micros() - startMicros) < periodMicros && sampleCount < maxSamples) {
      int centered = analogRead(_analogPin) - _threshold; // Read adc value and substrract the _threshold 
      samples[sampleCount++] = centered * ADC_VREF / ADC_RES; //convert to volts 
    }

    // Compute RMS
    float sumSq = 0.0f;
    for (int i = 0; i < sampleCount; i++) {
      sumSq += samples[i] * samples[i];
    }
    
    float meanSq = sumSq / sampleCount;
    float rmsVoltage = sqrt(meanSq);
    float rmsCurrent = (rmsVoltage * 1000.0f) / ACS_MV_PER_AMP;

    return rmsCurrent;
  }