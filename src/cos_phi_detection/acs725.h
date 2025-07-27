#ifndef ACS725_H
#define ACS725_H

#include <Arduino.h>
#include "comparator.h"
#include "const.h"

class Acs725 : public Comparator {
  public:
    Acs725(uint8_t analogPin, int samplesRequired = 2, float frequesncyHz = FREQ);
    void setup(void);
    IRAM_ATTR float readRMS( ); 

  private: 
    portMUX_TYPE _mux = portMUX_INITIALIZER_UNLOCKED;
};



#endif