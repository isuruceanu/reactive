#ifndef PHASE_COMPENSATION_H
#define PHASE_COMPENSATION_H

#include <Arduino.h>
#include "shift_register.h"

class PhaseCompensation{
  public:



  private:
    ShiftRegister *_shift_register;
    uint8_t _start_shift;
    uint8_t _end_shift;
    uint8_t _next_on_idx;
    uint8_t _next_off_idx;
    

};


#endif
