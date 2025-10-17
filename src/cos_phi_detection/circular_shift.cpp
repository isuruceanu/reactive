#include "circular_shift.h"


//SwitchOnOffStrategy::SwitchOnOffStrategy( ) {};

CircularShift::CircularShift(int size) : _size(size) {};


void CircularShift::reset(void) {
  _idxOn = -1;
  _idxOff = -1;
}

int CircularShift::on()  {
  _idxOn = (_idxOn + 1) % _size;
  return _idxOn;
}

int CircularShift::off()  {
  _idxOff = (_idxOff + 1) % _size;
  return _idxOff;
}