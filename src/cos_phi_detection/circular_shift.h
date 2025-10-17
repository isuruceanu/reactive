#ifndef CIRCULAR_SHIFT_H
#define CIRCULAR_SHIFT_H


#include <Arduino.h>

class SwitchOnOffStrategy {
  public:
    virtual int on(void) = 0; //abstract method to swithc on, returns the index 
    virtual int off(void) = 0; // abstract method to switch off, return the index of switched off 
};

class CircularShift : public SwitchOnOffStrategy {
  public:
    CircularShift( int size);
    void reset(void);
    int on(void) override;
    int off(void) override;

  private:
    int _size;
    int _idxOn = -1;
    int _idxOff = -1;
};


#endif