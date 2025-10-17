#include "shift_register.h"
#include "circular_shift.h"

ShiftRegister::ShiftRegister(uint8_t dataPin, uint8_t clockPin, uint8_t latchPin, uint8_t ledCount)
  : _dataPin(dataPin), _clockPin(clockPin), _latchPin(latchPin), _ledCount(ledCount), _currentIndex(-1), _ledState(0) {}

void ShiftRegister::begin() {
  pinMode(_dataPin, OUTPUT);
  pinMode(_clockPin, OUTPUT);
  pinMode(_latchPin, OUTPUT);
  clearAll();
}

void ShiftRegister::next(bool turnOffPrevious) {
  if (_currentIndex < _ledCount - 1) {
    if (turnOffPrevious && _currentIndex >= 0) {
      _ledState &= ~(1UL << _currentIndex);
    }
    _currentIndex++;
    _ledState |= (1UL << _currentIndex);
    updateShiftRegister();
  }
}

void ShiftRegister::previous(bool turnOffCurrent) {
  if (_currentIndex > 0) {
    if (turnOffCurrent) {
      _ledState &= ~(1UL << _currentIndex);
    }
    _currentIndex--;
    _ledState |= (1UL << _currentIndex);
    updateShiftRegister();
  }
}

void ShiftRegister::clearAll() {
  _ledState = 0;
  _currentIndex = -1;
  updateShiftRegister();
}

void ShiftRegister::on(SwitchOnOffStrategy *st) {
  int idx = st->on();
  set(idx, true);
}

void ShiftRegister::off(SwitchOnOffStrategy *st) {
  int idx = st->off();
  set(idx, false);
}


void ShiftRegister::set(uint8_t index, bool state) {
  if (index >= _ledCount) return;
  if (state) {
    _ledState |= (1UL << index);
  } else {
    _ledState &= ~(1UL << index);
  }
  updateShiftRegister();
}

void ShiftRegister::updateShiftRegister() {
  digitalWrite(_latchPin, LOW);
  for (int i = (_ledCount - 1) / 8; i >= 0; i--) {
    shiftOut(_dataPin, _clockPin, MSBFIRST, (_ledState >> (i * 8)) & 0xFF);
  }
  digitalWrite(_latchPin, HIGH);
}