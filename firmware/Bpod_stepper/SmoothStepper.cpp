/*
SmoothStepper
Copyright (C) 2020 Florian Rau

This program is free software: you can redistribute it and/or modify
it under the terms of the GNU General Public License as published by
the Free Software Foundation, version 3.

This program is distributed  WITHOUT ANY WARRANTY and without even the
implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
See the GNU General Public License for more details.

You should have received a copy of the GNU General Public License
along with this program.  If not, see <http://www.gnu.org/licenses/>.
*/

#include "Arduino.h"
#include "SmoothStepper.h"

SmoothStepper::SmoothStepper(uint8_t pinStep, uint8_t pinDirection) {
  _pinStep = pinStep;
  _pinDirection = pinDirection;
  pinMode(_pinStep, OUTPUT);
  pinMode(_pinDirection, OUTPUT);
}

void SmoothStepper::setPinEnable(uint8_t pinEnable) {
  _pinEnable = pinEnable;
  pinMode(_pinEnable, OUTPUT);
}

void SmoothStepper::setInvertEnable(bool invertEnable) {
  _invertEnable = invertEnable;
}

void SmoothStepper::setInvertDirection(bool invertDirection) {
  _invertDirection = invertDirection;
}

void SmoothStepper::setStepsPerRev(uint32_t stepsPerRev) {
  _stepsPerRev = stepsPerRev;
}

void SmoothStepper::setAcceleration(float a) {
  if (a <= 0.0)                                               // limit acceleration to positive values
    return;
  _a  = a;
  _c0 = 676000.0 * sqrt(2.0 / a);                             // Estimate the first interval (eq. 15)
}

void SmoothStepper::setMaxSpeed(float vMax) {
  if (vMax <= 0.0)                                            // limit vMax to positive values
    return;
  _vMax = vMax;
}

void SmoothStepper::setPulseWidth(uint16_t pulseWidth) {
  _pulseWidth = pulseWidth;
}

void SmoothStepper::enableDriver() {
  digitalWrite(_pinEnable, HIGH ^ _invertEnable);
}

void SmoothStepper::disableDriver() {
  digitalWrite(_pinEnable, LOW  ^ _invertEnable);
}

bool SmoothStepper::getDirection() {
  return _direction;
}

float SmoothStepper::getMaxSpeed() {
  return _vMax;
}

float SmoothStepper::getAcceleration() {
  return _a;
}

void SmoothStepper::moveSteps(int32_t nSteps) {
  _direction = (nSteps >= 0);
  if (_direction)
    digitalWrite(_pinDirection, LOW  ^ _invertDirection);
  else {
    nSteps = nSteps * -1;
    digitalWrite(_pinDirection, HIGH ^ _invertDirection);
  }

  if (nSteps == 0)                                            // nothing to do for nSteps == 0
    return;

  _isRunning = true;
  step();                                                     // first step
  if (nSteps == 1) {                                          // a single step doesn't require fancy formulas
    _isRunning = false;
    return;
  }

  // calculate transition points ("linear-factor method")
  float m  = (float) nSteps;
  float n2 = round(_vMax * _vMax / (0.736 * _a));             // eq24
  n2 = floor(min(n2, m / 2.0));                               // limit n2 to m/2
  float n3 = m - n2;                                          // n3 is symmetric to n2
  float ci;

  // run the step sequence
  _stop = false;
  for (int32_t i = 1; i < nSteps; i++) {
    if (_stop)
      break;
    else if (i == 1)
      ci = _c0;                                               // first interval
    else if (i < n2)
      ci = ci - 2.0*ci/(4.0*(i-1)+1.0) * (n2-i+1.0)/n2;       // acceleration (eq22)
    else if (i < n3)
      ci = ci;                                                // top speed
    else
      ci = ci - 2.0*ci/(4.0*(i-m)+1.0) * (i-n3)/(m-n3-1.0);   // deceleration (eq25)
    delayMicroseconds(ci - _pulseWidth);                      // delay for ci microseconds
    step();
  }
  _isRunning = false;
}

void SmoothStepper::setPosition(int32_t position) {
  _position = position;
}

int32_t SmoothStepper::getPosition() {
  return _position;
}

void SmoothStepper::movePosition(int32_t target) {
  int32_t nSteps = target - _position;
  moveSteps(nSteps);
}

void SmoothStepper::moveDegrees(float degrees) {
  int32_t nSteps = round(degrees * _stepsPerRev / 360.0);
  moveSteps(nSteps);
}

void SmoothStepper::step() {
  digitalWrite(_pinStep, HIGH);
  delayMicroseconds(_pulseWidth);
  digitalWrite(_pinStep, LOW);

  if (_direction)
    _position++;
  else
    _position--;
}

void SmoothStepper::stop() {
  _stop = true;
}

bool SmoothStepper::isRunning() {
  return _isRunning;
}
