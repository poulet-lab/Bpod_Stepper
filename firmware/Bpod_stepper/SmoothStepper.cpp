// SmoothStepper.cpp
//
// Copyright (C) 2020 Florian Rau

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

void SmoothStepper::setAcceleration(float a) {
  if (a <= 0.0)                               // limit acceleration to positive values
    return;
  _a  = a;                                    // Set private variable
  _c0 = 676000.0 * sqrt(2.0 / a);             // Estimate the first interval (eq. 15)
}

void SmoothStepper::setMaxSpeed(float vMax) {
  if (vMax <= 0.0)                            // limit vMax to positive values
    return;
  _vMax = vMax;                               // set private variable
}

void SmoothStepper::setMaxRPM(float maxRPM) {
  if (maxRPM <= 0.0)                          // limit maxRPM
    return;
  _vMax = maxRPM/60.0 * (float)_stepsPerRev;  // set private variable
}

void SmoothStepper::setStepsPerRev(unsigned int stepsPerRev) {
  _stepsPerRev = stepsPerRev;
}

void SmoothStepper::setPulseWidth(unsigned int pulseWidth) {
  _pulseWidth = pulseWidth;
}

void SmoothStepper::moveSteps(long nSteps) {
  float n2;
  float n3;
  float ci;
  float m;

  if (nSteps < 0) {
    nSteps = nSteps * -1;
    digitalWrite(_pinDirection, HIGH);
  }
  else
    digitalWrite(_pinDirection, LOW);

  if (nSteps == 0) {                                          // nothing to do for nSteps == 0
    return;
  }

  if (nSteps == 1) {                                          // a single step doesn't require fancy formulas
    step();
    return;
  }

  // calculate transition points ("linear-factor method")
  m  = (float) nSteps;
  n2 = round(_vMax * _vMax / (0.736 * _a));                   // eq24
  n2 = floor(min(n2, m / 2.0));                               // limit n2 to m/2
  n3 = nSteps - n2;                                           // n3 is symmetric to n2

  // run the step sequence
  for (long i = 1; i <= nSteps-1; i++) {
    if (i == 1)
      ci = _c0;                                               // first interval
    else if (i < n2)
      ci = ci - 2.0*ci/(4.0*(i-1)+1.0) * (n2-i+1.0)/n2;       // acceleration (eq22)
    else if (i < n3)
      ci = ci;                                                // top speed
    else
      ci = ci - 2.0*ci/(4.0*(i-m)+1.0) * (i-n3)/(m-n3-1.0);   // deceleration (eq25)
    step();                                                   // step once
    delayMicroseconds(ci - _pulseWidth);                      // delay for ci microseconds
  }
  step();                                                     // final step
}

void SmoothStepper::enableDriver() {
  digitalWrite(_pinEnable, LOW);
}

void SmoothStepper::disableDriver() {
  digitalWrite(_pinEnable, HIGH);
}

void SmoothStepper::step() {
  digitalWrite(_pinStep, HIGH);
  delayMicroseconds(_pulseWidth);
  digitalWrite(_pinStep, LOW);
}
