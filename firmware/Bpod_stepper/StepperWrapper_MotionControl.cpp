/*
StepperWrapper
Copyright (C) 2021 Florian Rau

This program is free software: you can redistribute it and/or modify it under
the terms of the GNU General Public License as published by the Free Software
Foundation, version 3.

This program is distributed  WITHOUT ANY WARRANTY and without even the implied
warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU
General Public License for more details.

You should have received a copy of the GNU General Public License along with
this program.  If not, see <http://www.gnu.org/licenses/>.


_______________________________________________________________________________
*/


#include <Arduino.h>
#include "StepperWrapper.h"
#include <TMCStepper.h>

StepperWrapper_MotionControl::StepperWrapper_MotionControl() : StepperWrapper() {}

void StepperWrapper_MotionControl::init(uint16_t rms_current) {
  StepperWrapper::init(rms_current);

  if (_PCBrev < 1.4)
    throwError(42);
}

void StepperWrapper_MotionControl::setAcceleration(float aHzs) {
  //a5160 = aHzs / fCLK^2 / (512*256) / 2^24
  uint16_t a5160 = round(aHzs * factA * _microsteps);
  _driver->AMAX(a5160);
  _driver->DMAX(a5160);
  _driver->a1(a5160);
  _driver->d1(a5160);
}

void StepperWrapper_MotionControl::setSpeed(float vHz) {
  // v5160 = vHz / ( fCLK/2 / 2^23 )
  uint32_t v5160 = round(vHz * factV * _microsteps);
  _driver->VMAX(v5160);
  _driver->v1(v5160);
}

void StepperWrapper_MotionControl::setTarget(int32_t target) {
  _driver->XTARGET(target * _microsteps);
}

int32_t StepperWrapper_MotionControl::getPosition() {
 return _driver->XACTUAL() / (int32_t) _microsteps;
}

void StepperWrapper_MotionControl::resetPosition() {
 _driver->XACTUAL(0);
}