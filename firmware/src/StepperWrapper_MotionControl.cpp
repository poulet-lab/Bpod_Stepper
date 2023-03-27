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
#include <TMCStepper.h>
#include "StepperWrapper.h"
#include "SerialDebug.h"

StepperWrapper_MotionControl::StepperWrapper_MotionControl() : StepperWrapper() {}

void StepperWrapper_MotionControl::init() {
  StepperWrapper::init();
  _driver = get5160();

  //if (PCBrev < 1.4)
  //throwError(42);
}

float StepperWrapper_MotionControl::a() {
  DEBUG_PRINTFUN();
  return _driver->AMAX() / factA / _microsteps;
}

void StepperWrapper_MotionControl::a(float aHzs) {
  DEBUG_PRINTFUN(aHzs);
  //a5160 = aHzs / fCLK^2 / (512*256) / 2^24
  uint16_t a5160 = round(constrain(aHzs * factA * _microsteps, 0, uint16_t(-1)));
  _driver->AMAX(a5160);
  _driver->DMAX(a5160);
  _driver->a1(a5160);
  _driver->d1(a5160);
}

void StepperWrapper_MotionControl::hardStop() {
  DEBUG_PRINTFUN();
  // TODO
}

void StepperWrapper_MotionControl::moveMicroSteps(int32_t steps) {
  DEBUG_PRINTFUN(steps);
  _driver->XTARGET(_driver->XACTUAL() + steps);
}

int32_t StepperWrapper_MotionControl::microPosition() {
  DEBUG_PRINTFUN();
  return _driver->XACTUAL();
}

void StepperWrapper_MotionControl::microPosition(int32_t target) {
  DEBUG_PRINTFUN(target);
  _driver->RAMPMODE(0);
  _driver->XTARGET(target);
}

void StepperWrapper_MotionControl::setPosition(int32_t pos) {
  DEBUG_PRINTFUN();
  _driver->RAMPMODE(3);
  _driver->XACTUAL(pos);
}

void StepperWrapper_MotionControl::rotate(int8_t direction) {
  DEBUG_PRINTFUN();
  // TODO
}

void StepperWrapper_MotionControl::softStop() {
  DEBUG_PRINTFUN();
  // TODO
}

float StepperWrapper_MotionControl::vMax() {
  DEBUG_PRINTFUN();
  return _driver->VMAX() / factV / _microsteps;
}

void StepperWrapper_MotionControl::vMax(float vHz) {
  DEBUG_PRINTFUN(vHz);
  // v5160 = vHz / ( fCLK/2 / 2^23 )
  uint32_t v5160 = round(constrain(vHz * factV * _microsteps, 0, pow(2,23)-512));
  _driver->VMAX(v5160);
  _driver->v1(0); // Disables A1 and D1 phase, use AMAX, DMAX only
}

bool StepperWrapper_MotionControl::isRunning() {
  return _driver->stst();
}
