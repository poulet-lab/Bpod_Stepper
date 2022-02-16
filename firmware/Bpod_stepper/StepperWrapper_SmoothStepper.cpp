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

StepperWrapper_SmoothStepper::StepperWrapper_SmoothStepper() : StepperWrapper() {}

void StepperWrapper_SmoothStepper::init(uint16_t rms_current) {
  DEBUG_PRINTFUN();
  StepperWrapper::init(rms_current);
  _stepper = new SmoothStepper(pin.Step, pin.Dir);
  _stepper->setStepsPerRev(200 * (uint32_t) _microsteps);
  _stepper->setPinEnable(pin.En);             // We do want to use the enable pin
  _stepper->setInvertEnable(_invertPinEn);    // Enable pin on TMC2100 is inverted
  _stepper->setInvertDirection(_invertPinDir);// Invert the direction pin?
}

void StepperWrapper_SmoothStepper::setMicrosteps(uint16_t ms) {
  DEBUG_PRINTFUN(ms);
  StepperWrapper::setMicrosteps(ms);
}

float StepperWrapper_SmoothStepper::a() {
  DEBUG_PRINTFUN();
  return round(_stepper->getAcceleration() / _microsteps);
}

void StepperWrapper_SmoothStepper::a(float aHzs) {
  DEBUG_PRINTFUN(aHzs);
  _stepper->setAcceleration(aHzs * (float) _microsteps);
}

void StepperWrapper_SmoothStepper::hardStop() {
  DEBUG_PRINTFUN();
  // TODO
}

void StepperWrapper_SmoothStepper::moveSteps(int32_t steps) {
  DEBUG_PRINTFUN(steps);
  digitalWriteFast(LED_BUILTIN, HIGH);
  _stepper->moveSteps(steps * _microsteps);
  digitalWriteFast(LED_BUILTIN, LOW);
}

int32_t StepperWrapper_SmoothStepper::position() {
  DEBUG_PRINTFUN();
  return round(_stepper->getPosition() / (int32_t) _microsteps);
}

void StepperWrapper_SmoothStepper::position(int32_t target) {
  DEBUG_PRINTFUN(target);
  digitalWriteFast(LED_BUILTIN, HIGH);
  _stepper->movePosition(target * _microsteps);
  digitalWriteFast(LED_BUILTIN, LOW);
}

void StepperWrapper_SmoothStepper::setPosition(int32_t pos) {
  DEBUG_PRINTFUN();
  _stepper->setPosition(pos);
}

void StepperWrapper_SmoothStepper::rotate(int8_t direction) {
  DEBUG_PRINTFUN();
  // TODO
}

void StepperWrapper_SmoothStepper::softStop() {
  DEBUG_PRINTFUN();
  // TODO
}

float StepperWrapper_SmoothStepper::vMax() {
  DEBUG_PRINTFUN();
  return round(_stepper->getMaxSpeed() / _microsteps);
}

void StepperWrapper_SmoothStepper::vMax(float v) {
  DEBUG_PRINTFUN(v);
  _stepper->setMaxSpeed(v * (float) _microsteps);
}

bool StepperWrapper_SmoothStepper::isRunning() {
  return _stepper->isRunning();
}
