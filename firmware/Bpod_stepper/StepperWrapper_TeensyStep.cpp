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

extern ArCOM Serial1COM;

StepperWrapper_TeensyStep::StepperWrapper_TeensyStep() : StepperWrapper() {
  _motor = new Stepper(pin.Step, pin.Dir);
  _motor->setInverseRotation(_invertPinDir);

  _stepControl = new StepControl;
  _stepControl->setCallback(CBstop);

  _rotateControl = new RotateControl;
}

void StepperWrapper_TeensyStep::init(uint16_t rms_current) {
  DEBUG_PRINTFUN();
  StepperWrapper::init(rms_current);
}

float StepperWrapper_TeensyStep::a() {
  DEBUG_PRINTFUN();
  return _a / (float) _microsteps;
}

void StepperWrapper_TeensyStep::a(float aHzs) {
  DEBUG_PRINTFUN(aHzs);
  if (this->isRunning())
    return;
  _a = round(aHzs * (float) _microsteps);
  _motor->setAcceleration(_a);
}

void StepperWrapper_TeensyStep::hardStop() {
  DEBUG_PRINTFUN();
  _stepControl->emergencyStop();
  _rotateControl->emergencyStop();
  Serial1COM.writeByte(4);
  digitalWriteFast(LED_BUILTIN, LOW);
}

void StepperWrapper_TeensyStep::moveSteps(int32_t steps) {
  DEBUG_PRINTFUN(steps);
  if (this->isRunning())
    return;
  _motor->setMaxSpeed(_vMax);
  _motor->setTargetRel(steps * _microsteps);
  _stepControl->moveAsync(*_motor);
  Serial1COM.writeByte(2);
  digitalWriteFast(LED_BUILTIN, HIGH);
}

int32_t StepperWrapper_TeensyStep::position() {
  DEBUG_PRINTFUN();
  return _motor->getPosition() / (int32_t) _microsteps;
}

void StepperWrapper_TeensyStep::position(int32_t target) {
  DEBUG_PRINTFUN(target);
  if (this->isRunning())
    return;
  _motor->setMaxSpeed(_vMax);
  _motor->setTargetAbs(target * _microsteps);
  _stepControl->moveAsync(*_motor);
  Serial1COM.writeByte(2);
  digitalWriteFast(LED_BUILTIN, HIGH);
}

void StepperWrapper_TeensyStep::resetPosition() {
  DEBUG_PRINTFUN();
  if (this->isRunning())
    return;
  _motor->setPosition(0);
}

void StepperWrapper_TeensyStep::rotate(int8_t direction) {
  DEBUG_PRINTFUN(direction);
  if (this->isRunning())
    return;
  if (direction>=0)
    _motor->setMaxSpeed(_vMax);
  else
    _motor->setMaxSpeed(-_vMax);
  _rotateControl->rotateAsync(*_motor);
  Serial1COM.writeByte(2);
  digitalWriteFast(LED_BUILTIN, HIGH);
}

void StepperWrapper_TeensyStep::softStop() {
  DEBUG_PRINTFUN();
  _stepControl->stopAsync();
  _rotateControl->stopAsync();
  Serial1COM.writeByte(3);
}

float StepperWrapper_TeensyStep::vMax() {
  DEBUG_PRINTFUN();
  return _vMax / (float) _microsteps;
}

void StepperWrapper_TeensyStep::vMax(float vMax) {
  DEBUG_PRINTFUN(vMax);
  if (this->isRunning())
    return;

  // constrain speed
  float msMax = (vDriver>0) ? 256.0 : 16.0; // maximum micro-stepping resolution
  float vMaxMax = 100000;                   // maximum pulse-rate (TODO: verify!)
  float vMaxMin = 1.0 / msMax;              // minimum pulse-rate
  vMax = constrain(vMax, vMaxMin, vMaxMax); // sanitized pulse-rate

  // always use the highest possible micro-stepping resolution for a given vMax
  uint8_t exp = 8 - ceil( log(ceil(vMax*msMax/vMaxMax))/log(2.0) );
  this->setMicrosteps(pow(2,exp));

  // TODO: correct macro position on change of microsteps

  // set vMax
  _vMax = round(vMax * _microsteps);
}

void StepperWrapper_TeensyStep::CBstop() {
  DEBUG_PRINTFUN();
  Serial1COM.writeByte(3);
  digitalWriteFast(LED_BUILTIN, LOW);
}

bool StepperWrapper_TeensyStep::isRunning() {
  return _stepControl->isRunning() || _rotateControl->isRunning();
}
