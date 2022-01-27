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
extern StepperWrapper* wrapper;

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
  this->writePosition(_motor->getPosition());
}

void StepperWrapper_TeensyStep::moveSteps(int32_t steps) {
  DEBUG_PRINTFUN(steps);
  if (this->isRunning() || steps==0)            // return if moving or at target already
    return;
  bool dir = (steps>0) ? 1 : -1;
  if (this->atLimit(dir))                       // return if we are at a limit switch
    return;
  toggleISRlimit(dir);                          // attach ISRs for limit switches
  _motor->setMaxSpeed(_vMax);                   // set speed
  _motor->setTargetRel(steps * _microsteps);    // set relative target
  _stepControl->moveAsync(*_motor);             // start moving
  Serial1COM.writeByte(2);                      // send serial message 2: "Start"
  digitalWriteFast(LED_BUILTIN, HIGH);          // enable built-in LED
}

int32_t StepperWrapper_TeensyStep::position() {
  DEBUG_PRINTFUN();
  return _motor->getPosition() / (int32_t) _microsteps;
}

void StepperWrapper_TeensyStep::position(int32_t target) {
  DEBUG_PRINTFUN(target);
  this->moveSteps(target - this->position());   // convert to relative target
}

void StepperWrapper_TeensyStep::setPosition(int32_t pos) {
  DEBUG_PRINTFUN(pos);
  if (this->isRunning())
    return;
  _motor->setPosition(pos);
  this->writePosition(_motor->getPosition());
}

void StepperWrapper_TeensyStep::rotate(int8_t dir) {
  DEBUG_PRINTFUN(dir);
  if (this->isRunning() || this->atLimit(dir))  // return if moving or at limit switch
    return;
  toggleISRlimit(dir);                          // attach ISRs for limit switches
  _motor->setMaxSpeed((dir>=0)?_vMax:-_vMax);   // set direction of rotation (via sign of _vMax)
  _rotateControl->rotateAsync(*_motor);         // start moving
  Serial1COM.writeByte(2);                      // send serial message 2: "Start"
  digitalWriteFast(LED_BUILTIN, HIGH);          // enable built-in LED
}

void StepperWrapper_TeensyStep::softStop() {
  DEBUG_PRINTFUN();
  if (_stepControl->isRunning())
    _stepControl->stopAsync();
  else if (_rotateControl->isRunning()) {
    _rotateControl->stopAsync();
    this->CBstop(); // incorrect timing due to manual call
  }
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
  float msMax = (vDriver>0) ? 256.0 : 16.0;     // maximum micro-stepping resolution
  float vMaxMax = 100000;                       // maximum pulse-rate (TODO: verify!)
  float vMaxMin = 1.0 / msMax;                  // minimum pulse-rate
  vMax = constrain(vMax, vMaxMin, vMaxMax);     // sanitized pulse-rate

  // always use the highest possible micro-stepping resolution for a given vMax --- TODO: Doesn't work properly
  //uint8_t exp = 8 - ceil( log(ceil(vMax*msMax/vMaxMax))/log(2.0) );
  //this->setMicrosteps(pow(2,exp));
  this->setMicrosteps(0);

  // TODO: correct macro position on change of microsteps

  // set vMax
  _vMax = round(vMax * _microsteps);
}

void StepperWrapper_TeensyStep::CBstop() {
  DEBUG_PRINTFUN();
  Serial1COM.writeByte(3);
  digitalWriteFast(LED_BUILTIN, LOW);
  StepperWrapper_TeensyStep* w = (StepperWrapper_TeensyStep*) wrapper;
  w->writePosition(w->_motor->getPosition());
}

bool StepperWrapper_TeensyStep::isRunning() {
  return _stepControl->isRunning() || _rotateControl->isRunning();
}
