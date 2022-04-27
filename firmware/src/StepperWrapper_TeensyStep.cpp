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


#include <TMCStepper.h>
#include "StepperWrapper.h"
#include "SerialDebug.h"

extern ArCOM Serial1COM;
extern StepperWrapper* wrapper;

StepperWrapper_TeensyStep::StepperWrapper_TeensyStep() : StepperWrapper() {
  _motor = new Stepper(pin.Step, pin.Dir);
  _motor->setInverseRotation(_invertPinDir);
  _motor->setPosition(0);
  _stepControl = new StepControl;
  _stepControl->setCallback(CBstop);
  _rotateControl = new RotateControl;
  _rotateControl->setCallback(CBstop);
}

void StepperWrapper_TeensyStep::init(uint16_t rms_current) {
  StepperWrapper::init(rms_current);
}

float StepperWrapper_TeensyStep::a() {
  return _a;
}

void StepperWrapper_TeensyStep::a(float aHzs) {
  if (isRunning() || _a == aHzs)
    return;
  _aMu = round(aHzs * _microsteps);
  _a   = (float) _aMu / _microsteps;
  DEBUG_PRINTF("Setting acceleration to %1.0f steps/s^2\n",aHzs);
}

void StepperWrapper_TeensyStep::hardStop() {
  noInterrupts();
  _stepControl->emergencyStop();
  _rotateControl->emergencyStop();
  Serial1COM.writeByte(4);
  interrupts();
  digitalWriteFast(LED_BUILTIN, LOW);
  updateMicroPosition();
  DEBUG_PRINTF("Emergency stop at position %d\n",_microPosition);
  writePosition(_microPosition);
}

void StepperWrapper_TeensyStep::moveSteps(int32_t steps) {
  moveMicroSteps(steps * _msRes);
}

void StepperWrapper_TeensyStep::moveMicroSteps(int32_t steps) {
  if (isRunning() || steps==0)                      // return if moving or at target already
    return;
  bool dir = (steps>0) ? 1 : -1;
  if (atLimit(dir))                                 // return if we are at a limit switch
    return;
  toggleISRlimit(dir);                              // attach ISRs for limit switches

  DEBUG_PRINTF("Moving %d 1/%d-steps ...\n",steps,_microsteps);
  _motor->setAcceleration(_aMu);                    // set acceleration
  _motor->setMaxSpeed(_vMaxMu);                     // set speed
  _motor->setTargetRel(steps);                      // set relative target
  _stepControl->moveAsync(*_motor);                 // start moving

  Serial1COM.writeByte(2);                          // send serial message 2: "Start"
  digitalWriteFast(LED_BUILTIN, HIGH);              // enable built-in LED
}

int32_t StepperWrapper_TeensyStep::position() {
  return _microPosition/_msRes + (_motor->getPosition()/_microsteps);
}

void StepperWrapper_TeensyStep::position(int32_t target) {
  DEBUG_PRINTF("Target position: %d micro-steps\n",target*_msRes);
  target = (target*_msRes - _microPosition) / (_msRes/_microsteps);
  moveMicroSteps(target);   // convert to relative target
}

void StepperWrapper_TeensyStep::setPosition(int32_t pos) {
  if (isRunning() || pos == _microPosition)
    return;
  DEBUG_PRINTF("Setting position to %d micro-steps\n",pos);
  noInterrupts();
  _microPosition = pos;
  interrupts();
  _motor->setPosition(0);
  writePosition(pos);
}

void StepperWrapper_TeensyStep::rotate(int8_t dir) {
  if (isRunning() || atLimit(dir))        // return if already moving or at limit switch
    return;
  dir = (dir>=0) ? 1 : -1;                            // sanitize input argument
  toggleISRlimit(dir);                                // attach ISRs for limit switches

  DEBUG_PRINTF("Starting rotation in %s direction ...\n",(dir>0) ? "positive" : "negative");
  _motor->setAcceleration(_aMu);                      // set acceleration
  _motor->setMaxSpeed(dir * _vMaxMu);                 // set speed & direction of movement
  _rotateControl->rotateAsync(*_motor);               // start moving

  Serial1COM.writeByte(2);                            // send serial message 2: "Start"
  digitalWriteFast(LED_BUILTIN, HIGH);                // enable built-in LED
}

void StepperWrapper_TeensyStep::softStop() {
  if (_stepControl->isRunning())
    _stepControl->stopAsync();
  else if (_rotateControl->isRunning()) {
    _rotateControl->stopAsync();
  }
}

float StepperWrapper_TeensyStep::vMax() {
  return _vMax;
}

void StepperWrapper_TeensyStep::vMax(float vMax) {
  if (isRunning() || vMax == _vMax)
    return;

  // constrain speed
  float vMaxMax = 100000;                             // maximum pulse-rate (TODO: verify!)
  float vMaxMin = 1.0 / _msRes;                       // minimum pulse-rate
  _vMax = constrain(vMax, vMaxMin, vMaxMax);
  DEBUG_PRINTF("Setting peak velocity to %1.0f steps/s\n",_vMax);

  // always use the highest possible micro-stepping resolution for a given vMax
  uint8_t exp = 8 - ceil( log(ceil(vMax*_msRes/vMaxMax))/log(2.0) );
  setMicrosteps(pow(2,exp));
}

void StepperWrapper_TeensyStep::CBstop() {
  StepperWrapper_TeensyStep* w = (StepperWrapper_TeensyStep*) wrapper;
  Serial1COM.writeByte(3);
  digitalWriteFast(LED_BUILTIN, LOW);

  // Bit of a hack, as the callback is sometimes fired slightly too early:
  w->postStopTimer.begin(StepperWrapper_TeensyStep::PostStop, 1);
}

void StepperWrapper_TeensyStep::PostStop() {
  StepperWrapper_TeensyStep* w = (StepperWrapper_TeensyStep*) wrapper;
  w->postStopTimer.end();
  w->updateMicroPosition();
  DEBUG_PRINTF("Motor stopped at position %d\n",w->_microPosition);
  w->writePosition(w->_microPosition);
}

bool StepperWrapper_TeensyStep::isRunning() {
  return _stepControl->isRunning() || _rotateControl->isRunning();
}

void StepperWrapper_TeensyStep::setMicrosteps(uint16_t ms) {
  StepperWrapper::setMicrosteps(ms);
  updateMicroPosition();
  _vMaxMu = round(_vMax * _microsteps);
  _aMu    = round(_a * _microsteps);
}

void StepperWrapper_TeensyStep::updateMicroPosition() {
  noInterrupts();
  _microPosition += floor(_motor->getPosition() * (_msRes/_microsteps));
  interrupts();
  _motor->setPosition(0);
}
