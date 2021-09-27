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
#include "SmoothStepper.h"
#include <TMCStepper.h>

StepperWrapper_SmoothStepper::StepperWrapper_SmoothStepper() : StepperWrapper() {}

void StepperWrapper_SmoothStepper::init(uint16_t rms_current) {
  StepperWrapper::init(rms_current);
  
  _stepper = new SmoothStepper(_pin.Step, _pin.Dir);

  _stepper->setStepsPerRev(200 * (uint32_t) _microsteps);

  _stepper->setPinEnable(_pin.En);            // We do want to use the enable pin
  _stepper->setInvertEnable(_invertPinEn);    // Enable pin on TMC2100 is inverted
  _stepper->setInvertDirection(_invertPinDir);// Invert the direction pin?
  _stepper->resetPosition();                  // Reset position of motor

  enableDriver(true);                         // Enable the driver
}

void StepperWrapper_SmoothStepper::setAcceleration(float aHzs) {
  _stepper->setAcceleration(aHzs * (float) _microsteps);
}

void StepperWrapper_SmoothStepper::setSpeed(float v) {
  _stepper->setMaxSpeed(v * (float) _microsteps);
}

void StepperWrapper_SmoothStepper::setTarget(int32_t target) {
  _stepper->movePosition(target * _microsteps);
}

void StepperWrapper_SmoothStepper::setMicrosteps(uint16_t ms) {
  StepperWrapper::setMicrosteps(ms);
  _stepper->setStepsPerRev(200 * _microsteps);
}

int32_t StepperWrapper_SmoothStepper::getPosition() {
  return _stepper->getPosition() / (int32_t) _microsteps;
}

void StepperWrapper_SmoothStepper::resetPosition() {
  _stepper->resetPosition();
}