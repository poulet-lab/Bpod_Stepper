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

_______________________________________________________________________________


REVISION HISTORY

version 1.0.0   initial release
version 1.0.1   various cleanups / style fixes (thank you: Florian Uekermann)
version 1.0.2   add stop routine for interrupting movement

_______________________________________________________________________________


SmoothStepper uses algorithms from the following publication:

Austin D (2005) Generate stepper-motor speed profiles in real time.
EE Times-India: 01/2005:1-5.

_______________________________________________________________________________
*/
#ifndef SmoothStepper_h
#define SmoothStepper_h

#include "Arduino.h"

class SmoothStepper {
  public:
    // constructor
    SmoothStepper(uint8_t pinStep, uint8_t pinDirection);

    // set enable pin
    void setPinEnable(uint8_t pinEnable);

    // invert enable pin
    void setInvertEnable(bool invert);

    // invert direction pin
    void setInvertDirection(bool invert);

    // set the number of steps per revolution
    void setStepsPerRev(uint32_t stepsPerRev);

    // set the acceleration (steps / s^2)
    void setAcceleration(float acceleration);

    // set the maximum speed (steps / s)
    void setMaxSpeed(float maxSpeed);

    // set the duration of step pulses (µs)
    void setPulseWidth(uint16_t pulseWidth);

    // enables the stepper motor  driver (via _pinEnable)
    void enableDriver();

    // disables the stepper motor  driver (via _pinEnable)
    void disableDriver();

    // move by n steps
    void moveSteps(int32_t nSteps);

    // move by n degrees
    void moveDegrees(float degrees);

    // stop movement
    void stop();

  private:
    void step();                      // step function
    bool _invertDirection = false;    // invert the direction pin?
    bool _invertEnable = false;       // invert the enable pin?
    volatile bool _running = false;   // are we moving the motor?
    volatile bool _stop = false;      // should we stop the movement?
    uint8_t _pinDirection;            // pin number: direction
    uint8_t _pinEnable;               // pin number: enable
    uint8_t _pinStep;                 // pin number: step
    uint16_t _pulseWidth = 1;         // duration of step pulses (µs)
    uint32_t _stepsPerRev = 200;      // steps per revolution
    float _a;                         // acceleration (steps / s^2)
    float _vMax;                      // maximum speed (steps / s)
    float _c0;                        // duration of first interval (µs)
};

#endif
