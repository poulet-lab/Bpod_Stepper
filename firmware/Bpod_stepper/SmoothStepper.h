/*
  SmoothStepper.h
*/
#ifndef SmoothStepper_h
#define SmoothStepper_h

#include "Arduino.h"

class SmoothStepper {
  public:
    SmoothStepper(uint8_t pinStep, uint8_t pinDirection);
    void moveSteps(long nSteps);
    void setPinEnable(uint8_t pinEnable);

    void setInvertEnable(bool invert);

    // set the acceleration (steps / s^2)
    void setAcceleration(float acceleration);

    // set the maximum speed (steps / s)
    void setMaxSpeed(float maxSpeed);

    // set the maximum speed (rounds / min)
    void setMaxRPM(float maxRPM);

    // set the number of steps per revolution
    void setStepsPerRev(unsigned int stepsPerRev);

    // set the duration of step pulses (µs)
    void setPulseWidth(unsigned int pulseWidth);

    // enables the stepper motor  driver (via _pinEnable)
    void enableDriver();

    // disables the stepper motor  driver (via _pinEnable)
    void disableDriver();
    
  private:
    void step();                      // 
    uint8_t _pinStep;                 // pin number: step
    uint8_t _pinDirection;            // pin number: direction
    uint8_t _pinEnable;               // pin number: enable
    float _a;                         // acceleration (steps / s^2)
    float _vMax;                      // maximum speed (steps / s)
    float _c0;                        // duration of first interval (µs)
    unsigned int _pulseWidth = 1;     // duration of step pulses (µs)
    unsigned int _stepsPerRev = 200;  // steps per full revolution
    bool _invertEnable = false;       // invert the enable pin?
};

#endif
