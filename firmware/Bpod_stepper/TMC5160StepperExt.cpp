#include <Arduino.h>
#include <TMCStepper.h>
#include "TMC5160StepperExt.h"

TMC5160StepperExt::TMC5160StepperExt(uint16_t pinCS, float RS) : TMC5160Stepper(pinCS, RS, -1) {}
TMC5160StepperExt::TMC5160StepperExt(uint16_t pinCS, float RS, uint16_t pinMOSI, uint16_t pinMISO, uint16_t pinSCK) : TMC5160Stepper(pinCS, RS, pinMOSI, pinMISO, pinSCK, -1) {}
void TMC5160StepperExt::RAMP_STAT(uint16_t input) {write(RAMP_STAT_t::address,input);};
