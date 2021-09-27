#ifndef TMC5160StepperExt_h
#define TMC5160StepperExt_h

class TMC5160StepperExt : public TMC5160Stepper {
public:
  TMC5160StepperExt(uint16_t pinCS, float RS);
  TMC5160StepperExt(uint16_t pinCS, float RS, uint16_t pinMOSI, uint16_t pinMISO, uint16_t pinSCK);
  using TMC5160Stepper::RAMP_STAT;
  void RAMP_STAT(uint16_t input);
};

#endif
