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
#include <SPI.h>
#include <TMCStepper.h>
#include "EEstore.h"                        // Import EEstore library
#include "StepperWrapper.h"
#include <avr/io.h>
#include <avr/interrupt.h>
#include "SerialDebug.h"

const float PCBrev        = StepperWrapper::idPCB();
const teensyPins pin      = StepperWrapper::getPins(PCBrev);
const uint8_t vDriver     = StepperWrapper::idDriver();
volatile uint8_t errorID  = 0;
IntervalTimer timerErrorBlink;

StepperWrapper::StepperWrapper() {
  DEBUG_PRINTFUN();
  pinMode(pin.En, OUTPUT);
  pinMode(LED_BUILTIN, OUTPUT);
  if (PCBrev>=1.4) {
    pinMode(pin.VIO, OUTPUT);

    pinMode(pin.Diag0, INPUT_PULLUP);       // DIAG pins on TMC5160 ...
    pinMode(pin.Diag1, INPUT_PULLUP);       // use open collector output
    //attachInterrupt(digitalPinToInterrupt(pin.Diag0), ISRdiag0, CHANGE);
    //attachInterrupt(digitalPinToInterrupt(pin.Diag1), ISRdiag1, CHANGE);

    pinMode(pin.VM, INPUT);
    attachInterrupt(digitalPinToInterrupt(pin.VM), ISRchangeVM, CHANGE);
    if (!digitalRead(pin.VM))
      throwError(1);
  }

  enableDriver(false);                      // disable driver for now
  powerDriver(true);                        // power the driver
}


void StepperWrapper::ISRdiag0() {
  if (errorID)
    return;
  TMC2130Stepper* driver = get2130();
  if (driver->drv_err()) {
    if (driver->uv_cp())
      throwError(3);                        // undervoltage charge-pump
    else if (driver->otpw())
      throwError(4);                        // overtemperature pre-warning threshold is exceeded
    else if (driver->ot())
      throwError(5);                        // overtemperature limit has been reached
    else if (driver->s2gb())
      throwError(6);                        // short to ground indicator phase B
    else if (driver->s2ga())
      throwError(7);                        // short to ground indicator phase A
  }
  // TODO
}


void StepperWrapper::ISRdiag1() {
  // TODO
}


void StepperWrapper::ISRchangeVM() {
  DEBUG_PRINTFUN();
  if (digitalRead(pin.VM))
   SCB_AIRCR = 0x05FA0004;                  // reset teensy
  else
    throwError(1);
}


void StepperWrapper::init(uint16_t rms_current) {
  DEBUG_PRINTFUN(rms_current);

  if (vDriver == 0x11)
    init2130(rms_current);                  // initialize TMC2130
  else if (vDriver == 0x30)
    init5160(rms_current);                  // initialize TMC5160
  else
    init2100();                             // initialize TMC2100

  enableDriver(true);                       // activate motor outputs
}


void StepperWrapper::init2100() {
  DEBUG_PRINTFUN();
  StepperWrapper::setMicrosteps(16);        // set microstep resolution
}


void StepperWrapper::init2130(uint16_t rms_current) {
  DEBUG_PRINTFUN();

  _invertPinDir = true;

  TMC2130Stepper* driver = get2130();

  // configuration of DIAG pins & interrupts
  driver->GSTAT(0b111);                     // reset error flags
  driver->diag0_error(true);                // enable DIAG0 active on driver errors
  driver->diag0_otpw(true);                 // enable DIAG0 active on driver over temperature prewarning
  attachInterrupt(digitalPinToInterrupt(pin.Diag0), ISRdiag0, CHANGE);
  attachInterrupt(digitalPinToInterrupt(pin.Diag1), ISRdiag1, CHANGE);

  driver->rms_current(rms_current,1);       // set motor current, standstill reduction disabled
  enableDriver(true);                       // activate motor outputs

  // StealthChop configuration
  driver->en_pwm_mode(1);                   // enable StealthChop
  setMicrosteps(256);                       // set microstep resolution
  driver->intpol(1);                        // interpolation to 256 microsteps
  driver->pwm_autoscale(1);                 // enable automatic tuning of PWM amplitude offset
  driver->pwm_grad(4);                      // amplitude regulation loop gradient
  driver->pwm_ampl(128);                    // user defined amplitude (offset)
  driver->pwm_freq(0b01);                   // set PWM Frequency (~38kHz with internal clock)
  delay(150);                               // stand still for automatic tuning AT#1

  // Chopper configuration
  driver->chm(0);                           // Chopper mode: SpreadCycle
  driver->toff(4);                          // Chopper slow decay time. Required to enable the motor.
  driver->tbl(2);
  driver->hstrt(4);
  driver->hend(0);
  driver->TPWMTHRS(0);                      // Disable SpreadCycle chopper (use StealthChop only)

  // Power savings
  driver->rms_current(rms_current,0);       // Set motor current, standstill reduction enabled
  driver->TPOWERDOWN(0);
  driver->iholddelay(0);                    // instant IHOLD
  driver->freewheel(0x01);                  // 0x00 = normal operation, 0x01 = freewheeling
}


void StepperWrapper::init5160(uint16_t rms_current) {
  DEBUG_PRINTFUN(rms_current);

  _invertPinDir = false;

  TMC5160Stepper* driver = get5160();

  //_driver->RAMP_STAT(ramp_stat);            // clear RAMP_STAT flags (TODO: TMC5160StepperExt)
  driver->GSTAT(0b111);                    // reset error flags

  // TODO: Check global & driver status
  // if (_driver.uv_cp())
  //   throwError(42);                         // Error: Charge pump undervoltage
  // else if (_driver.drv_err())
  // {
  //   if (bitRead(_driver.DRV_STATUS(),12))
  //     throwError(42);                       // Error: Short to supply phase A.
  //   else if (bitRead(_driver.DRV_STATUS(),13))
  //     throwError(42);                       // Error: Short to supply phase B.
  //   else if (_driver.s2ga())
  //     throwError(42);                       // Error: Short to ground phase A.
  //   else if (_driver.s2gb())
  //     throwError(42);                       // Error: Short to ground phase B.
  //   else if (_driver.ot())
  //     throwError(42);                       // Error: Overtemperature.
  // }

  driver->rms_current(rms_current,1);       // set motor current, standstill reduction disabled
  enableDriver(true);                       // activate motor outputs

  // StealthChop configuration
  driver->en_pwm_mode(1);                   // enable StealthChop
  setMicrosteps(256);                       // set microstep resolution
  driver->pwm_freq(0b01);                   // set PWM Frequency (35.1kHz with 12MHz internal clock)
  driver->pwm_ofs(30);                      // initial value: PWM amplitude offset (TODO: Load from EEPROM?)
  driver->pwm_grad(0);                      // initial value: PWM amplitude gradient (TODO: Load from EEPROM?)
  driver->pwm_autoscale(1);                 // enable automatic tuning of PWM amplitude offset
  driver->pwm_autograd(1);                  // enable automatic tuning of PWM amplitude gradient
  delay(150);                               // stand still for automatic tuning AT#1

  // Chopper configuration
  driver->chm(0);                           // Chopper mode: SpreadCycle
  driver->toff(5);                          // Chopper slow decay time. Required to enable the motor.
  // driver->tbl(2);
  // driver->hstrt(4);
  // driver->hend(0);
  driver->TPWMTHRS(0);                      // Disable SpreadCycle chopper (use StealthChop only)

  // Power savings
  driver->rms_current(rms_current,0);       // Set motor current, standstill reduction enabled
  driver->TPOWERDOWN(0);
  driver->iholddelay(7);                    // Delayed motor power down after standstill
  driver->freewheel(0x00);                  // 0x00 = normal operation, 0x01 = freewheeling

  // TODO: StallGuard / CoolStep
  // driver->TCOOLTHRS(* 256);
  // driver->sgt(2);
  // driver->sg_stop(true);
  // driver->THIGH(1500 * 256);
  // driver->tbl(2);
  // driver->chm(0);
}


void StepperWrapper::blinkenlights() {
  DEBUG_PRINTFUN();
  pinMode(LED_BUILTIN, OUTPUT);
  for (int i = 750; i > 0; i--) {
    delayMicroseconds(i);
    digitalWriteFast(LED_BUILTIN, HIGH);
    delayMicroseconds(750 - i);
    digitalWriteFast(LED_BUILTIN, LOW);
  }
  for (int i = 1; i <= 2 ; i++) {
    delay(75);
    digitalWriteFast(LED_BUILTIN, HIGH);
    delay(50);
    digitalWriteFast(LED_BUILTIN, LOW);
  }
}


void StepperWrapper::powerDriver(bool power) {
  DEBUG_PRINTFUN(power);
  pinMode(pin.VIO, OUTPUT);
  if (PCBrev>=1.4) {
    digitalWrite(pin.VIO, power);
    delay(150);
  }
}


void StepperWrapper::enableDriver(bool enable) {
  DEBUG_PRINTFUN(enable);
  pinMode(pin.En, OUTPUT);
  digitalWrite(pin.En, enable ^ _invertPinEn);
}


float StepperWrapper::idPCB() {
  DEBUG_PRINTFUN();
  // r1.4 onwards has the revision number coded in hardware.
  // It can be read by checking if pins 20-23 are connected
  // to GND. The connection status forms a binary code:
  //
  //       pin20 pin21 pin22 pin23
  // r1.4x                     x
  // r1.5x               x
  // r1.6x               x     x
  // r1.7x         x
  // etc.
  uint8_t x = 0;
  for (int i = 23; i >= 20; i--) {
    pinMode(i, INPUT_PULLUP);
    delayMicroseconds(10);
    bitWrite(x, abs(i - 23), !digitalRead(i));
    pinMode(i, INPUT_DISABLE);
  }
  if (x)
    return 1.3 + x / 10.0;
  else {
    // revisions older than 1.4 can be identified by other means:
    pinMode(29, INPUT);
    if (digitalRead(29)) {        // with r1.2 and r1.3 pin 29 reads HIGH
      pinMode(29, INPUT_DISABLE);
      pinMode(9, OUTPUT);
      digitalWrite(9, LOW);
      pinMode(14, INPUT_PULLUP);
      delayMicroseconds(10);
      bool tmp = !digitalRead(14);
      pinMode(9, INPUT_DISABLE);
      pinMode(14, INPUT_DISABLE);
      if (tmp)                    // r1.3 connects pin 9 and 14 ...
        return 1.3;
      else                        // ... while r1.2 does not
        return 1.2;
    } else                        // otherwise its r1.1
      return 1.1;
  }
}


uint8_t StepperWrapper::idDriver() {
  DEBUG_PRINTFUN();
  TMC2130Stepper* driver = get2130();
  if (!driver->test_connection()) // if we can connect via SPI
    return driver->version();     // return driver version
  else
    return 0;
}


teensyPins StepperWrapper::getPins(float PCBrev) {
  DEBUG_PRINTFUN(PCBrev);
  teensyPins pin;
  pin.Error = 33;
  if (PCBrev <= 1.1) {  // the original layout
    pin.Dir   =  2;
    pin.Step  =  3;
    pin.Sleep =  4;
    pin.Reset =  5;
    pin.CFG3  =  6;
    pin.CFG2  =  7;
    pin.CFG1  =  8;
    pin.En    =  9;
    pin.IO1   = 10;
    pin.IO2   = 11;
    return pin;
  } else {              // major reorganisation with r1.2
    pin.Dir   =  4;
    pin.Step  =  5;
    pin.Sleep =  6;
    pin.Reset = 12;
    pin.CFG3  =  8;
    pin.CFG2  = 27;
    pin.CFG1  = 11;
    pin.En    = 24;
    pin.IO1   = 36;
    pin.IO2   = 37;
    pin.IO3   = 38;
    pin.IO4   = 14;
    pin.IO5   = 18;
    pin.IO6   = 19;
  }
  if (PCBrev >= 1.3) {  // corrected layout for hardware SPI with r1.3
    pin.Dir   =  5;
    pin.Step  =  6;
    pin.Sleep =  7;
    pin.Reset =  8;
    pin.CFG3  = 10;
    pin.CFG2  = 14;
    pin.En    = 12;
    pin.IO4   = 15;
  }
  if (PCBrev >= 1.4) {
    pin.Diag0 = 24;
    pin.Diag1 = 25;
    pin.VIO   = 28;
    pin.VM    = 4 ;
  }
  return pin;
}


TMC2130Stepper* StepperWrapper::get2130() {
  DEBUG_PRINTFUN();
  static bool initialized = false;
  static TMC2130Stepper* driver;

  if (!initialized) {
    powerDriver(true);
    if (PCBrev<1.3)
      driver = new TMC2130Stepper(pin.CFG3, 0.110, pin.CFG1, pin.Reset, pin.CFG2);
    else {
      driver = new TMC2130Stepper(pin.CFG3, 0.110);
      SPI.setMISO(pin.Reset);
      SPI.setMOSI(pin.CFG1);
      SPI.setSCK(pin.CFG2);
      SPI.begin();
    }
    driver->begin();
    initialized = true;
  }

  return driver;
}


TMC5160Stepper* StepperWrapper::get5160() {
  DEBUG_PRINTFUN();
  static bool initialized = false;
  static TMC5160Stepper* driver;

  if (!initialized) {
    powerDriver(true);
    if (PCBrev<1.3)
      driver = new TMC5160Stepper(pin.CFG3, 0.075, pin.CFG1, pin.Reset, pin.CFG2);
    else {
      driver = new TMC5160Stepper(pin.CFG3, 0.075);
      SPI.setMISO(pin.Reset);
      SPI.setMOSI(pin.CFG1);
      SPI.setSCK(pin.CFG2);
      SPI.begin();
    }
    driver->begin();
    initialized = true;
  }

  return driver;
}


bool StepperWrapper::SDmode() {
  DEBUG_PRINTFUN();
  bool sd_mode = true;
  if (vDriver == 0x30) {
    TMC5160Stepper* driver = get5160();
    if (!driver->test_connection())
      sd_mode = driver->sd_mode();
  }
  return sd_mode;
}


void StepperWrapper::ISRblinkError() {
  digitalWriteFast(LED_BUILTIN, !digitalReadFast(LED_BUILTIN));
}


void StepperWrapper::throwError(uint8_t ID) {
  DEBUG_PRINTFUN(ID);
  if (errorID)
    return;
  enableDriver(false);
  errorID = ID;
  detachInterrupt(digitalPinToInterrupt(pin.Diag0));
  digitalWrite(pin.Error, HIGH);
  timerErrorBlink.begin(ISRblinkError, 250000);
}


void StepperWrapper::clearError() {
  DEBUG_PRINTFUN();
  errorID = 0;
  digitalWrite(pin.Error, LOW);
  timerErrorBlink.end();
  digitalWriteFast(LED_BUILTIN, LOW);
}


void StepperWrapper::RMS(uint16_t rms_current) {
  switch (vDriver) {
    case 0x11:
      {
        rms_current = constrain(rms_current,0,850);
        TMC2130Stepper* driver = get2130();
        driver->rms_current(rms_current);
        return;
      }
    case 0x30:
      {
        rms_current = constrain(rms_current,0,2000);
        TMC5160Stepper* driver = get5160();
        driver->rms_current(rms_current);
        return;
      }
    default:
      return;
  }
}


uint16_t StepperWrapper::RMS() {
  switch (vDriver) {
    case 0x11:
      {
        TMC2130Stepper* driver = get2130();
        return(driver->rms_current());
      }
    case 0x30:
      {
        TMC5160Stepper* driver = get5160();
        return(driver->rms_current());
      }
    default:
      return 0;
  }
}


void StepperWrapper::setMicrosteps(uint16_t ms) {
  DEBUG_PRINTFUN(ms);
  switch (vDriver) {
    case 0x11:
      {
        TMC2130Stepper* driver = get2130();
        driver->microsteps(ms);
        _microsteps = driver->microsteps();
        return;
      }
    case 0x30:
      {
        TMC5160Stepper* driver = get5160();
        driver->microsteps(ms);
        _microsteps = driver->microsteps();
        return;
      }
  }

  switch (ms) {
    case 16:
      pinMode(pin.CFG1, INPUT);
      pinMode(pin.CFG2, INPUT);
      break;
    case 4:
      pinMode(pin.CFG1, OUTPUT);
      pinMode(pin.CFG2, INPUT);
      digitalWrite(pin.CFG1, HIGH);
      break;
    case 2:
      pinMode(pin.CFG1, INPUT);
      pinMode(pin.CFG2, OUTPUT);
      digitalWrite(pin.CFG2, LOW);
      break;
    default:
      ms = 1;
      pinMode(pin.CFG1, OUTPUT);
      pinMode(pin.CFG2, OUTPUT);
      digitalWrite(pin.CFG1, LOW);
      digitalWrite(pin.CFG2, LOW);
  }
  _microsteps = ms;
}
