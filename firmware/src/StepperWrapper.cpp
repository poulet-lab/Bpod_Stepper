/*
StepperWrapper
Copyright (C) 2023 Florian Rau

This program is free software: you can redistribute it and/or modify it under
the terms of the GNU General Public License as published by the Free Software
Foundation, version 3.

This program is distributed  WITHOUT ANY WARRANTY and without even the implied
warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU
General Public License for more details.

You should have received a copy of the GNU General Public License along with
this program.  If not, see <http://www.gnu.org/licenses/>.
*/

#include "StepperWrapper.h"
#include "EEstore.h"
#include "EEstoreStruct.h"
#include "SerialDebug.h"
#include <Arduino.h>
#include <TMCStepper.h>
#include <avr/interrupt.h>
#include <avr/io.h>
#include <functional>

#define SPREADCYCLE  false
#define CONSTOFFTIME true

// In order to use identically named methods of different child classes of
// TMCStepper we define lambda functions that are initalized by the following
// pre-processor macro:
#define setLambdas(d)                                                          \
  {                                                                            \
  cs2rms            = [d](uint8_t CS)   { return d->cs2rms(CS); };            \
  sg_result         = [d]()             { return d->sg_result(); };           \
  get_en_pwm_mode   = [d]()             { return d->en_pwm_mode(); };         \
  set_en_pwm_mode   = [d](bool val)     {        d->en_pwm_mode(val); };      \
  get_freewheel     = [d]()             { return d->freewheel() == 0x01; };   \
  set_freewheel     = [d](bool val)     {        d->freewheel(val); };        \
  get_ihold         = [d]()             { return d->ihold(); };               \
  set_ihold         = [d](uint8_t CS)   {        d->ihold(CS); };             \
  get_microsteps    = [d]()             { return d->microsteps(); };          \
  set_microsteps    = [d](uint8_t ms)   {        d->microsteps(ms); };        \
  get_rms_current   = [d]()             { return d->rms_current(); };         \
  set_rms_current   = [d](uint16_t I)   {        d->rms_current(I); };        \
  get_sfilt         = [d]()             { return d->sfilt(); };               \
  set_sfilt         = [d](bool val)     {        d->sfilt(val); };            \
  get_sgt           = [d]()             { return d->sgt(); };                 \
  set_sgt           = [d](uint8_t val)  {        d->sgt(val); };              \
  get_TPWMTHRS      = [d]()             { return d->TPWMTHRS(); };            \
  set_TPWMTHRS      = [d](uint32_t val) {        d->TPWMTHRS(val); };         \
}

extern ArCOM usbCOM;

const uint8_t PCBrev      = StepperWrapper::idPCB();
const teensyPins pin      = StepperWrapper::getPins(PCBrev);
volatile uint8_t errorID  = 0;
volatile uint8_t ISRcode  = 0;
IntervalTimer timerErrorBlink;

// initialize static/const members
const uint8_t StepperWrapper::vDriver = StepperWrapper::idDriver();
TMC2130Stepper* const StepperWrapper::TMC2130 = StepperWrapper::get2130();
TMC5160Stepper* const StepperWrapper::TMC5160 = StepperWrapper::get5160();
const bool StepperWrapper::is2130 = StepperWrapper::idDriver() == 0x11;
const bool StepperWrapper::is5160 = StepperWrapper::idDriver() == 0x30;
const uint32_t StepperWrapper::fCLK = (StepperWrapper::is2130) ? 12.2E6 : 12E6;
const float StepperWrapper::RSense = (StepperWrapper::is2130) ? 0.110 : 0.075;
const char * StepperWrapper::name = StepperWrapper::getName();

StepperWrapper::StepperWrapper() {

  // set number of IO pins
  _nIO = (PCBrev<12) ? 2 : 6;

  pinMode(pin.En, OUTPUT);
  pinMode(LED_BUILTIN, OUTPUT);

  if (PCBrev>=14) {
    pinMode(pin.VIO, OUTPUT);

    // attach interrupts for diagnostic pins
    pinMode(pin.Diag0, INPUT);
    pinMode(pin.Diag1, INPUT);
    //attachInterrupt(digitalPinToInterrupt(pin.Diag0), ISRdiag0, RISING);
    attachInterrupt(digitalPinToInterrupt(pin.Diag1), ISRdiag1, CHANGE);

    // detect presence of motor power supply
    pinMode(pin.VM, INPUT);
    attachInterrupt(digitalPinToInterrupt(pin.VM), ISRchangeVM, CHANGE);
    if (!digitalRead(pin.VM)) {
      throwError(1);
    }

    // throw error if no supported driver found
    if (vDriver == 0) {
      throwError(1);
  }
  }

  TimerStream.priority(255);                // lowest priority for stream

  enableDriver(false);                      // disable driver for now
  powerDriver(true);                        // power the driver
}

void StepperWrapper::ISRstream() {
  static uint32_t data[3];
  static std::function<uint32_t()> DRV_STATUS;
  static std::function<uint32_t()> TSTEP;

  if (!DRV_STATUS) {
    if (is2130) {
      TMC2130Stepper* d = StepperWrapper::get2130();
      DRV_STATUS        = [&d]() { return d->DRV_STATUS(); };
      TSTEP             = [&d]() { return d->TSTEP(); };
    } else if (is5160) {
      TMC5160Stepper* d = StepperWrapper::get5160();
      DRV_STATUS        = [&d]() { return d->DRV_STATUS(); };
      TSTEP             = [&d]() { return d->TSTEP(); };
    }
  }

  data[0] = millis();
  data[1] = DRV_STATUS();
  data[2] = TSTEP();
  usbCOM.writeUint32Array(data,3);
}

bool StepperWrapper::getStream() { return StatusTimerStream; }

void StepperWrapper::setStream(bool enable) {
  if (enable && !StatusTimerStream) {
    StatusTimerStream = TimerStream.begin(StepperWrapper::ISRstream, 50000);
    return;
  } else if (!enable && StatusTimerStream) {
    TimerStream.end();
  }
  StatusTimerStream = false;
}

void StepperWrapper::ISRdiag0() {
  DEBUG_PRINTLN("DIAG0!");
  if (errorID) {
    return;
  }
  uint8_t gstat = get2130()->GSTAT();
  if (bitRead(gstat, 2)) {
    throwError(3);                          // under-voltage charge-pump
  } else if (bitRead(gstat, 1)) {
    uint32_t status = get2130()->DRV_STATUS();
    if (bitRead(status, 25)) {
      throwError(4);                        // over-temperature limit has been reached
    } else if (bitRead(status, 26)) {
      throwError(5);                        // over-temperature pre-warning threshold is exceeded
    } else if (bitRead(status, 27)) {
      throwError(6);                        // short to ground indicator phase A
    } else if (bitRead(status, 28)) {
      throwError(7);                        // short to ground indicator phase B
  }
  }
  // TODO
}

void StepperWrapper::ISRdiag1() {
  noInterrupts();
  DEBUG_PRINTLN("Stall detected!");
  interrupts();
  ISRhardStop();
}

void StepperWrapper::ISRchangeVM() {
  if (digitalReadFast(pin.VM)) {
    cli();
    SCB_AIRCR = 0x05FA0004;                 // reset teensy
    asm volatile ("dsb");
    while (true) {}
  } else {
    throwError(1);
}
}

void StepperWrapper::init() {
  if (is2130) {
    init(TMC2130);
  } else if (is5160) {
    init(TMC5160);
  }
}

void StepperWrapper::init(auto *stepper) {
  DEBUG_PRINT("\n");
  DEBUG_PRINTF("Initializing %s\n", name);

  // preliminaries
  setLambdas(stepper);                      // set lambdas using preprocessor macro
  _invertPinDir = is2130;                   // get identical directions for all drivers

  // Initialize SDcard, load position
  useSD = this->SD.begin(SdioConfig(FIFO_SDIO));
  DEBUG_PRINTF("SD card %s\n",
               (useSD) ? "detected - initalizing" : "NOT detected");
  if (this->useSD) {
    this->filePos.open("position.bin", O_RDWR | O_CREAT);
  }

  // error states / diagnostics
  stepper->GSTAT();                         // reset error flags
  stepper->diag0_error(true);               // enable DIAG0 active on driver errors
  stepper->diag1_stall(true);               // enable DIAG1 active on driver stall
  stepper->diag0_int_pushpull(false);
  stepper->diag1_pushpull(false);

  // micro-stepping
  StepperWrapper::setMicrosteps(256);       // highest micro-stepping resolution
  stepper->intpol(true);                    // always interpolate to 256 micro-steps

  // StealthChop configuration
  stepper->rms_current(p.rms_current,
                       1); // set motor current, standstill reduction disabled
  enableDriver(true);                       // activate motor outputs
  stepper->en_pwm_mode(true);               // enable StealthChop
  stepper->pwm_grad(
      0); // initial value: PWM amplitude gradient (TODO: Load from EEPROM?)
  if (is5160) {                             // WITH TMC5160:
    TMC5160->pwm_autograd(
        true);            //   enable automatic tuning of PWM amplitude gradient
    TMC5160->pwm_ofs(30);                   //   initial value: PWM amplitude offset
  }
  stepper->pwm_autoscale(
      true);               // enable automatic tuning of PWM amplitude offset
  stepper->pwm_ampl(128);                   // user defined amplitude (offset)
  stepper->pwm_freq(0b01);                  // set PWM Frequency
  delay(150);                               // stand still for automatic tuning AT#1

  // Chopper configuration
  stepper->chm(SPREADCYCLE);                // chopper mode: SpreadCycle
  stepper->toff(4);                         // chopper slow decay time. Required to enable the motor.
  stepper->tbl(2);
  stepper->hstrt(4);
  stepper->hend(0);

  // stallGuard2
  stepper->TPWMTHRS(0); // disable SpreadCycle chopper (use StealthChop only)

  stepper->sgt(0);
  stepper->TCOOLTHRS(1048575);
  stepper->semin(1);

  stepper->TPOWERDOWN(0);
  stepper->iholddelay(0);                   // instant IHOLD

  // Load parameters from EEPROM
  DEBUG_PRINT("\n");
  DEBUG_PRINTLN("Loading parameters from EEPROM:");
  RMS(p.rms_current);
  holdRMS(p.hold_rms_current);
  vMax(p.vMax);
  a(p.a);
  setIOresistor(p.IOresistor,sizeof(p.IOresistor));
  setIOmode(p.IOmode,sizeof(p.IOmode));
  setChopper(p.chopper);
  setPosition(readPosition());
  DEBUG_PRINT("\n");
}

TMC2130Stepper* StepperWrapper::get2130() {
  static TMC2130Stepper* driver = nullptr;

  if (!driver) {
    TMC2130Stepper* tmp;
    powerDriver(true);
    if (PCBrev < 13) {
      tmp = new TMC2130Stepper(pin.CFG3, 0.110, pin.CFG1, pin.Reset, pin.CFG2);
    } else {
      tmp = new TMC2130Stepper(pin.CFG3, 0.110);
      SPI.setMISO(pin.Reset);
      SPI.setMOSI(pin.CFG1);
      SPI.setSCK(pin.CFG2);
      SPI.begin();
    }
    delay(50);
    tmp->begin();
    if (!tmp->test_connection() && tmp->version() == 0x11) {
      driver = tmp;
  }
  }
  return driver;
}

TMC5160Stepper* StepperWrapper::get5160() {
  static TMC5160Stepper* driver = nullptr;

  if (!driver) {
    TMC5160Stepper* tmp;
    powerDriver(true);
    if (PCBrev < 13) {
      tmp = new TMC5160Stepper(pin.CFG3, 0.075, pin.CFG1, pin.Reset, pin.CFG2);
    } else {
      tmp = new TMC5160Stepper(pin.CFG3, 0.075);
      SPI.setMISO(pin.Reset);
      SPI.setMOSI(pin.CFG1);
      SPI.setSCK(pin.CFG2);
      SPI.begin();
    }
    delay(50);
    tmp->begin();
    if (!tmp->test_connection() && tmp->version() == 0x30) {
      driver = tmp;
  }
  }
  return driver;
}

void StepperWrapper::blinkenlights() {
  DEBUG_PRINTLN("Blinkenlights!");
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
  DEBUG_PRINTLN("Start-up finished. Waiting for commands.\n");
}

void StepperWrapper::powerDriver(bool power) {
  static bool state = false;

  if (PCBrev < 14 || state == power) {
    return;
  }

  DEBUG_PRINTF("Powering %s driver.\n", (power) ? "up" : "down");
  pinMode(pin.VIO, OUTPUT);
  digitalWriteFast(pin.VIO, power);
  state = power;
  delay(200);
}

void StepperWrapper::enableDriver(bool enable) {
  DEBUG_PRINTLN((enable) ? "Enabling driver" : "Disabling driver");
  pinMode(pin.En, OUTPUT);
  digitalWrite(pin.En, enable ^ _invertPinEn);
}

uint8_t StepperWrapper::idPCB() {
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
  if (x) {
    return 13 + x;
  } else { // revisions older than 1.4:
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
      if (tmp) { // r1.3 connects pin 9 and 14 ...
        return 13;
      } else { // ... while r1.2 does not
        return 12;
      }
    } else { // otherwise its r1.1
      return 11;
  }
}
}

uint8_t StepperWrapper::idDriver() {
  static uint8_t id = 0;
  if (id == 0) {
    if (get2130()) {
      id = get2130()->version();
    } else if (get5160()) {
      id = get5160()->version();
  }
  }
  return id;
}

const char * StepperWrapper::getName() {
  if (get2130()) {
    return "TMC2130";
  } else if (get5160()) {
    return "TMC5160";
  } else {
    return "unknown";
}
}

teensyPins StepperWrapper::getPins(float PCBrev) {
  teensyPins pin;
  pin.Error = 33;
  if (PCBrev <= 11) {   // the original layout
    pin.Dir   =  2;
    pin.Step  =  3;
    pin.Sleep =  4;
    pin.Reset =  5;
    pin.CFG3  =  6;
    pin.CFG2  =  7;
    pin.CFG1  =  8;
    pin.En    =  9;
    pin.IO[0] = 10;     // IO1 - watch out, zero indexing!
    pin.IO[1] = 11;     // IO2
    return pin;
  } else {              // r1.2 major reorganization
    pin.Dir   =  4;
    pin.Step  =  5;
    pin.Sleep =  6;
    pin.Reset = 12;
    pin.CFG3  =  8;
    pin.CFG2  = 27;
    pin.CFG1  = 11;
    pin.En    = 24;
    pin.IO[0] = 36;     // IO1 - watch out, zero indexing!
    pin.IO[1] = 37;     // IO2
    pin.IO[2] = 38;     // IO3
    pin.IO[3] = 14;     // IO4
    pin.IO[4] = 18;     // IO5
    pin.IO[5] = 19;     // IO6
  }
  if (PCBrev >= 13) {   // r1.3: corrected layout for hardware SPI
    pin.Dir   =  5;
    pin.Step  =  6;
    pin.Sleep =  7;
    pin.Reset =  8;
    pin.CFG3  = 10;
    pin.CFG2  = 14;
    pin.En    = 12;
    pin.IO[3] = 15;     // IO4
  }
  if (PCBrev >=
      14) { // r1.4: support for DIAG pins, VIO control and VM monitoring
    pin.Diag0 = 24;
    pin.Diag1 = 25;
    pin.VIO   = 28;
    pin.VM    = 4 ;
  }
  if (PCBrev >= 15) {   // r1.5: support for hardware quadrature encoder
    pin.IO[0] = 30;     // IO1 - watch out, zero indexing!
    pin.IO[1] = 29;     // IO2
  }
  return pin;
}

void StepperWrapper::SGautoTune() {
  detachInterrupt(digitalPinToInterrupt(pin.Diag1));

  float vMaxOld = vMax();
  float aOld = a();
  int32_t mPosOld = microPosition();
  uint16_t sgRes;

  vMax(20);         // set velocity to a 20 full-steps per second
  a(200);           // set acceleration
  setChopper(0);    // enable PWM chopper
  set_sgt(0);       // reset StallGuard threshold to 0
  set_sfilt(true);  // enable 4-step averaging of StallGuard reading
  rotate(1);        // start moving the motor
  delay(250);       // wait for StallGuard reading to settle

  DEBUG_PRINTLN("Starting automatic tuning of stallGuard threshold:");
  for (int8_t sgt = 0; sgt < 64; sgt++) {
    set_sgt(sgt);         // set StallGuard threshold
    delay(200);           // wait 4 full-steps
    sgRes = sg_result();  // get StallGuard reading
    DEBUG_PRINTF("  SGT =% 3d; SG_RESULT = % 4d\n", sgt, sgRes);

    if (sgRes > 0) {      // once the SG reading starts to rise above zero
      set_sgt(--sgt);     // fall back to the last zero reading
      DEBUG_PRINTF("Setting stallGuard threshold to %d\n", sgt);
      break;
    }
  }

  softStop();             // decelerate the motor to a stop
  delay(200);
  vMax(vMaxOld);          //
  a(aOld);
  set_sfilt(false);
  microPosition(mPosOld);

  attachInterrupt(digitalPinToInterrupt(pin.Diag1), ISRdiag1, CHANGE);
}

bool StepperWrapper::SDmode() {
  bool sd_mode = true;
  if (is5160) {
    sd_mode = TMC5160->sd_mode();
  }
  DEBUG_PRINTF("Driver configured for %s.\n",
               (sd_mode) ? "step/dir interface" : "internal ramp generator");
  return sd_mode;
}

void StepperWrapper::ISRblinkError() {
  digitalWriteFast(LED_BUILTIN, !digitalReadFast(LED_BUILTIN));
}

uint32_t StepperWrapper::ISRgeneric(uint32_t t0, uint8_t opCode) {
  if (ISRcode > 0) {
    return 0;
  }
  uint32_t t1 = millis();
  if (t0 == 0 || t1 - t0 > debounceMillis) {
    ISRcode = opCode;
  }
  return t1;
}

void StepperWrapper::ISRsoftStop() {
  static uint32_t t0  = 0;
  t0 = StepperWrapper::ISRgeneric(t0,'x');
}

void StepperWrapper::ISRhardStop() {
  static uint32_t t0  = 0;
  t0 = StepperWrapper::ISRgeneric(t0,'X');
}

void StepperWrapper::ISRforwards() {
  static uint32_t t0  = 0;
  t0 = StepperWrapper::ISRgeneric(t0,'F');
}

void StepperWrapper::ISRbackwards() {
  static uint32_t t0  = 0;
  t0 = StepperWrapper::ISRgeneric(t0,'B');
}

void StepperWrapper::ISRzero() {
  static uint32_t t0  = 0;
  t0 = StepperWrapper::ISRgeneric(t0,'Z');
}

void StepperWrapper::ISRpos1() { StepperWrapper::ISRposN(1); }
void StepperWrapper::ISRpos2() { StepperWrapper::ISRposN(2); }
void StepperWrapper::ISRpos3() { StepperWrapper::ISRposN(3); }
void StepperWrapper::ISRpos4() { StepperWrapper::ISRposN(4); }
void StepperWrapper::ISRpos5() { StepperWrapper::ISRposN(5); }
void StepperWrapper::ISRpos6() { StepperWrapper::ISRposN(6); }
void StepperWrapper::ISRpos7() { StepperWrapper::ISRposN(7); }
void StepperWrapper::ISRpos8() { StepperWrapper::ISRposN(8); }
void StepperWrapper::ISRpos9() { StepperWrapper::ISRposN(9); }
void StepperWrapper::ISRposN(uint8_t n) {
  static uint32_t t0[9]{0};
  t0[n-1] = StepperWrapper::ISRgeneric(t0[n-1],n);
}

void StepperWrapper::throwError(uint8_t ID) {
  DEBUG_PRINTF("ERROR %d\n",ID);
  if (errorID) {
    return;
  }
  enableDriver(false);
  errorID = ID;
  detachInterrupt(digitalPinToInterrupt(pin.Diag0));
  digitalWrite(pin.Error, HIGH);
  timerErrorBlink.begin(ISRblinkError, 250000);
}

void StepperWrapper::clearError() {
  errorID = 0;
  digitalWrite(pin.Error, LOW);
  timerErrorBlink.end();
  digitalWriteFast(LED_BUILTIN, LOW);
}

void StepperWrapper::RMS(uint16_t rms_current) {
  if (is2130) {
    rms_current = constrain(rms_current, 30, 850);
  } else if (is5160) {
    rms_current = constrain(rms_current, 48, 2000);
  }

  uint8_t iHold = get_ihold();
  set_rms_current(rms_current);
  set_ihold(iHold);
  DEBUG_PRINTF("RMS current set to %d mA\n",RMS());
}

uint16_t StepperWrapper::RMS() { return get_rms_current(); }

void StepperWrapper::holdRMS(uint16_t rms) {
  uint8_t CS = 31;
  if (rms==0) {
    set_ihold(0);
    set_freewheel(true);
  } else {
    set_freewheel(false);
    while (cs2rms(CS) > rms && CS > 0) {
      CS--;
    }
    set_ihold(CS);
  }
  DEBUG_PRINTF("Hold RMS current set to %d mA%s\n", rms,
               (rms) ? "" : " (freewheeling)");
}

uint16_t StepperWrapper::holdRMS() {
  uint8_t cs = get_ihold();
  if (get_freewheel() && cs == 0) {
    return 0;
  } else {
    return cs2rms(cs);
}
}

uint16_t StepperWrapper::getMicrosteps() { return _microsteps; }

void StepperWrapper::setMicrosteps(uint16_t ms) {
  ms = constrain(ms,1,_msRes);
  ms = pow(2,ceil(log(ms)/log(2)));
  ms = (ms==1) ? 0 : ms;

  set_microsteps(ms);
  _microsteps = get_microsteps();
  _microsteps = (_microsteps==0) ? 1 : _microsteps;

  _microstepDiv = (_msRes/_microsteps);
  DEBUG_PRINTF("Setting microstep resolution to 1/%d\n",_microsteps);
}

void StepperWrapper::setChopper(bool chopper) {
  DEBUG_PRINTF("Switching to %s chopper\n",(chopper) ? "voltage" : "PWM");
  set_en_pwm_mode(chopper);
  // if (chopper) {
  //   set_TPWMTHRS(speed2ticks(0));
  // } else {
  //   set_TPWMTHRS(speed2ticks(5));
  // }
}

bool StepperWrapper::getChopper() { return get_en_pwm_mode(); }

uint8_t StepperWrapper::getIOmode(uint8_t idx) {
  idx--;
  if (idx >= _nIO) {
    return 0;
  }
  return _ioMode[idx];
}

void StepperWrapper::setIOmode(uint8_t mode[], uint8_t l) {
  for (uint8_t idx = 1; idx <= l; idx++) {
    setIOmode(idx,mode[idx-1]);
}
}

void StepperWrapper::setIOmode(uint8_t idx, uint8_t mode) {
  DEBUG_PRINTF("Setting IO%d to mode %3d\n",idx,mode);
  idx--;
  if (idx >= _nIO) {
    return;
  }

  _ioMode[idx] = mode;

  detachInterrupt(digitalPinToInterrupt(pin.IO[idx]));
  switch (mode) {
    case 1:
      attachInput(idx, ISRpos1);
      break;
    case 2:
      attachInput(idx, ISRpos2);
      break;
    case 3:
      attachInput(idx, ISRpos3);
      break;
    case 4:
      attachInput(idx, ISRpos4);
      break;
    case 5:
      attachInput(idx, ISRpos5);
      break;
    case 6:
      attachInput(idx, ISRpos6);
      break;
    case 7:
      attachInput(idx, ISRpos7);
      break;
    case 8:
      attachInput(idx, ISRpos8);
      break;
    case 9:
      attachInput(idx, ISRpos9);
      break;
    case 'x':
      attachInput(idx, ISRsoftStop);
      break;
    case 'X':
      attachInput(idx, ISRhardStop);
      break;
    case 'F':
      attachInput(idx, ISRforwards);
      break;
    case 'B':
      attachInput(idx, ISRbackwards);
      break;
    case 'Z':
      attachInput(idx, ISRzero);
      break;
    case 'J':
    case 'L':
      pinMode(pin.IO[idx], _ioResistor[idx]);
      break;
    case 'a':
      _ioMode[idx] = (idx==0 && PCBrev >= 14) ? mode : 0;
      break;
    case 'b':
      _ioMode[idx] = (idx==1 && PCBrev >= 14) ? mode : 0;
      break;
    default:
      _ioMode[idx] = 0;
      pinMode(pin.IO[idx], INPUT);
  }
  initEncoder();
}

void StepperWrapper::attachInput(uint8_t idx, void (*userFunc)(void)) {
  pinMode(pin.IO[idx], _ioResistor[idx]);
  uint8_t direction = (_ioResistor[idx]==INPUT_PULLUP) ? FALLING : RISING;
  attachInterrupt(digitalPinToInterrupt(pin.IO[idx]), userFunc, direction);
}

void StepperWrapper::toggleISRlimit(int8_t direction) {
  for (uint8_t idx = 0; idx < sizeof(_ioMode); idx++ ) {
    if (_ioMode[idx] == 'J' || _ioMode[idx] == 'L') {
      if (_ioMode[idx] - 75 == direction) {
        attachInput(idx, ISRhardStop);
      } else {
        detachInterrupt(digitalPinToInterrupt(pin.IO[idx]));
    }
  }
}
}

bool StepperWrapper::atLimit(int8_t direction) {
  for (uint8_t idx = 0; idx < sizeof(_ioMode); idx++ ) {
    if (_ioMode[idx] - 75 == direction) {
      if (digitalRead(pin.IO[idx]) ^ (_ioResistor[idx] == INPUT_PULLUP)) {
        return true;
    }
  }
  }
  return false;
}

uint8_t StepperWrapper::getIOresistor(uint8_t idx) {
  idx--;
  if (idx >= _nIO) {
    return 0;
  }
  uint8_t output = _ioResistor[idx];
  if (output > 0) {
    output--;
  }
  return output;
}

void StepperWrapper::setIOresistor(uint8_t r[], uint8_t l) {
  for (uint8_t idx = 1; idx <= l; idx++) {
    setIOresistor(idx,r[idx-1]);
}
}

void StepperWrapper::setIOresistor(uint8_t idx, uint8_t r) {
  DEBUG_PRINTF("Setting IO%d to %s\n", idx,
               (r == 0 ? "floating" : (r == 1 ? "pull-up" : "pull-down")));
  idx--;
  if (idx >= _nIO || r > 2) {
    return;
  }
  if (r > 0) {
    r++;
  }
  _ioResistor[idx] = r;
  if (_ioMode[idx] > 0) {
    setIOmode(idx+1, _ioMode[idx]);
}
}

void StepperWrapper::rotate() { this->rotate(1); }

int32_t StepperWrapper::readPosition() {
  if (!this->useSD) {
    return 0;
  }
  int32_t pos;
  this->filePos.rewind();
  int count = this->filePos.read((uint8_t*) &pos, 4);
  if (count != 4) {
    return 0;
  }
  DEBUG_PRINTF("Loading current position from SD card: %d\n",pos);
  return pos;
}

bool StepperWrapper::writePosition(int32_t pos) {
  static int32_t lastPos = readPosition();
  if (!this->useSD || pos == lastPos) {
    return false;
  }
  this->filePos.rewind();
  size_t count = this->filePos.write((uint8_t*) &_microPosition, 4);
  this->filePos.sync();
  if (count == 4) {
    lastPos = pos;
    DEBUG_PRINTF("Storing current position to SD card: %d\n\n",pos);
    return true;
  } else {
    return false;
}
}

void StepperWrapper::go2target(uint8_t id) {
  if (this->isRunning() || this->position() == p.target[id]) {
    return;
  }
  DEBUG_PRINTF("Predefined target #%d\n",id);
  this->a((p.aTarget[id]>0) ? p.aTarget[id] : p.a);
  this->vMax((p.vMaxTarget[id]>0) ? p.vMaxTarget[id] : p.vMax);
  this->position(p.target[id]);
}

void StepperWrapper::initEncoder() {
  // check if all encoder lines are assigned
  bool useEncoder = _ioMode[0]=='a' && _ioMode[1]=='b';

  // decide what to do
  if (useEncoder == (_enc != nullptr)) {
    return;
  } else if (useEncoder) {
    DEBUG_PRINTLN("Enabling hardware quadrature encoder");
    _enc = new QuadDecode<2>;
    _enc->setup();
    _enc->start();
    _enc->zeroFTM();
  } else {
    DEBUG_PRINTLN("Disabling hardware quadrature encoder");
    _enc->reset();
    _enc = nullptr;
  }
}

int32_t StepperWrapper::encoderPosition() {
  return (_enc == nullptr) ? 0 : _enc->calcPosn();
}

void StepperWrapper::resetEncoderPosition() {
  if (_enc != nullptr) {
    _enc->zeroFTM();
}
}

void StepperWrapper::moveSteps(int32_t steps) {
  moveMicroSteps(steps * _msRes);
}

int32_t StepperWrapper::position() { return microPosition() / _msRes; }

void StepperWrapper::position(int32_t target) {
  microPosition(target * _msRes);
}

void StepperWrapper::stepsPerRevolution(uint16_t steps) {
  _stepsPerRevolution = steps;
}

uint16_t StepperWrapper::stepsPerRevolution() { return _stepsPerRevolution; }

void StepperWrapper::countsPerRevolution(uint16_t counts) {
  _countsPerRevolution = counts;
}

uint16_t StepperWrapper::countsPerRevolution() { return _countsPerRevolution; }

uint32_t StepperWrapper::speed2ticks(float stepsPerSec) {
  static float vDiv = (fCLK / 256.0);
  return vDiv / stepsPerSec;
}

float StepperWrapper::ticks2speed(uint32_t ticks) {
  static float vDiv = (fCLK / 256.0);
  return vDiv / ticks;
}
