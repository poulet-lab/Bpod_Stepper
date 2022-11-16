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
#include <avr/io.h>
#include <avr/interrupt.h>
#include "EEstore.h"                        // Import EEstore library
#include "StepperWrapper.h"
#include "SerialDebug.h"



extern ArCOM Serial1COM;
extern ArCOM usbCOM;

const uint8_t PCBrev      = StepperWrapper::idPCB();
const teensyPins pin      = StepperWrapper::getPins(PCBrev);
const uint8_t vDriver     = StepperWrapper::idDriver();
volatile uint8_t errorID  = 0;
volatile uint8_t ISRcode  = 0;
IntervalTimer timerErrorBlink;

StepperWrapper::StepperWrapper() {
  _nIO = (PCBrev<12) ? 2 : 6;              // set number of IO pins

  pinMode(pin.En, OUTPUT);
  pinMode(LED_BUILTIN, OUTPUT);
  if (PCBrev>=14) {
    pinMode(pin.VIO, OUTPUT);

    pinMode(pin.Diag0, INPUT_PULLUP);
    pinMode(pin.Diag1, INPUT_PULLUP);
    //attachInterrupt(digitalPinToInterrupt(pin.Diag0), ISRdiag0, RISING);
    //attachInterrupt(digitalPinToInterrupt(pin.Diag1), ISRdiag1, RISING);

    // Detect presence of motor power supply
    pinMode(pin.VM, INPUT);
    attachInterrupt(digitalPinToInterrupt(pin.VM), ISRchangeVM, CHANGE);
    if (!digitalRead(pin.VM))
      throwError(1);

    // Throw error if no supported driver found
    if (vDriver == 0)
      throwError(1);
  }

  // Initialize SDcard, load position
  this->useSD = this->SD.begin(SdioConfig(FIFO_SDIO));
  DEBUG_PRINTF("SD card %sdetected.%s\n",(this->useSD) ? "" : "NOT ", (this->useSD) ? " Initalizing." : "");
  if (this->useSD)
    this->filePos.open("position.bin", O_RDWR | O_CREAT);

  TimerStream.priority(255);                // lowest priority for stream

  enableDriver(false);                      // disable driver for now
  powerDriver(true);                        // power the driver
}

void StepperWrapper::ISRstream() {
  static uint32_t data[3];

  data[0] = millis();
  switch (vDriver) {
    case 0x11:
      {
        TMC2130Stepper* driver = get2130();
        data[1] = driver->DRV_STATUS();
        data[2] = driver->TSTEP();
        break;
      }
    case 0x30:
      {
        TMC5160Stepper* driver = get5160();
        data[1] = driver->DRV_STATUS();
        data[2] = driver->TSTEP();
      }
  }

  usbCOM.writeUint32Array(data,3);
}

bool StepperWrapper::getStream() {
  return StatusTimerStream;
}

void StepperWrapper::setStream(bool enable) {
  if (enable && !StatusTimerStream) {
    StatusTimerStream = TimerStream.begin(StepperWrapper::ISRstream, 50000);
    return;
  }
  else if (!enable && StatusTimerStream)
    TimerStream.end();
  StatusTimerStream = false;
}

void StepperWrapper::ISRdiag0() {
  if (errorID)
    return;
  uint8_t gstat = get2130()->GSTAT();
  if (bitRead(gstat,2))
    throwError(3);                          // undervoltage charge-pump
  else if (bitRead(gstat,1)) {
    uint32_t status = get2130()->DRV_STATUS();
    if (bitRead(status, 25))
      throwError(4);                        // overtemperature limit has been reached
    else if (bitRead(status, 26))
      throwError(5);                        // overtemperature pre-warning threshold is exceeded
    else if (bitRead(status, 27))
      throwError(6);                        // short to ground indicator phase A
    else if (bitRead(status, 28))
      throwError(7);                        // short to ground indicator phase B
  }
  // TODO
}

void StepperWrapper::ISRdiag1() {
  DEBUG_PRINTLN("Stall detected!");
}

void StepperWrapper::ISRchangeVM() {
  if (digitalRead(pin.VM))
    SCB_AIRCR = 0x05FA0004;                 // reset teensy
  else
    throwError(1);
}

void StepperWrapper::init(uint16_t rms_current) {
  if (vDriver == 0x11)
    init2130(rms_current);                  // initialize TMC2130
  else if (vDriver == 0x30)
    init5160(rms_current);                  // initialize TMC5160
}

void StepperWrapper::init2130(uint16_t rms_current) {
  DEBUG_PRINTLN("Initializing driver: TMC2130");

  _invertPinDir = true;
  StepperWrapper::setMicrosteps(256);

  TMC2130Stepper* driver = get2130();

  // configuration of DIAG pins & interrupts
  driver->GSTAT(0b111);                     // reset error flags

  driver->diag0_error(true);                // enable DIAG0 active on driver errors
  driver->diag1_stall(true);
  driver->diag0_int_pushpull(false);
  driver->diag1_pushpull(false);
  attachInterrupt(digitalPinToInterrupt(pin.Diag0), ISRdiag0, FALLING);
  attachInterrupt(digitalPinToInterrupt(pin.Diag1), ISRdiag1, FALLING);

  // RMS current
  driver->rms_current(rms_current,1);       // set motor current, standstill reduction disabled
  enableDriver(true);                       // activate motor outputs

  // StealthChop configuration
  driver->en_pwm_mode(1);                   // enable StealthChop
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

  driver->sgt(0);
  driver->sfilt(true);

  driver->TPWMTHRS(0);                      // Disable SpreadCycle chopper (use StealthChop only)

  driver->rms_current(rms_current);         // Set motor current
  driver->TPOWERDOWN(0);
  driver->iholddelay(0);                    // instant IHOLD
}

void StepperWrapper::init5160(uint16_t rms_current) {
  DEBUG_PRINTLN("Initializing driver: TMC5160");

  _invertPinDir = false;
  StepperWrapper::setMicrosteps(256);

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

  driver->rms_current(rms_current);         // Set motor current
  driver->TPOWERDOWN(0);
  driver->iholddelay(0);                    // instant IHOLD

  // TODO: StallGuard / CoolStep
  // driver->TCOOLTHRS(* 256);
  // driver->sgt(2);
  // driver->sg_stop(true);
  // driver->THIGH(1500 * 256);
  // driver->tbl(2);
  // driver->chm(0);
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
  DEBUG_PRINTF("Powering %s driver.\n",(power) ? "up" : "down");
  pinMode(pin.VIO, OUTPUT);
  if (PCBrev>=14) {
    digitalWrite(pin.VIO, power);
    delay(200);
  }
}

void StepperWrapper::enableDriver(bool enable) {
  DEBUG_PRINTF("%s driver: EN=%s\n",(enable)?"Enabling":"Disabling",
    (enable^_invertPinEn)?"HIGH":"LOW");
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
  if (x)
    return 13 + x;
  else {                          // revisions older than 1.4:
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
        return 13;
      else                        // ... while r1.2 does not
        return 12;
    } else                        // otherwise its r1.1
      return 11;
  }
}

uint8_t StepperWrapper::idDriver() {
  TMC2130Stepper* driver = get2130();
  if (!driver->test_connection())
    return driver->version();
  return 0;
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
  } else {              // major reorganisation with r1.2
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
  if (PCBrev >= 13) {   // corrected layout for hardware SPI with r1.3
    pin.Dir   =  5;
    pin.Step  =  6;
    pin.Sleep =  7;
    pin.Reset =  8;
    pin.CFG3  = 10;
    pin.CFG2  = 14;
    pin.En    = 12;
    pin.IO[3] = 15;     // IO4
  }
  if (PCBrev >= 14) {   // r1.4: support for DIAG pins, VIO control and VM monitoring
    pin.Diag0 = 24;
    pin.Diag1 = 25;
    pin.VIO   = 28;
    pin.VM    = 4 ;
  }
  if (PCBrev >= 15) {
    pin.IO[0] = 30;     // IO1 - watch out, zero indexing!
    pin.IO[1] = 29;     // IO2
  }
  return pin;
}

TMC2130Stepper* StepperWrapper::get2130() {
  static bool initialized = false;
  static TMC2130Stepper* driver;

  if (!initialized) {
    powerDriver(true);
    if (PCBrev<13)
      driver = new TMC2130Stepper(pin.CFG3, 0.110, pin.CFG1, pin.Reset, pin.CFG2);
    else {
      driver = new TMC2130Stepper(pin.CFG3, 0.110);
      SPI.setMISO(pin.Reset);
      SPI.setMOSI(pin.CFG1);
      SPI.setSCK(pin.CFG2);
      SPI.begin();
    }
    delay(50);
    driver->begin();
    initialized = true;
  }

  return driver;
}

TMC5160Stepper* StepperWrapper::get5160() {
  static bool initialized = false;
  static TMC5160Stepper* driver;

  if (!initialized) {
    powerDriver(true);
    if (PCBrev<13)
      driver = new TMC5160Stepper(pin.CFG3, 0.075, pin.CFG1, pin.Reset, pin.CFG2);
    else {
      driver = new TMC5160Stepper(pin.CFG3, 0.075);
      SPI.setMISO(pin.Reset);
      SPI.setMOSI(pin.CFG1);
      SPI.setSCK(pin.CFG2);
      SPI.begin();
    }
    delay(50);
    driver->begin();
    initialized = true;
  }

  return driver;
}

bool StepperWrapper::SDmode() {
  bool sd_mode = true;
  if (vDriver == 0x30) {
    TMC5160Stepper* driver = get5160();
    if (!driver->test_connection())
      sd_mode = driver->sd_mode();
  }
  DEBUG_PRINTF("Driver configured for %s.\n",(sd_mode) ? "step/dir interface" : "internal ramp generator");
  return sd_mode;
}

void StepperWrapper::ISRblinkError() {
  digitalWriteFast(LED_BUILTIN, !digitalReadFast(LED_BUILTIN));
}

uint32_t StepperWrapper::ISRgeneric(uint32_t t0, uint8_t opCode) {
  if (ISRcode>0)
    return 0;
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
  if (errorID)
    return;
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
  uint16_t hold_rms = holdRMS();
  switch (vDriver) {
    case 0x11:
      rms_current = constrain(rms_current, 30, 850);
      get2130()->rms_current(rms_current);
      holdRMS(hold_rms);
      break;
    case 0x30:
      rms_current = constrain(rms_current, 48, 2000);
      get5160()->rms_current(rms_current);
      holdRMS(hold_rms);
      break;
    default:
      return;
  }
  DEBUG_PRINTF("RMS current set to %d mA\n",RMS());
}

uint16_t StepperWrapper::RMS() {
  switch (vDriver) {
    case 0x11:
      return get2130()->rms_current();
    case 0x30:
      return get5160()->rms_current();
    default:
      return 0;
  }
}

void StepperWrapper::holdRMS(uint16_t rms) {
  uint8_t CS = 31;

  switch (vDriver) {
    case 0x11:
      if (rms==0) {
        get2130()->ihold(0);
        get2130()->freewheel(0x01);
        return;
      } else {
        get2130()->freewheel(0x00);
        while (rms < get2130()->cs2rms(CS) && CS > 0)
          CS--;
        get2130()->ihold(CS);
        break;
      }
    case 0x30:
      if (rms==0) {
        get5160()->ihold(0);
        get5160()->freewheel(0x01);
        return;
      } else {
        get5160()->freewheel(0x00);
        while (rms < get5160()->cs2rms(CS) && CS > 0)
          CS--;
        get5160()->ihold(CS);
        break;
      }
    default:
      return;
  }

}

uint16_t StepperWrapper::holdRMS() {
  uint8_t cs;
  uint16_t rms;
  bool freewheel;
  switch (vDriver) {
    case 0x11:
      cs = get2130()->ihold();
      rms = get2130()->cs2rms(cs);
      freewheel = get2130()->freewheel() == 0x01;
      break;
    case 0x30:
      cs = get5160()->ihold();
      rms = get5160()->cs2rms(cs);
      freewheel = get5160()->freewheel() == 0x01;
      break;
    default:
      return 0;
  }
  if (freewheel && cs == 0)
    return 0;
  else
    return rms;
}

uint16_t StepperWrapper::getMicrosteps() {
  return _microsteps;
}

void StepperWrapper::setMicrosteps(uint16_t ms) {
  ms = constrain(ms,1,_msRes);
  ms = pow(2,ceil(log(ms)/log(2)));
  ms = (ms==1) ? 0 : ms;

  switch (vDriver) {
    case 0x11:  // TMC2130
      get2130()->microsteps(ms);
      _microsteps = get2130()->microsteps();
      break;
    case 0x30:  // TMC5160
      get5160()->microsteps(ms);
      _microsteps = get5160()->microsteps();
      break;
  }
  _microsteps = (_microsteps==0) ? 1 : _microsteps;
  _microstepDiv = (_msRes/_microsteps);
  DEBUG_PRINTF("Setting microstep resolution to 1/%d\n",_microsteps);
}

void StepperWrapper::setChopper(bool chopper) {
  DEBUG_PRINTF("Switching to %s chopper\n",(chopper) ? "voltage" : "PWM");
  switch (vDriver) {
    case 0x11:
      get2130()->en_pwm_mode(chopper);
      return;
    case 0x30:
      get5160()->en_pwm_mode(chopper);
      return;
  }
}

bool StepperWrapper::getChopper() {
  switch (vDriver) {
    case 0x11:
      return get2130()->en_pwm_mode();
    case 0x30:
      return get5160()->en_pwm_mode();
    default:
      return 0;
  }
}

uint8_t StepperWrapper::getIOmode(uint8_t idx) {
  idx--;
  if (idx>=_nIO)
    return 0;
  return _ioMode[idx];
}

void StepperWrapper::setIOmode(uint8_t mode[], uint8_t l) {
  for (uint8_t idx = 1; idx <= l; idx++)
    setIOmode(idx,mode[idx-1]);
}

void StepperWrapper::setIOmode(uint8_t idx, uint8_t mode) {
  DEBUG_PRINTF("Setting IO%d to mode %3d\n",idx,mode);
  idx--;
  if (idx>=_nIO)
    return;

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
      if (_ioMode[idx] - 75 == direction)
        attachInput(idx, ISRhardStop);
      else
        detachInterrupt(digitalPinToInterrupt(pin.IO[idx]));
    }
  }
}

bool StepperWrapper::atLimit(int8_t direction) {
  for (uint8_t idx = 0; idx < sizeof(_ioMode); idx++ ) {
    if (_ioMode[idx] - 75 == direction) {
      if (digitalRead(pin.IO[idx]) ^ (_ioResistor[idx]==INPUT_PULLUP))
        return true;
    }
  }
  return false;
}

uint8_t StepperWrapper::getIOresistor(uint8_t idx) {
  idx--;
  if (idx>=_nIO)
    return 0;
  uint8_t output = _ioResistor[idx];
  if (output > 0)
    output--;
  return output;
}

void StepperWrapper::setIOresistor(uint8_t r[], uint8_t l) {
  for (uint8_t idx = 1; idx <= l; idx++)
    setIOresistor(idx,r[idx-1]);
}

void StepperWrapper::setIOresistor(uint8_t idx, uint8_t r) {
  DEBUG_PRINTF("Setting IO%d to %s\n",idx,(r==0?"floating":(r==1?"pull-up":"pull-down")));
  idx--;
  if (idx>=_nIO || r > 2)
    return;
  if (r > 0)
    r++;
  _ioResistor[idx] = r;
  if (_ioMode[idx]>0)
    setIOmode(idx+1, _ioMode[idx]);
}

void StepperWrapper::rotate() {
  this->rotate(1);
}

int32_t StepperWrapper::readPosition() {
  if (!this->useSD)
    return 0;
  int32_t pos;
  this->filePos.rewind();
  int count = this->filePos.read((uint8_t*) &pos, 4);
  if (count != 4)
    return 0;
  DEBUG_PRINTF("Loading current position from SD card: %d\n",pos);
  return pos;
}

bool StepperWrapper::writePosition(int32_t pos) {
  static int32_t lastPos = readPosition();
  if (!this->useSD || pos == lastPos)
    return false;
  this->filePos.rewind();
  size_t count = this->filePos.write((uint8_t*) &_microPosition, 4);
  this->filePos.sync();
  if (count == 4) {
    lastPos = pos;
    DEBUG_PRINTF("Storing current position to SD card: %d\n\n",pos);
    return true;
  } else
    return false;
}

void StepperWrapper::go2target(uint8_t id) {
  if (this->isRunning() || this->position() == p.target[id])
    return;
  DEBUG_PRINTF("Predefined target #%d\n",id);
  this->a((p.aTarget[id]>0) ? p.aTarget[id] : p.a);
  this->vMax((p.vMaxTarget[id]>0) ? p.vMaxTarget[id] : p.vMax);
  this->position(p.target[id]);
}

void StepperWrapper::initEncoder() {
  // check if all encoder lines are assigned
  bool useEncoder = _ioMode[0]=='a' && _ioMode[1]=='b';

  // decide what to do
  if (useEncoder == (_enc != nullptr))
    return;
  else if (useEncoder) {
    DEBUG_PRINTLN("Enabling hardware quadrature encoder");
    _enc = new QuadDecode<2>;
    _enc->setup();
    _enc->start();
    _enc->zeroFTM();
  }
  else {
    DEBUG_PRINTLN("Disabling hardware quadrature encoder");
    _enc->reset();
    _enc = nullptr;
  }
}

int32_t StepperWrapper::encoderPosition() {
  return (_enc == nullptr) ? 0 : _enc->calcPosn();
}

void StepperWrapper::resetEncoderPosition() {
  if (_enc != nullptr)
    _enc->zeroFTM();
}

void StepperWrapper::moveSteps(int32_t steps) {
  moveMicroSteps(steps * _msRes);
}

int32_t StepperWrapper::position() {
  return microPosition() / _msRes;
}

void StepperWrapper::position(int32_t target) {
  microPosition(target * _msRes);
}

void StepperWrapper::stepsPerRevolution(uint16_t steps) {
  _stepsPerRevolution = 200;
}

uint16_t StepperWrapper::stepsPerRevolution() {
  return _stepsPerRevolution;
}

void StepperWrapper::countsPerRevolution(uint16_t counts) {
  _countsPerRevolution = counts;
}

uint16_t StepperWrapper::countsPerRevolution() {
  return _countsPerRevolution;
}
