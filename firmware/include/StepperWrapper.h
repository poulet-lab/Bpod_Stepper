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


_______________________________________________________________________________
*/

#pragma once
#include <ArCOM.h>          // https://github.com/bimac/ArCOM
#include <Arduino.h>
#include <QuadDecode_def.h> // https://github.com/bimac/Teensy3x_QuadDecode
#include <SD.h>             // https://github.com/PaulStoffregen/SD
#include <TMCStepper.h>     // https://github.com/teemuatlut/TMCStepper
#include <TeensyStep.h>     // https://github.com/luni64/TeensyStep
#include "EEstoreStruct.h"

#define STEPPER_ERROR___NO_DRIVER_FOUND 1
#define STEPPER_ERROR___VM_UNPLUGGED 2
#define STEPPER_ERROR___UNDER_VOLTAGE 3
#define STEPPER_ERROR___OVER_TEMPERATURE 4
#define STEPPER_ERROR___OVER_TEMPERATURE_PRE 5
#define STEPPER_ERROR___SHORT_TO_GND_PHASE_A 6
#define STEPPER_ERROR___SHORT_TO_GND_PHASE_B 7

struct teensyPins {
  uint8_t Dir;
  uint8_t Step;
  uint8_t Sleep;
  uint8_t Reset;
  uint8_t CFG1;
  uint8_t CFG2;
  uint8_t CFG3;
  uint8_t En;
  uint8_t Diag0;
  uint8_t Diag1;
  uint8_t VIO;
  uint8_t VM;
  uint8_t Error;
  uint8_t IO[6]{0};
};

extern volatile uint8_t errorID; // error ID (set through interrupt sub-routine)
extern volatile uint8_t ISRcode; // opCode (set through interrupt sub-routine)
extern storageVars p;

struct StepperWrapper {
  StepperWrapper(); // constructor

  static TMC2130Stepper *const TMC2130;
  static TMC5160Stepper *const TMC5160;

  static const uint8_t vDriver;              // driver version
  static const uint8_t PCBrev;               // PCB revision
  static const teensyPins pin;               // pin numbers
  static const bool is2130;                  // bool: is TMC2130?
  static const bool is5160;                  // bool: is TMC5160?
  static const uint32_t fCLK;                // internal clock frequency
  static const char *driverName;             // name of motor driver
  template <class T> static T *const driver; // TMCstepper object

  static const uint8_t _nIO;              // number of IO pins
  static constexpr uint16_t _msRes = 256; // micro-stepping resolution

  template <class T>
  static T *getDriver(float, uint8_t); // return pointer to driver (or nullptr)

  virtual void init(); // initialize stepper class

  // templated members
  template <class T> void init(T *);

  virtual void setMicrosteps(uint16_t ms); // set microstepping resolution
  uint16_t getMicrosteps();                // get microstepping resolution

  static void blinkenlights(); // extra fancy LED sequence to say hi
  static uint8_t idDriver();   // identify TMC stepper driver
  static bool SDmode();        // are we using STEP/DIR mode?

  void SGautoTune();

  virtual float a() = 0;       // get acceleration (full steps / s^2)
  virtual void a(float a) = 0; // set acceleration (full steps / s^2)
  virtual void softStop() = 0; // stop after defined slow-down
  virtual void hardStop() = 0; // stop as fast as possible (risk of step-loss)
  virtual void
  moveMicroSteps(int32_t steps) = 0; // move to relative position (micro-steps)
  virtual bool isRunning() = 0;      // is the motor currently running?
  virtual int32_t microPosition() = 0; // get current position (micro-steps)
  virtual void
  microPosition(int32_t target) = 0; // go to absolute position (micro-steps)
  virtual void
  setPosition(int32_t pos) = 0; // set position without moving (microsteps)
  virtual void rotate(int8_t direction) = 0; // initiate rotation
  virtual float vMax() = 0;       // get peak velocity (full steps / s)
  virtual void vMax(float v) = 0; // set peak velocity (full steps / s)

  void rotate();                    // start rotation
  void moveSteps(int32_t steps);    // move to relative position (full-steps)
  void go2target(uint8_t id);       // move to predefined target
  void position(int32_t target);    // move to absolute position (full-steps)
  int32_t position();               // get absolute position (full-steps)
  uint16_t RMS();                   // get RMS current
  void RMS(uint16_t rms_current);   // set RMS current
  void holdRMS(uint16_t val);       // set hold current
  uint16_t holdRMS();               // get hold current
  void freewheel(uint8_t val);      // set passive braking mode
  uint8_t freewheel();              // get passive braking mode
  void setChopper(uint8_t chopper); // set chopper mode
  uint8_t getChopper();             // get chopper mode
  void setIOmode(uint8_t mode[6], uint8_t l); // set IO mode (all IO ports)
  void setIOmode(uint8_t idx, uint8_t role);  // set IO mode (specific IO port)
  uint8_t getIOmode(uint8_t idx);             // get IO mode (specific IO port)
  void setIOresistor(uint8_t r[6],
                     uint8_t l);      // set input resistor (all IO ports)
  void setIOresistor(uint8_t idx,
                     uint8_t r);      // set input resistor (specific IO port)
  uint8_t getIOresistor(uint8_t idx); // get input resistor (specific IO port)
  void setStream(bool enable);        // enable/disable live streaming
  bool getStream();                   // get status of live stream
  int32_t readPosition();             // read _microPosition from SD card
  int32_t encoderPosition();          // get encoder position
  void resetEncoderPosition();        // reset encoder position to zero
  void stepsPerRevolution(uint16_t steps);  // set steps per revolution
  uint16_t stepsPerRevolution();            // get steps per revolution
  void countsPerRevolution(uint16_t steps); // set encoder counts per revolution
  uint16_t countsPerRevolution();           // get encoder counts per revolution

protected:
  static void powerDriver(bool);   // supply VIO to driver board?
  static void enableDriver(bool);  // enable driver board via EN pin?
  static void throwError(uint8_t); // throw error with specified numeric ID
  static void clearError();        // clear error condition

  volatile int32_t _microPosition;

  ArCOM *_COM;
  void toggleISRlimit(int8_t direction);
  bool atLimit(int8_t direction);
  uint16_t _microsteps = 1;
  uint16_t _microstepDiv = 256;
  uint16_t _stepsPerRevolution = 200;
  uint16_t _countsPerRevolution = 32768;
  static constexpr bool _invertPinEn = true;
  bool _invertPinDir = false;
  bool writePosition(int32_t pos); // Write _microPosition to SD card
  uint32_t speed2ticks(float stepsPerSec);
  float ticks2speed(uint32_t ticks);

private:
  // storageVars p; // EEPROM store

  // interrupt service routines
  static void ISRchangeVM();   // called when VM was (dis)connected
  static void ISRdiag0();      // called when diag0 changes
  static void ISRdiag1();      // called when diag1 changes
  static void ISRblinkError(); // blink lights ad infinitum
  static void ISRstream();
  static uint32_t ISRgeneric(uint32_t, uint8_t); // IO: ISR debounce
  static void ISRsoftStop();                     // IO: soft stop
  static void ISRhardStop();                     // IO: emergency stop
  static void ISRforwards();                     // IO: start rotating forwards
  static void ISRbackwards();                    // IO: start rotating backwards
  static void ISRzero();                         // IO: reset position to zero
  static void ISRpos1();          // IO: go to predefined position 1
  static void ISRpos2();          // IO: go to predefined position 2
  static void ISRpos3();          // IO: go to predefined position 3
  static void ISRpos4();          // IO: go to predefined position 4
  static void ISRpos5();          // IO: go to predefined position 5
  static void ISRpos6();          // IO: go to predefined position 6
  static void ISRpos7();          // IO: go to predefined position 7
  static void ISRpos8();          // IO: go to predefined position 8
  static void ISRpos9();          // IO: go to predefined position 9
  static void ISRposN(uint8_t n); // IO: go to predefined position N

  // stream timer
  IntervalTimer TimerStream;
  bool StatusTimerStream = false;

  // related to SD card
  bool useSD = false; // Bool: SD available?
  SdFs SD;            // SD file system class
  FsFile filePos;     // File for storing current position

  void attachInput(uint8_t, void (*)(void));
  static const uint32_t debounceMillis = 50; // duration for input debounce [ms]
  uint8_t _ioMode[6]{0};
  uint8_t _ioResistor[6]{0}; // input resistor for IO pins (0 = no resistor, 1 =
                             // pullup, 2 = pulldown)

  // quadrature encoder
  void initEncoder();
  QuadDecode<2> *_enc = nullptr;

  // lambda functions
  std::function<uint16_t(uint8_t)> cs2rms;
  std::function<uint16_t()> sg_result;
  std::function<bool()> get_chm;
  std::function<void(bool)> set_chm;
  std::function<bool()> get_en_pwm_mode;
  std::function<void(bool)> set_en_pwm_mode;
  std::function<uint8_t()> get_ihold;
  std::function<void(uint8_t)> set_ihold;
  std::function<uint8_t()> get_freewheel;
  std::function<void(uint8_t)> set_freewheel;
  std::function<uint16_t()> get_microsteps;
  std::function<void(uint16_t)> set_microsteps;
  std::function<uint16_t()> get_rms_current;
  std::function<void(uint16_t)> set_rms_current;
  std::function<bool()> get_sfilt;
  std::function<void(bool)> set_sfilt;
  std::function<uint8_t()> get_sgt;
  std::function<void(int8_t)> set_sgt;
  std::function<uint32_t()> get_TPWMTHRS;
  std::function<void(uint32_t)> set_TPWMTHRS;
};

class StepperWrapper_TeensyStep : public StepperWrapper {
public:
  StepperWrapper_TeensyStep();

  void init();
  void setMicrosteps(uint16_t);

  float a();
  void a(float);
  void hardStop();
  void moveMicroSteps(int32_t);
  bool isRunning();
  int32_t microPosition();
  void microPosition(int32_t);
  void setPosition(int32_t);
  void rotate(int8_t);
  void softStop();
  float vMax();
  void vMax(float);

private:
  uint32_t _aMu;
  int32_t _vMaxMu;
  float _a;
  float _vMax;
  Stepper *_motor;
  StepControl *_stepControl;
  RotateControl *_rotateControl;
  void updateMicroPosition();
  static void CBstop();
  IntervalTimer postStopTimer;
  static void PostStop();
};

class StepperWrapper_MotionControl : public StepperWrapper {
public:
  StepperWrapper_MotionControl();

  void init();

  float a();
  void a(float);
  void hardStop();
  bool isRunning();
  void moveMicroSteps(int32_t);
  int32_t microPosition();
  void microPosition(int32_t);
  void setPosition(int32_t);
  void rotate(int8_t);
  void softStop();
  float vMax();
  void vMax(float);

private:
  TMC5160Stepper *_driver;
  static constexpr float factA =
      (float)(1ul << 24) / (12E6 * 12E6 / (512.0 * 256.0));
  static constexpr float factV = (float)(1ul << 24) / (12E6);
};
