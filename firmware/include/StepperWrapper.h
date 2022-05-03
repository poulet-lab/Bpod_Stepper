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


#pragma once

#include <TeensyStep.h>
#include <TMCStepper.h>
#include <SdFat.h>
#include <ArCOM.h>
#include <QuadDecode_def.h>
#include "SmoothStepper.h"
#include "EEstoreStruct.h"

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
  uint8_t IO[6] {0};
};

extern const float PCBrev;                      // PCB revision
extern const uint8_t vDriver;                   // version number of TMC stepper driver
extern const teensyPins pin;                    // pin numbers
extern volatile uint8_t errorID;                // error ID (set through interrupt sub-routine)
extern volatile uint8_t ISRcode;                // opCode (set through interrupt sub-routine)
extern storageVars p;

class StepperWrapper
{
  public:
    StepperWrapper();                           // constructor

    static void blinkenlights();                // extra fancy LED sequence to say hi
    static float idPCB();                       // identify PCB revision
    static uint8_t idDriver();                  // identify TMC stepper driver
    static bool SDmode();                       // are we using STEP/DIR mode?
    static teensyPins getPins(float PCBrev);

    uint16_t RMS();                             // get RMS current
    void RMS(uint16_t rms_current);             // set RMS current

    virtual void init(uint16_t rms_current);    // initialize stepper class
    virtual void setMicrosteps(uint16_t ms);    // set microstepping resolution
    uint16_t getMicrosteps();                   // get microstepping resolution

    virtual float a() = 0;                      // get acceleration (full steps / s^2)
    virtual void a(float a) = 0;                // set acceleration (full steps / s^2)
    virtual void hardStop() = 0;                // stop without slowing down
    virtual void moveSteps(int32_t steps) = 0;  // move to relative position (full steps)
    virtual bool isRunning() = 0;               // is the motor currently running?
    virtual int32_t position() = 0;             // get current position (full steps)
    virtual void position(int32_t target) = 0;  // go to absolute position (full steps)
    virtual void setPosition(int32_t pos) = 0;  // set position without moving (microsteps)
    virtual void rotate(int8_t direction) = 0;  // initiate rotation
    virtual void softStop() = 0;                // stop after slowing down
    virtual float vMax() = 0;                   // get peak velocity (full steps / s)
    virtual void vMax(float v) = 0;             // set peak velocity (full steps / s)

    void go2target(uint8_t id);
    void rotate();
    void setChopper(bool chopper);
    bool getChopper();
    void setIOmode(uint8_t mode[6], uint8_t l); // set IO mode (all IO ports)
    void setIOmode(uint8_t idx, uint8_t role);  // set IO mode (specific IO port)
    uint8_t getIOmode(uint8_t idx);             // get IO mode (specific IO port)
    void setIOresistor(uint8_t r[6], uint8_t l);// set input resistor (all IO ports)
    void setIOresistor(uint8_t idx, uint8_t r); // set input resistor (specific IO port)
    uint8_t getIOresistor(uint8_t idx);         // get input resistor (specific IO port)
    void setStream(bool enable);                // enable/disable live streaming of motor parameters
    bool getStream();                           // get status of live stream
    int32_t readPosition();                     // read _microPosition from SD card
    int32_t encoderPosition();
    void resetEncoderPosition();

  protected:
    volatile int32_t _microPosition;
    static TMC2130Stepper* get2130();
    static TMC5160Stepper* get5160();
    static void powerDriver(bool power);        // supply VIO to driver board?
    static void enableDriver(bool enable);      // enable driver board via EN pin?
    static void throwError(uint8_t errorID);    // throw error with specified numeric ID
    static void clearError();                   // clear error conditition
    void toggleISRlimit(int8_t direction);
    bool atLimit(int8_t direction);
    static constexpr float fCLK = 12E6;         // internal clock frequency of TMC5160
    uint16_t _microsteps = 1;
    static constexpr bool _invertPinEn  = true;
    bool _invertPinDir = false;
    ArCOM *_COM;
    bool writePosition(int32_t pos);            // Write _microPosition to SD card
    const uint16_t _msRes = (vDriver>0)?256:16; // highest microstepping resolution available

  private:
    static void ISRchangeVM();                  // called when VM was (dis)connected
    static void ISRdiag0();                     // called when diag0 changes
    static void ISRdiag1();                     // called when diag1 changes
    static void ISRblinkError();                // blink lights ad infinitum
    static void ISRstream();
    static uint32_t ISRgeneric(uint32_t t0, uint8_t opCode); // IO: ISR debouncer
    static void ISRsoftStop();                  // IO: soft stop
    static void ISRhardStop();                  // IO: emergency stop
    static void ISRforwards();                  // IO: start rotating forwards
    static void ISRbackwards();                 // IO: start totating backwards
    static void ISRzero();                      // IO: reset position to zero
    static void ISRpos1();                      // IO: go to predefined position 1
    static void ISRpos2();                      // IO: go to predefined position 2
    static void ISRpos3();                      // IO: go to predefined position 3
    static void ISRpos4();                      // IO: go to predefined position 4
    static void ISRpos5();                      // IO: go to predefined position 5
    static void ISRpos6();                      // IO: go to predefined position 6
    static void ISRpos7();                      // IO: go to predefined position 7
    static void ISRpos8();                      // IO: go to predefined position 8
    static void ISRpos9();                      // IO: go to predefined position 9
    static void ISRposN(uint8_t n);             // IO: go to predefined position N

    IntervalTimer TimerStream;
    bool StatusTimerStream = false;

    bool useSD = false;                         // Bool: SD available?
    SdFs SD;                                    // SD file system class
    FsFile filePos;                             // File for storing current position

    void attachInput(uint8_t idx, void (*userFunc)(void));
    void init2100();
    void init2130(uint16_t rms_current);
    void init5160(uint16_t rms_current);
    bool _hardwareSPI = false;
    uint8_t _nIO;                               // number of IO pins
    static const uint32_t debounceMillis = 500; // duration for input debounce [ms]
    uint8_t _ioMode[6] {0};
    uint8_t _ioResistor[6] {0};                 // input resistor for IO pins (0 = no resistor, 1 = pullup, 2 = pulldown)
    
    void initEncoder();
    QuadDecode<2>* _enc = nullptr; 
};



class StepperWrapper_SmoothStepper : public StepperWrapper
{
  public:
    StepperWrapper_SmoothStepper();             // constructor

    void init(uint16_t rms_current);
    void setMicrosteps(uint16_t ms);

    float a();
    void a(float a);
    void hardStop();
    bool isRunning();
    void moveSteps(int32_t steps);
    int32_t position();
    void position(int32_t target);
    void setPosition(int32_t pos);
    void rotate(int8_t direction);
    void softStop();
    float vMax();
    void vMax(float v);

  private:
    SmoothStepper* _stepper;
};


class StepperWrapper_TeensyStep : public StepperWrapper
{
  public:
    StepperWrapper_TeensyStep();                // constructor

    void init(uint16_t rms_current);
    void setMicrosteps(uint16_t ms);

    float a();
    void a(float a);
    void hardStop();
    void moveSteps(int32_t steps);
    void moveMicroSteps(int32_t steps);
    bool isRunning();
    int32_t position();
    void position(int32_t target);
    void setPosition(int32_t pos);
    void rotate(int8_t direction);
    void softStop();
    float vMax();
    void vMax(float v);

  private:
    uint32_t _aMu;
    int32_t _vMaxMu;
    float _a;
    float _vMax;
    Stepper* _motor;
    StepControl* _stepControl;
    RotateControl* _rotateControl;
    void updateMicroPosition();
    static void CBstop();
    IntervalTimer postStopTimer;
    static void PostStop();
};


class StepperWrapper_MotionControl : public StepperWrapper
{
  public:
    StepperWrapper_MotionControl();             // constructor

    void init(uint16_t rms_current);

    float a();
    void a(float a);
    void hardStop();
    bool isRunning();
    void moveSteps(int32_t steps);
    int32_t position();
    void position(int32_t target);
    void setPosition(int32_t pos);
    void rotate(int8_t direction);
    void softStop();
    float vMax();
    void vMax(float v);

  private:
    TMC5160Stepper* _driver;
    static constexpr float factA = (float)(1ul<<24) / (fCLK * fCLK / (512.0 * 256.0));
    static constexpr float factV = (float)(1ul<<24) / (fCLK);
};
