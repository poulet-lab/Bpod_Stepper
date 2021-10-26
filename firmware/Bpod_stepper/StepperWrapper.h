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


#ifndef StepperWrapper_h
#define StepperWrapper_h


class TMC5160Stepper;                           // forward declaration of TMC5160Stepper
class SmoothStepper;                            // forward declaration of SmoothStepper

struct teensyPins {
  uint8_t Dir;
  uint8_t Step;
  uint8_t Sleep;
  uint8_t Reset;
  uint8_t CFG1;
  uint8_t CFG2;
  uint8_t CFG3;
  uint8_t En;
  uint8_t IO1;
  uint8_t IO2;
  uint8_t IO3;
  uint8_t IO4;
  uint8_t IO5;
  uint8_t IO6;
  uint8_t Diag0;
  uint8_t Diag1;
  uint8_t VIO;
  uint8_t VM;
  uint8_t Error;
};

extern const float PCBrev;                      // PCB revision
extern const teensyPins pin;                    // pin numbers

class StepperWrapper
{
  public:
    StepperWrapper();                           // constructor

    static void powerUp();                      // called when VM was connected
    static void blinkError();                   // blink lights ad infinitum
    static void blinkenlights();                // extra fancy LED sequence to say hi
    static float idPCB();                       // identify PCB revision
    static bool SDmode();                       // are we using STEP/DIR mode?
    static teensyPins getPins(float PCBrev);
    static constexpr uint8_t errorPin = 33;     // pin for error interrupt
    static void powerDriver(bool power);        // supply VIO to driver board?
    static void enableDriver(bool enable);      // enable driver board via EN pin?

    uint8_t getErrorID() const;                 // return private member _errorID
    bool getTMC5160() const;                    // return _TMC5160

    uint16_t RMS();                             // get RMS current
    void RMS(uint16_t rms_current);             // set RMS current
    void throwError(uint8_t errorID);           // throw error with specified numeric ID

    virtual void init(uint16_t rms_current);    // initialize stepper class
    virtual void setMicrosteps(uint16_t ms);    // set microstepping resolution

    virtual float a() = 0;                      // get acceleration (full steps / s^2)
    virtual void a(float a) = 0;                // set acceleration (full steps / s^2)
    virtual void moveSteps(int32_t steps) = 0;  // move to relative position (full steps)
    virtual int32_t position() = 0;             // get current position (full steps)
    virtual void position(int32_t target) = 0;  // go to absolute position (full steps)
    virtual void resetPosition() = 0;           // reset position to zero
    virtual float vMax() = 0;                   // get peak velocity (full steps / s)
    virtual void vMax(float v) = 0;             // set peak velocity (full steps / s)

  protected:
    static constexpr float fCLK = 12E6;         // internal clock frequencz of TMC5160
    bool _TMC5160 = false;
    TMC5160Stepper* _driver;
    uint16_t _microsteps = 1;
    static constexpr bool _invertPinEn  = true;
    static constexpr bool _invertPinDir = false;

  private:
    void init2100();
    void init5160(uint16_t rms_current);
    static TMC5160Stepper* getDriver();
    volatile uint8_t _errorID = 0;
    static constexpr float _Rsense = 0.075;
    bool _hardwareSPI = false;
};



class StepperWrapper_SmoothStepper : public StepperWrapper
{
  public:
    StepperWrapper_SmoothStepper();             // constructor

    void init(uint16_t rms_current);
    void setMicrosteps(uint16_t ms);

    float a();
    void a(float a);
    void moveSteps(int32_t steps);
    int32_t position();
    void position(int32_t);
    void resetPosition();
    float vMax();
    void vMax(float v);

  private:
    SmoothStepper* _stepper;
};


class StepperWrapper_MotionControl : public StepperWrapper
{
  public:
    StepperWrapper_MotionControl();             // constructor

    void init(uint16_t rms_current);

    float a();
    void a(float a);
    void moveSteps(int32_t steps);
    int32_t position();
    void position(int32_t);
    void resetPosition();
    float vMax();
    void vMax(float v);

  private:
    static constexpr float factA = (float)(1ul<<24) / (fCLK * fCLK / (512.0 * 256.0));
    static constexpr float factV = (float)(1ul<<24) / (fCLK);
};
#endif
