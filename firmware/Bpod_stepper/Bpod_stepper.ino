/*
Bpod_stepper
Copyright (C) 2020 Florian Rau

This program is free software: you can redistribute it and/or modify
it under the terms of the GNU General Public License as published by
the Free Software Foundation, version 3.

This program is distributed  WITHOUT ANY WARRANTY and without even the
implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
See the GNU General Public License for more details.

You should have received a copy of the GNU General Public License
along with this program.  If not, see <http://www.gnu.org/licenses/>.
*/

#include "ArCOM.h"         // Import serial communication wrapper
#include "SmoothStepper.h" // Import SmoothStepper library

// Module setup
ArCOM usbCOM(Serial);                                 // Wrap Serial (USB on Teensy 3.X)
ArCOM Serial1COM(Serial1);                            // Wrap Serial1 (UART on Arduino M0, Due + Teensy 3.X)
ArCOM *COM;                                           // Pointer to ArCOM object
char  moduleName[] = "Stepper";                       // Name of module for manual override UI and state machine assembler
const char* eventNames[] = {"Start", "Stop", "Limit"};

// Constants
#define FirmwareVersion 1
#define pinDir          2
#define pinStep         3
#define pinSleep        4
#define pinReset        5
#define pinCFG3         6
#define pinCFG2         7
#define pinCFG1         8
#define pinEnable       9
#define pinLimit1      10
#define pinLimit2      11
#define pinLED         13

// Variables
bool     lastDir     = true;                   // last movement direction: true = CW, false = CCW
bool     invertLimit = false;
uint8_t  pinLimit[]  = {pinLimit1, pinLimit2}; // Array of pins for limit switches
uint8_t  limitID;
uint8_t  direction;
uint8_t  nEventNames = sizeof(eventNames) / sizeof(char *);
uint8_t  opCode      = 0;
uint32_t stepsPerRev = 3200;                   // Steps per revolution (TMC2100 stealthChop mode = 3200)
int32_t  nSteps      = 0;
int32_t  alpha       = 0;
float    vMax        = (float) stepsPerRev/2;  // Set default speed
float    a           = (float) stepsPerRev;    // Set default acceleration

SmoothStepper stepper(pinStep, pinDir);

void setup()
{
  Serial1.begin(1312500);

  // Set CFG1 and CFG2 pins to tri-state
  // In case of the TMC2100 driver, this equals stealthChop mode with 1/16 steps
  // See https://learn.watterott.com/silentstepstick/pinconfig/tmc2100/#step-configuration
  pinMode(pinCFG1, INPUT);
  pinMode(pinCFG2, INPUT);

  // Configure limit switches to use internal pull-up resistors.
  // Needs setting invertLimit to TRUE, as pins are now active low.
  pinMode(pinLimit1, INPUT_PULLUP);
  pinMode(pinLimit2, INPUT_PULLUP);
  invertLimit = true;

  // Interrupts for limit switches
  if (invertLimit) {
    attachInterrupt(digitalPinToInterrupt(pinLimit1), hitLimit, FALLING);
    attachInterrupt(digitalPinToInterrupt(pinLimit2), hitLimit, FALLING);
  } else {
    attachInterrupt(digitalPinToInterrupt(pinLimit1), hitLimit, RISING);
    attachInterrupt(digitalPinToInterrupt(pinLimit2), hitLimit, RISING);
  }

  // Configure the stepper library
  stepper.setPinEnable(pinEnable);          // We do want to use the enable pin
  stepper.setInvertEnable(true);            // Enable pin on TMC2100 is inverted
  stepper.setInvertDirection(false);        // Invert the direction pin?
  stepper.setStepsPerRev(stepsPerRev);      // Set number of steps per revolution
  stepper.setMaxSpeed(vMax);                // Set max speed
  stepper.setAcceleration(a);               // Set acceleration
  stepper.disableDriver();                  // Disable the driver

  // Extra fancy LED sequence to say hi
  pinMode(pinLED, OUTPUT);
  for (int i = 750; i > 0; i--) {
    delayMicroseconds(i);
    digitalWriteFast(pinLED, HIGH);
    delayMicroseconds(750-i);
    digitalWriteFast(pinLED, LOW);
  }
  for (int i = 1; i <=2 ; i++) {
    delay(75);
    digitalWriteFast(pinLED, HIGH);
    delay(50);
    digitalWriteFast(pinLED, LOW);
  }
}

void loop()
{
  if (usbCOM.available()>0)                                       // Byte available at usbCOM?
    COM = &usbCOM;                                                //   Point *COM to usbCOM
  else if (Serial1COM.available())                                // Byte available at Serial1COM?
    COM = &Serial1COM;                                            //   Point *COM to Serial1COM
  else                                                            // Otherwise
    return;                                                       //   Skip to next iteration of loop()

  opCode = COM->readByte();

  if (opCode == 'D') {                                            // Run degrees (pos = CW, neg = CCW)
    alpha = (int32_t) COM->readInt16();                           //   Read Int16
    runDegrees();                                                 //   Run degrees
  }
  else if (opCode == 'S') {                                       // Run steps (pos = CW, neg = CCW)
    nSteps = (int32_t) COM->readInt16();                          //   Read Int16
    runSteps();                                                   //   Run steps
  }
  else if (opCode == 'A') {                                       // Set acceleration (steps / s^2)
    a = (float) COM->readInt16();                                 //   Read value
    stepper.setAcceleration(a);                                   //   Set acceleration
  }
  else if (opCode == 'V') {                                       // Set speed (steps / s)
    vMax = (float) COM->readInt16();                              //   Read Int16
    stepper.setMaxSpeed(vMax);                                    //   Set Speed
  }
  else if (opCode == 'L') {                                       // Search for limit switch
    direction = COM->readUint8();                                 //   Direction (0 = CCW, 1 = CW)
    findLimit();                                                  //   Search for limit switch
  }
  else if (opCode == 'G') {                                       // Get parameters
    switch (COM->readByte()) {                                    //   Read Byte
      case 'A':                                                   //   Return acceleration
        COM->writeInt16((int16_t)a);
        break;
      case 'V':                                                   //   Return speed
        COM->writeInt16((int16_t)vMax);
        break;
    }
  }
  else if ((opCode == 212) && (COM == &usbCOM)) {                 // USB Handshake
    COM->writeByte(211);
    COM->writeUint32(FirmwareVersion);
  }
  else if ((opCode == 255) && (COM == &Serial1COM)) {             // Return module information (if command arrived via UART)
    returnModuleInfo();
  }
}

void runSteps() {
  digitalWriteFast(pinLED, HIGH);                                 // Enable the onboard LED
  stepper.enableDriver();                                         // Enable the driver
  Serial1COM.writeByte(1);                                        // Send event 1: Start
  stepper.moveSteps(nSteps);                                      // Set destination
  Serial1COM.writeByte(2);                                        // Send event 2: Stop
  stepper.disableDriver();                                        // Disable the driver
  digitalWriteFast(pinLED, LOW);                                  // Disable the onboard LED
  lastDir = nSteps > 0;
}

void runDegrees() {
  digitalWriteFast(pinLED, HIGH);                                 // Enable the onboard LED
  stepper.enableDriver();                                         // Enable the driver
  Serial1COM.writeByte(1);                                        // Send event 1: Start
  stepper.moveDegrees(alpha);                                     // Move by angle alpha
  Serial1COM.writeByte(2);                                        // Send event 2: Stop
  stepper.disableDriver();                                        // Disable the driver
  digitalWriteFast(pinLED, LOW);                                  // Disable the onboard LED
  lastDir = alpha > 0;
}

void findLimit() {
  nSteps = 2147483647;                                            // A very large number
  if (direction == 0)                                             // CCW movement:
    nSteps = -nSteps;                                             //   A very large, negative number
  runSteps();                                                     // Run!
}

void returnModuleInfo() {
  Serial1COM.writeByte(65);                                       // Acknowledge
  Serial1COM.writeUint32(FirmwareVersion);                        // 4-byte firmware version
  Serial1COM.writeByte(sizeof(moduleName) - 1);
  Serial1COM.writeCharArray(moduleName, sizeof(moduleName) - 1);  // Module name
  Serial1COM.writeByte(1);                                        // 1 if more info follows, 0 if not
  Serial1COM.writeByte('#');                                      // Op code for: Number of behavior events this module can generate
  Serial1COM.writeByte(3);                                        // 3 states ("Start", "Stop" and "Limit")
  Serial1COM.writeByte(1);                                        // 1 if more info follows, 0 if not
  Serial1COM.writeByte('E');                                      // Op code for: Behavior event names
  Serial1COM.writeByte(nEventNames);
  for (unsigned int i = 0; i < nEventNames; i++) {                // Once for each event name
    Serial1COM.writeByte(strlen(eventNames[i]));                  // Send event name length
    for (unsigned int j = 0; j < strlen(eventNames[i]); j++) {    // Once for each character in this event name
      Serial1COM.writeByte(*(eventNames[i] + j));                 // Send the character
    }
  }
  Serial1COM.writeByte(0);                                        // 1 if more info follows, 0 if not
}

void hitLimit() {
  if (stepper.isRunning()) {
    stepper.stop();                                               // Stop motor
    Serial1COM.writeByte(3);                                      // Send event 3: Limit
  }
}
