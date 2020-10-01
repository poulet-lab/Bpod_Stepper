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
#include "SmoothStepper.h" // Import stepper library

// Module setup
ArCOM Serial1COM(Serial1);              // Wrap Serial1 (UART on Arduino M0, Due + Teensy 3.X)
char moduleName[] = "Stepper";          // Name of module for manual override UI and state machine assembler
char* eventNames[] = {"Start", "Stop"};

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
#define stepsPerRev   200
#define uStepRes       16

// Variables
byte nEventNames   = sizeof(eventNames) / sizeof(char *);
byte opCode        = 0;
long nSteps        = 0;
long uStepsPerRev  = stepsPerRev * uStepRes;
SmoothStepper stepper(pinStep, pinDir);

void setup()
{
  Serial.begin(9600);
  
  Serial1.begin(1312500);
  stepper.setPinEnable(pinEnable);                // We do want to use the enable pin
  stepper.setInvertEnable(true);                  // enable pin on TMC2100 is inverted
  stepper.setInvertDirection(false);              // invert direction pin?
  stepper.setMaxSpeed(uStepsPerRev);              // Set a sensible max speed (default: 360°/s)
  stepper.setAcceleration(uStepsPerRev);          // Set default acceleration (default: 360°/s^2)
  stepper.disableDriver();

  // Set all configuration pins to tri-state
  // In case of the TMC2100 driver, this equals stealthChop mode with 1/16 steps
  // See https://learn.watterott.com/silentstepstick/pinconfig/tmc2100/#step-configuration
  pinMode(pinCFG1, INPUT); 
  pinMode(pinCFG2, INPUT);
  pinMode(pinCFG3, INPUT);

  // Fancy LED sequence to say hi
  pinMode(pinLED, OUTPUT);
  for (int i = 750; i > 0; i--) {
    delayMicroseconds(i);
    digitalWrite(pinLED, HIGH);
    delayMicroseconds(750-i);
    digitalWrite(pinLED, LOW);
  }
  for (int i = 1; i <=2 ; i++) {
    delay(75);
    digitalWrite(pinLED, HIGH);
    delay(50);
    digitalWrite(pinLED, LOW);
  }
}

void loop()
{
  if (Serial1COM.available()) {
    opCode = Serial1COM.readByte();
    if (opCode == 0) {                                            // Disable Driver
      stepper.disableDriver();
    }
    else if (opCode == 1) {                                       // Enable Driver
      stepper.enableDriver();
    }
    else if (opCode == 2) {                                       // Set acceleration (full steps / s^2)
      float acc = (float) Serial1COM.readInt16();                 //   Read value
      stepper.setAcceleration(acc * uStepRes);                    //   Set acceleration
    }
    else if (opCode == 3) {                                       // Set speed (full steps / s)
      float vmax = (float) Serial1COM.readInt16();                //   Read value
      stepper.setMaxSpeed(vmax * uStepRes);                       //   Set Speed
    }
    else if (opCode == 4) {                                       // Run full steps
      nSteps = (long) Serial1COM.readInt16() * uStepRes;          //   Read value, correct for uSteps
      runSteps();                                                 //   Run steps
    }
    else if (opCode == 5) {                                       // Run uSteps
      nSteps = (long) Serial1COM.readInt16();                     //   Read value, correct for uSteps
      runSteps();                                                 //   Run steps
    }
    else if (opCode == 6) {                                       // Run degrees
      nSteps = (long) Serial1COM.readInt16() * uStepsPerRev / 360;
      runSteps();
    }
    else if (opCode == 255) {                                     // Return module information
      returnModuleInfo();
    }
  }
}

float rpm2sps(float rpm) {
  float sps = (rpm * stepsPerRev * uStepRes) / 60;
  return sps;
}

void runSteps() {
  stepper.enableDriver();
  digitalWrite(pinLED, HIGH);                                     // Enable the onboard LED
  Serial1COM.writeByte(1);                                        // Send event 1: Start
  stepper.moveSteps(nSteps);                                      // Set destination
  Serial1COM.writeByte(2);                                        // Send event 2: Stop
  digitalWrite(pinLED, LOW);                                      // Disable the onboard LED
  stepper.disableDriver();
}

void returnModuleInfo() {
  Serial1COM.writeByte(65);                                       // Acknowledge
  Serial1COM.writeUint32(FirmwareVersion);                        // 4-byte firmware version
  Serial1COM.writeByte(sizeof(moduleName) - 1);
  Serial1COM.writeCharArray(moduleName, sizeof(moduleName) - 1);  // Module name
  Serial1COM.writeByte(1);                                        // 1 if more info follows, 0 if not
  Serial1COM.writeByte('#');                                      // Op code for: Number of behavior events this module can generate
  Serial1COM.writeByte(2);                                        // 2 states ("Start" and "Stop")
  Serial1COM.writeByte(1);                                        // 1 if more info follows, 0 if not
  Serial1COM.writeByte('E');                                      // Op code for: Behavior event names
  Serial1COM.writeByte(nEventNames);
  for (int i = 0; i < nEventNames; i++) {                         // Once for each event name
    Serial1COM.writeByte(strlen(eventNames[i]));                  // Send event name length
    for (int j = 0; j < strlen(eventNames[i]); j++) {             // Once for each character in this event name
      Serial1COM.writeByte(*(eventNames[i] + j));                 // Send the character
    }
  }
  Serial1COM.writeByte(0);                                        // 1 if more info follows, 0 if not
}
