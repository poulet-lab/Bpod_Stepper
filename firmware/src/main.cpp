/*
Bpod_stepper
Copyright (C) 2020 Florian Rau

This program is free software: you can redistribute it and/or modify it under
the terms of the GNU General Public License as published by the Free Software
Foundation, version 3.

This program is distributed  WITHOUT ANY WARRANTY and without even the implied
warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU
General Public License for more details.

You should have received a copy of the GNU General Public License along with
this program.  If not, see <http://www.gnu.org/licenses/>.
*/

#include <Arduino.h>
#include <avr/io.h>
#include <limits>
#include <ArCOM.h>                // Import serial communication wrapper
#include "EEstore.h"              // Import EEstore library
#include "EEstoreStruct.h"        // Parameters to be loaded from EEPROM (and default values)
#include "StepperWrapper.h"       // Import StepperWrapper
#include "SerialDebug.h"

void throwError();
void returnModuleInfo();

// Module setup
ArCOM usbCOM(Serial);             // Wrap Serial (USB on Teensy 3.X)
ArCOM Serial1COM(Serial1);        // Wrap Serial1 (UART on Arduino M0, Due + Teensy 3.X)
ArCOM *COM;                       // Pointer to ArCOM object
char  moduleName[] = "Stepper";   // Name of module for manual override UI and state machine assembler
const char* eventNames[] = {"Error", "Start", "Stop", "EStop"};
#define FirmwareVersion 2

// Variables
uint8_t nEventNames  = sizeof(eventNames) / sizeof(char *);
uint8_t opCode       = 0;
extern const uint8_t PCBrev;      // PCB revision
extern const teensyPins pin;      // pin numbers
extern volatile uint8_t errorID;  // error ID
storageVars p;                    // struct for EEPROM storage (see EEstoreStruct.h)

// Pointer to StepperWrapper
StepperWrapper* stepper;

void setup()
{
  Serial1.begin(1312500);                                         // Initialize serial communication
  DEBUG_WAIT();

  DEBUG_PRINTLN("Welcome to BPOD_STEPPER");
  DEBUG_PRINTF("Hardware revision: %g\n",PCBrev/10.0);
  DEBUG_PRINTF("Firmware version:  %d\n",FirmwareVersion);
  DEBUG_PRINTF("Driver version:    %s\n\n",stepper->name);

  EEstore<storageVars>::getOrDefault(p);                          // Load parameters from EEPROM

  // Manage error interrupt
  pinMode(pin.Error, OUTPUT);
  digitalWrite(pin.Error, LOW);
  attachInterrupt(digitalPinToInterrupt(pin.Error), throwError, RISING);

  // Decide which implementation of StepperWrapper to use
  if (StepperWrapper::is5160 && !StepperWrapper::SDmode())
    stepper = new StepperWrapper_MotionControl();
  else
    stepper = new StepperWrapper_TeensyStep();

  // Initialize StepperWrapper
  stepper->init(p.rms_current);

  // Indicate successful start-up
  StepperWrapper::blinkenlights();
}


void loop()
{
  if (ISRcode>0) {                                                // Byte set through interrupt sub-routine
    opCode  = ISRcode;
    ISRcode = 0;
  }
  else {
    if (usbCOM.available()>0)                                     // Byte available at usbCOM?
      COM = &usbCOM;                                              //   Point *COM to usbCOM
    else if (Serial1COM.available())                              // Byte available at Serial1COM?
      COM = &Serial1COM;                                          //   Point *COM to Serial1COM
    else                                                          // Otherwise
      return;                                                     //   Skip to next iteration of loop()
    opCode = COM->readByte();
  }

  if (opCode <= 9 && opCode >= 1)  {                              // Move to predefined target (0-9)
    stepper->go2target(opCode-1);
    return;
  }
  switch(opCode) {
    case 'X':                                                     // Stop without slowing down
      stepper->hardStop();
      return;
    case 'x':                                                     // Stop after slowing down
      stepper->softStop();
      return;
    case 'S':                                                     // Move to relative position, full steps (pos = CW, neg = CCW)
      stepper->vMax(p.vMax);
      stepper->a(p.a);
      stepper->moveSteps(COM->readInt16());
      return;
    case 's':                                                     // Move to relative position, micro-steps (pos = CW, neg = CCW)
      stepper->vMax(p.vMax);
      stepper->a(p.a);
      stepper->moveMicroSteps(COM->readInt32());
      return;
    case 'P':                                                     // Move to absolute position, full steps
      stepper->vMax(p.vMax);
      stepper->a(p.a);
      stepper->position(COM->readInt16());
      return;
    case 'p':                                                     // Move to absolute position, micro-steps
      stepper->vMax(p.vMax);
      stepper->a(p.a);
      stepper->microPosition(COM->readInt32());
      return;
    case 'F':                                                     // Start moving forwards
      stepper->vMax(p.vMax);
      stepper->a(p.a);
      stepper->rotate(1);
      return;
    case 'B':                                                     // Start moving backwards
      stepper->vMax(p.vMax);
      stepper->a(p.a);
      stepper->rotate(-1);
      return;
    case 'Z':                                                     // Reset position to zero
      stepper->setPosition(0);
      return;
    case 'z':                                                     // Reset encoder position to zero
      stepper->resetEncoderPosition();
      return;
    case 'A':                                                     // Set acceleration (steps / s^2)
      stepper->a(COM->readUint16());
      p.a = stepper->a();
      return;
    case 'V':                                                     // Set peak velocity (steps / s)
      stepper->vMax(COM->readUint16());
      p.vMax = stepper->vMax();
      return;
    case 'I':                                                     // Set RMS current (mA)
      stepper->RMS(COM->readUint16());
      p.rms_current = stepper->RMS();
      return;
    case 'i':                                                     // Set hold RMS current (mA)
      stepper->holdRMS(COM->readUint16());
      p.hold_rms_current = stepper->holdRMS();
      return;
    case 'C':                                                     // Set chopper mode (0 = PWM chopper, 1 = voltage chopper)
      stepper->setChopper(COM->readUint8());
      p.chopper = stepper->getChopper();
      return;
    case 'M':                                                     // Set mode for IO port
    {
      uint8_t idx  = COM->readUint8();
      uint8_t mode = COM->readUint8();
      stepper->setIOmode(idx,mode);
      if (idx>0 && idx <= 6)
        p.IOmode[idx-1] = stepper->getIOmode(idx);
      return;
    }
    case 'R':                                                     // Set input resistor for IO port (0 = no resistor, 1 = pullup, 2 = pulldown)
    {
      uint8_t idx = COM->readUint8();
      uint8_t res = COM->readUint8();
      stepper->setIOresistor(idx,res);
      if (idx>0 && idx <= 6)
        p.IOresistor[idx-1] = stepper->getIOresistor(idx);
      return;
    }
    case 'Y':                                                     // Set steps per revolution
    {
      stepper->stepsPerRevolution(COM->readUint16());
      p.stepsPerRevolution = stepper->stepsPerRevolution();
      return;
    }
    case 'y':                                                     // Set encoder steps per revolution
    {
      stepper->countsPerRevolution(COM->readUint16());
      p.countsPerRevolution = stepper->countsPerRevolution();
      return;
    }
    case 'T':                                                     // Set predefined target
    {
      uint8_t idx = COM->readUint8() - 1;
      if (idx <= 8 && idx >= 0) {
        p.target[idx]     = COM->readInt32();
        p.aTarget[idx]    = COM->readUint16();
        p.vMaxTarget[idx] = COM->readUint16();
      }
      return;
    }
    case 'E':                                                     // Store current settings to EEPROM
      EEstore<storageVars>::set(p);
      return;
    case 'L':                                                     // Toggle live streaming of motor status
    {
      bool enable = COM->readUint8();
      stepper->setStream(enable);
      return;
    }
  }

  // The following commands will also RETURN data via serial - they cannot be used during stream mode
  if (COM == &usbCOM && stepper->getStream())
    return;
  switch(opCode) {
    case 'G':                                                     // Get parameters
      opCode = COM->readByte();                                   //   Read Byte
      if (opCode <= 9 && opCode >= 1) {                           //   Return predefined target
        uint8_t idx = opCode - 1;
        COM->writeInt32(p.target[idx]);
        COM->writeUint16(round(p.aTarget[idx]));
        COM->writeUint16(round(p.vMaxTarget[idx]));
        return;
      }
      switch (opCode) {
        case 'P':                                                 //   Return position
          COM->writeInt16(stepper->position());
          break;
        case 'p':                                                 //   Return micro-position
          COM->writeInt32(stepper->microPosition());
          break;
        case 'N':                                                 //   Return encoder position
          COM->writeInt32(stepper->encoderPosition());
          break;
        case 'A':                                                 //   Return acceleration
          COM->writeUint16(round(stepper->a()));
          break;
        case 'V':                                                 //   Return speed
          COM->writeUint16(round(stepper->vMax()));
          break;
        case 'H':                                                 //   Return hardware revision
          COM->writeUint8(PCBrev);
          break;
        case 'M':
          COM->writeUint8(stepper->getIOmode(COM->readByte()));
          break;
        case 'R':
          COM->writeUint8(stepper->getIOresistor(COM->readByte()));
          break;
        case 'I':
          COM->writeUint16(stepper->RMS());
          break;
        case 'i':
          COM->writeUint16(stepper->holdRMS());
          break;
        case 'C':                                                 //   Return chopper mode (0 = PWM chopper, 1 = voltage chopper)
          COM->writeUint8(stepper->getChopper());
          break;
        case 'T':
          COM->writeUint8(stepper->vDriver);
          break;
        case 'Y':                                                 //   Return steps per revolution
          COM->writeUint16(stepper->stepsPerRevolution());
          break;
        case 'y':                                                 //   Return encoder counts per revolution
          COM->writeUint16(stepper->countsPerRevolution());
          break;

      }
      break;
    case 212:                                                     // USB Handshake
      if (COM == &usbCOM) {                                       //   Check if connected via USB
        COM->writeByte(211);
        COM->writeUint32(FirmwareVersion);
      }
      break;
    case 255:                                                     // Return module information
      if (COM == &Serial1COM) {                                   //   Check if connected via UART
        returnModuleInfo();
      }
      break;
  }
}

void throwError() {
  // TODO: Implement proper error handler
  Serial1COM.writeByte(1);
}

void returnModuleInfo() {
  // FSM firmware v23 or newer sends a second info request byte to indicate that
  // it supports additional ops
  delayMicroseconds(100);
  boolean fsmSupportsHwInfo =
    (Serial1COM.available() && Serial1COM.readByte() == 255) ? true : false;

  Serial1COM.writeByte(65);                                       // Acknowledge
  Serial1COM.writeUint32(FirmwareVersion);                        // 4-byte firmware version
  Serial1COM.writeByte(sizeof(moduleName) - 1);
  Serial1COM.writeCharArray(moduleName, sizeof(moduleName) - 1);  // Module name
  Serial1COM.writeByte(1);                                        // 1 if more info follows, 0 if not
  Serial1COM.writeByte('#');                                      // Op code for: Number of behavior events this module can generate
  Serial1COM.writeByte(nEventNames);
  Serial1COM.writeByte(1);                                        // 1 if more info follows, 0 if not
  Serial1COM.writeByte('E');                                      // Op code for: Behavior event names
  Serial1COM.writeByte(nEventNames);
  for (unsigned int i = 0; i < nEventNames; i++) {                // Once for each event name
    Serial1COM.writeByte(strlen(eventNames[i]));                  // Send event name length
    for (unsigned int j = 0; j < strlen(eventNames[i]); j++) {    // Once for each character in this event name
      Serial1COM.writeByte(*(eventNames[i] + j));                 // Send the character
    }
  }
  if (fsmSupportsHwInfo) {
    Serial1COM.writeByte(1);                                      // 1 if more info follows, 0 if not
    Serial1COM.writeByte('V');                                    // Op code for: Hardware major version
    Serial1COM.writeByte(PCBrev/10);
    Serial1COM.writeByte(1);                                      // 1 if more info follows, 0 if not
    Serial1COM.writeByte('v');                                    // Op code for: Hardware minor version
    Serial1COM.writeByte(PCBrev%10);
  }
  Serial1COM.writeByte(0);                                        // 1 if more info follows, 0 if not
}
