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
extern const uint8_t vDriver;     // version number of TMC stepper driver
extern const teensyPins pin;      // pin numbers
extern volatile uint8_t errorID;  // error ID
storageVars p;                    // struct for EEPROM storage (see EEstoreStruct.h)

// Pointer to StepperWrapper
StepperWrapper* wrapper;

void setup()
{
  DEBUG_WAIT();
  DEBUG_PRINTLN("Welcome to BPOD_STEPPER");
  DEBUG_PRINTF("Hardware revision: %g\n",PCBrev/10.0);
  DEBUG_PRINTF("Firmware version:  %d\n\n",FirmwareVersion);

  Serial1.begin(1312500);                                         // Initialize serial communication
  EEstore<storageVars>::getOrDefault(p);                          // Load parameters from EEPROM

  // Manage error interrupt
  pinMode(pin.Error, OUTPUT);
  digitalWrite(pin.Error, LOW);
  attachInterrupt(digitalPinToInterrupt(pin.Error), throwError, RISING);

  // Decide which implementation of StepperWrapper to load
  if (StepperWrapper::SDmode()) {
    wrapper = new StepperWrapper_TeensyStep();
  } else {
    wrapper = new StepperWrapper_MotionControl();
  }
  wrapper->init(p.rms_current);

  // Load parameters/defaults
  wrapper->setPosition(wrapper->readPosition());                  // read last known position from SD-card
  wrapper->vMax(p.vMax);
  wrapper->a(p.a);
  wrapper->setIOresistor(p.IOresistor,sizeof(p.IOresistor));
  wrapper->setIOmode(p.IOmode,sizeof(p.IOmode));
  wrapper->setChopper(p.chopper);
  wrapper->RMS(p.rms_current);
  wrapper->holdRMS(p.hold_rms_current);

  StepperWrapper::blinkenlights();                                // Indicate successful start-up
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
    wrapper->go2target(opCode-1);
    return;
  }
  switch(opCode) {
    case 'X':                                                     // Stop without slowing down
      wrapper->hardStop();
      return;
    case 'x':                                                     // Stop after slowing down
      wrapper->softStop();
      return;
    case 'S':                                                     // Move to relative position, full steps (pos = CW, neg = CCW)
      wrapper->vMax(p.vMax);
      wrapper->a(p.a);
      wrapper->moveSteps(COM->readInt16());
      return;
    case 's':                                                     // Move to relative position, micro-steps (pos = CW, neg = CCW)
      wrapper->vMax(p.vMax);
      wrapper->a(p.a);
      wrapper->moveMicroSteps(COM->readInt32());
      return;
    case 'P':                                                     // Move to absolute position, full steps
      wrapper->vMax(p.vMax);
      wrapper->a(p.a);
      wrapper->position(COM->readInt16());
      return;
    case 'p':                                                     // Move to absolute position, micro-steps
      wrapper->vMax(p.vMax);
      wrapper->a(p.a);
      wrapper->microPosition(COM->readInt32());
      return;
    case 'F':                                                     // Start moving forwards
      wrapper->vMax(p.vMax);
      wrapper->a(p.a);
      wrapper->rotate(1);
      return;
    case 'B':                                                     // Start moving backwards
      wrapper->vMax(p.vMax);
      wrapper->a(p.a);
      wrapper->rotate(-1);
      return;
    case 'Z':                                                     // Reset position to zero
      wrapper->setPosition(0);
      return;
    case 'z':                                                     // Reset encoder position to zero
      wrapper->resetEncoderPosition();
      return;
    case 'A':                                                     // Set acceleration (steps / s^2)
      wrapper->a(COM->readUint16());
      p.a = wrapper->a();
      return;
    case 'V':                                                     // Set peak velocity (steps / s)
      wrapper->vMax(COM->readUint16());
      p.vMax = wrapper->vMax();
      return;
    case 'I':                                                     // Set RMS current (mA)
      wrapper->RMS(COM->readUint16());
      p.rms_current = wrapper->RMS();
      return;
    case 'i':                                                     // Set hold RMS current (mA)
      wrapper->holdRMS(COM->readUint16());
      p.hold_rms_current = wrapper->holdRMS();
      return;
    case 'C':                                                     // Set chopper mode (0 = PWM chopper, 1 = voltage chopper)
      wrapper->setChopper(COM->readUint8());
      p.chopper = wrapper->getChopper();
      return;
    case 'M':                                                     // Set mode for IO port
    {
      uint8_t idx  = COM->readUint8();
      uint8_t mode = COM->readUint8();
      wrapper->setIOmode(idx,mode);
      if (idx>0 && idx <= 6)
        p.IOmode[idx-1] = wrapper->getIOmode(idx);
      return;
    }
    case 'R':                                                     // Set input resistor for IO port (0 = no resistor, 1 = pullup, 2 = pulldown)
    {
      uint8_t idx = COM->readUint8();
      uint8_t res = COM->readUint8();
      wrapper->setIOresistor(idx,res);
      if (idx>0 && idx <= 6)
        p.IOresistor[idx-1] = wrapper->getIOresistor(idx);
      return;
    }
    case 'Y':                                                     // Set steps per revolution
    {
      wrapper->stepsPerRevolution(COM->readUint16());
      p.stepsPerRevolution = wrapper->stepsPerRevolution();
      return;
    }
    case 'y':                                                     // Set encoder steps per revolution
    {
      wrapper->countsPerRevolution(COM->readUint16());
      p.countsPerRevolution = wrapper->countsPerRevolution();
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
      wrapper->setStream(enable);
      return;
    }
  }

  // The following commands will also RETURN data via serial - they cannot be used during stream mode
  if (COM == &usbCOM && wrapper->getStream())
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
          COM->writeInt16(wrapper->position());
          break;
        case 'p':                                                 //   Return micro-position
          COM->writeInt32(wrapper->microPosition());
          break;
        case 'N':                                                 //   Return encoder position
          COM->writeInt32(wrapper->encoderPosition());
          break;
        case 'A':                                                 //   Return acceleration
          COM->writeUint16(round(wrapper->a()));
          break;
        case 'V':                                                 //   Return speed
          COM->writeUint16(round(wrapper->vMax()));
          break;
        case 'H':                                                 //   Return hardware revision
          COM->writeUint8(PCBrev);
          break;
        case 'M':
          COM->writeUint8(wrapper->getIOmode(COM->readByte()));
          break;
        case 'R':
          COM->writeUint8(wrapper->getIOresistor(COM->readByte()));
          break;
        case 'I':
          COM->writeUint16(wrapper->RMS());
          break;
        case 'i':
          COM->writeUint16(wrapper->holdRMS());
          break;
        case 'C':                                                 //   Return chopper mode (0 = PWM chopper, 1 = voltage chopper)
          COM->writeUint8(wrapper->getChopper());
          break;
        case 'T':
          COM->writeUint8(vDriver);
          break;
        case 'Y':                                                 //   Return steps per revolution
          COM->writeUint16(wrapper->stepsPerRevolution());
          break;
        case 'y':                                                 //   Return encoder counts per revolution
          COM->writeUint16(wrapper->countsPerRevolution());
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
