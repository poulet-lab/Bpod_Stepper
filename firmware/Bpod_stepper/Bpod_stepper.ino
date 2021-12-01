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


_______________________________________________________________________________
*/


#include <Arduino.h>
#include <avr/io.h>
#include <avr/interrupt.h>
#include <limits>
#include "ArCOM.h"                // Import serial communication wrapper
#include "EEstore.h"              // Import EEstore library
#include "StepperWrapper.h"       // Import StepperWrapper
#include "SerialDebug.h"

// Module setup
ArCOM usbCOM(Serial);             // Wrap Serial (USB on Teensy 3.X)
ArCOM Serial1COM(Serial1);        // Wrap Serial1 (UART on Arduino M0, Due + Teensy 3.X)
ArCOM *COM;                       // Pointer to ArCOM object
char  moduleName[] = "Stepper";   // Name of module for manual override UI and state machine assembler
const char* eventNames[] = {"Error", "Start", "Stop", "EmergencyStop"};
#define FirmwareVersion 2

// Variables
uint8_t nEventNames  = sizeof(eventNames) / sizeof(char *);
uint8_t opCode       = 0;
extern const float PCBrev;        // PCB revision
extern const uint8_t vDriver;     // version number of TMC stepper driver
extern const teensyPins pin;      // pin numbers
extern volatile uint8_t errorID;  // error ID

// Parameters to be loaded from EEPROM (and default values)
static const int StoreAddress = 0;
typedef struct{
  uint16_t rms_current = 800;     // motor RMS current (mA)
  float vMax = 200;
  float a = 800;
  uint8_t chopper = 1;
  int32_t target[9] {0};
  uint8_t IOmode[6] {0};
  uint8_t IOresistor[6] {0};
}storageVars;
storageVars p;

// Pointer to StepperWrapper
StepperWrapper* wrapper;

void setup()
{
  DEBUG_DELAY(1000);

  // Initialize serial communication
  Serial1.begin(1312500);

  // Load parameters from EEPROM
  EEstore<storageVars>::getOrDefault(StoreAddress,p);

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

  // Set default values
  wrapper->vMax(p.vMax);
  wrapper->a(p.a);
  wrapper->setIOresistor(p.IOresistor,sizeof(p.IOresistor));
  wrapper->setIOmode(p.IOmode,sizeof(p.IOmode));
  wrapper->setChopper(p.chopper);

  // Indicate successful start-up
  StepperWrapper::blinkenlights();
}


void loop()
{
  if (limit) {                                                    // Limit switch reached (called by interrupt)
    wrapper->hardStop();
    Serial1COM.writeByte(3);
    limit = false;
  }
  if (go2pos) {                                                   // Go to predefined target (called by interrupt)
    wrapper->position(p.target[go2pos-1]);
    go2pos = 0;
  }

  if (usbCOM.available()>0)                                       // Byte available at usbCOM?
    COM = &usbCOM;                                                //   Point *COM to usbCOM
  else if (Serial1COM.available())                                // Byte available at Serial1COM?
    COM = &Serial1COM;                                            //   Point *COM to Serial1COM
  else                                                            // Otherwise
    return;                                                       //   Skip to next iteration of loop()

  opCode = COM->readByte();

  if (opCode <= 9 && opCode >= 1)  {                              // Move to predefined target (0-9)
    wrapper->position(p.target[opCode-1]);
    return;
  }
  switch(opCode) {
    case 'X':                                                     // Stop without slowing down
      wrapper->hardStop();
      break;
    case 'x':                                                     // Stop after slowing down
      wrapper->softStop();
      break;
    case 'S':                                                     // Move to relative position (pos = CW, neg = CCW)
      wrapper->moveSteps(COM->readInt16());
      Serial1COM.writeByte(2);
      break;
    case 'P':                                                     // Move to absolute position
      Serial1COM.writeByte(2);
      wrapper->position(COM->readInt16());
      break;
    case 'L':                                                     // Start rotating (pos = CW, neg = CCW)
      wrapper->rotate(COM->readInt8());
      Serial1COM.writeByte(2);
    case 'Z':                                                     // Reset position to zero
      wrapper->resetPosition();
      break;
    case 'A':                                                     // Set acceleration (steps / s^2)
      wrapper->a(COM->readUint16());
      p.a = wrapper->a();
      break;
    case 'V':                                                     // Set peak velocity (steps / s)
      wrapper->vMax(COM->readUint16());
      p.vMax = wrapper->vMax();
      break;
    case 'I':                                                     // Set RMS current (mA)
      wrapper->RMS(COM->readUint16());
      p.rms_current = wrapper->RMS();
      break;
    case 'C':                                                     // Set chopper mode (0 = spreadCycle™, 1 = stealthChop™)
      wrapper->setChopper(COM->readUint8());
      p.chopper = wrapper->getChopper();
      break;
    case 'M':                                                     // Set mode for IO port
    {
      uint8_t idx  = COM->readUint8();
      uint8_t mode = COM->readUint8();
      wrapper->setIOmode(idx,mode);
      if (idx>0 && idx <= 6)
        p.IOmode[idx-1] = wrapper->getIOmode(idx);
      break;
    }
    case 'R':                                                     // Set input resistor for IO port (0 = no resistor, 1 = pullup, 2 = pulldown)
    {
      uint8_t idx = COM->readUint8();
      uint8_t res = COM->readUint8();
      wrapper->setIOresistor(idx,res);
      if (idx>0 && idx <= 6)
        p.IOresistor[idx-1] = wrapper->getIOresistor(idx);
      break;
    }
    case 'T':                                                     // Set predefined target
    {
      uint8_t  idx = COM->readUint8()-48;
      uint32_t pos = COM->readInt32();
      if (idx <= 9 && opCode >= 1)
        p.target[idx-1] = pos;
      break;
    }
    case 'E':                                                     // Store current settings to EEPROM
      EEstore<storageVars>::set(StoreAddress,p);
      break;
    case 'G':                                                     // Get parameters
      opCode = COM->readByte();                                   //   Read Byte
      if (opCode <= 9 && opCode >= 1) {                           //   Return predefined target
        COM->writeInt32(p.target[opCode-1]);
        return;
      }
      switch (opCode) {
        case 'P':                                                 //   Return position
          COM->writeInt16(wrapper->position());
          break;
        case 'A':                                                 //   Return acceleration
          COM->writeUint16(round(wrapper->a()));
          break;
        case 'V':                                                 //   Return speed
          COM->writeUint16(round(wrapper->vMax()));
          break;
        case 'H':                                                 //   Return hardware revision
          COM->writeUint8(PCBrev * 10);
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
        case 'C':                                                 //   Return chopper mode (0 = spreadCycle™, 1 = stealthChop™)
          COM->writeUint8(wrapper->getChopper());
          break;
        case 'T':
          COM->writeUint8(vDriver);
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
  // Placeholder for proper error handler
  DEBUG_PRINT("Error ");
  DEBUG_PRINTLN(errorID);
  Serial1COM.writeByte(1);
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
