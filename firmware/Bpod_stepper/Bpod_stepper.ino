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
#include "ArCOM.h"                // Import serial communication wrapper
#include "StepperWrapper.h"       // Import StepperWrapper
#include "EEstore.h"              // Import EEstore library
#include <avr/io.h>
#include <avr/interrupt.h>

// Module setup
ArCOM usbCOM(Serial);             // Wrap Serial (USB on Teensy 3.X)
ArCOM Serial1COM(Serial1);        // Wrap Serial1 (UART on Arduino M0, Due + Teensy 3.X)
ArCOM *COM;                       // Pointer to ArCOM object
char  moduleName[] = "Stepper";   // Name of module for manual override UI and state machine assembler
const char* eventNames[] = {"Start", "Stop", "Limit"};
#define FirmwareVersion 2

// Constants
static const int StoreAddress = 0;// 
static const float fCLK = 12E6;   // internal clock frequencz of TMC5160
static const float factA = (float)(1ul<<24) / (fCLK * fCLK / (512.0 * 256.0));
static const float factV = (float)(1ul<<24) / (fCLK);

// Variables
uint8_t  nEventNames = sizeof(eventNames) / sizeof(char *);
uint8_t  opCode      = 0;
float    PCBrev      = 0;
teensyPins pin;

// Parameters to be loaded from EEPROM (and default values)
typedef struct{
  uint16_t rms_current = 400;     // motor RMS current (mA)
  float vMax = 200;
  float a = 200;
}storageVars;
storageVars p;

// Pointer to StepperWrapper
StepperWrapper* wrapper;

void setup()
{
  // Initialize serial communication
  Serial1.begin(1312500);

  // Load parameters from EEPROM
  EEstore<storageVars>::getOrDefault(StoreAddress,p);

  // Identify PCB, obtain pin-layout
  PCBrev = StepperWrapper::idPCB(); 
  pin = StepperWrapper::getPins(PCBrev);

  // Manage error interrupt
  pinMode(pin.Error, OUTPUT);
  digitalWrite(pin.Error, LOW);
  attachInterrupt(digitalPinToInterrupt(pin.Error), throwError, RISING);

  // TODO: Check motor voltage (needs hardware support)
  pinMode(pin.VM, INPUT);
  attachInterrupt(digitalPinToInterrupt(pin.VM), powerGain, RISING);
  attachInterrupt(digitalPinToInterrupt(pin.VM), powerLoss, FALLING);
  
  // Decide which implementation of StepperWrapper to load
  if (StepperWrapper::SDmode()) {
    wrapper = new StepperWrapper_SmoothStepper();
  } else {
    wrapper = new StepperWrapper_MotionControl();
  }
  wrapper->init(p.rms_current);

  // Set default speed & acceleration
  wrapper->vMax(p.vMax);
  wrapper->a(p.a);
  
  // TODO: Manage DIAG Interrupts -> move to StepperWrapper
  // driver.RAMP_STAT(driver.RAMP_STAT()); // clear flags & interrupt conditions
  // attachInterrupt(digitalPinToInterrupt(pinDiag0), interrupt, FALLING);

  // Indicate successful start-up
  StepperWrapper::blinkenlights();
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
  switch(opCode) {
    case 'S':                                                     // Move to relative position (pos = CW, neg = CCW)
      wrapper->moveSteps(COM->readInt16());
      break;
    case 'P':                                                     // Move to absolute position
      wrapper->position(COM->readInt16());
      break;
    case 'L':                                                     // Search for limit switch
    //   direction = COM->readUint8();                               //   Direction (0 = CCW, 1 = CW)
    //   findLimit();                                                //   Search for limit switch
      break;
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
    case 'E':                                                     // Store current settings to EEPROM
      EEstore<storageVars>::set(StoreAddress,p);
      break;
    case 'G':                                                     // Get parameters
      switch (COM->readByte()) {                                  //   Read Byte
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
        case 'I':
          COM->writeUint16(wrapper->RMS());
          break;
        case 'T':
          COM->writeUint8(wrapper->getTMC5160());
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
  Serial.print("Error ");
  Serial.println(wrapper->getErrorID());
  digitalWrite(StepperWrapper::errorPin, LOW);
  StepperWrapper::blinkError();
}

void powerGain() {
  SCB_AIRCR = 0x05FA0004; // Reset teensy
}

void powerLoss() {
  // throw error
}

// void interrupt() {
//   digitalWriteFast(LED_BUILTIN, LOW);
//
//   // read RAMP_STAT to get reason for interrupt
//   uint16_t ramp_stat = driver.RAMP_STAT();
//   if (bitRead(ramp_stat,6)) {
//     // event_stop_sg
//   } else if (bitRead(ramp_stat,7)) {
//     // event_pos_reached
//   }
//
//   // clear flags & interrupt conditions
//   driver.RAMP_STAT(ramp_stat);
// }

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
