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

// Parameters to be loaded from EEPROM (and default values)
typedef struct{
  uint16_t rms_current = 400;     // motor RMS current (mA)
}storageVars;
storageVars p;

// Pointer to StepperWrapper
StepperWrapper* wrapper;

void setup()
{
  // Initialize serial communication
  Serial1.begin(1312500);

  delay(1000);

  // Load parameters from EEPROM
  loadParams();

  // Manage error interrupt
  pinMode(StepperWrapper::errorPin, OUTPUT);
  digitalWrite(StepperWrapper::errorPin, LOW);            // error pin starts off LOW
  attachInterrupt(digitalPinToInterrupt(StepperWrapper::errorPin), throwError, RISING);

  // Decide which implementation of StepperWrapper to load
  if (StepperWrapper::SDmode()) {
    wrapper = new StepperWrapper_SmoothStepper();
  } else {
    wrapper = new StepperWrapper_MotionControl();
  }
  wrapper->init(p.rms_current);

  // Set default speed & acceleration
  wrapper->setSpeed(200);
  wrapper->setAcceleration(400);

  // TODO: Manage DIAG Interrupts -> move to StepperWrapper
  // driver.RAMP_STAT(driver.RAMP_STAT()); // clear flags & interrupt conditions
  // attachInterrupt(digitalPinToInterrupt(pinDiag0), interrupt, FALLING);

  // Extra fancy LED sequence to say hi
  StepperWrapper::blinkenlights();
}


void loop()
{
  wrapper->setTarget(200);
  delay(4000);
  wrapper->setTarget(-200);
  delay(4000);
}

void throwError() {
  // Placeholder for proper error handler
  Serial.print("Error ");
  Serial.println(wrapper->getErrorID());
  digitalWrite(StepperWrapper::errorPin, LOW);
  StepperWrapper::blinkError();
}

void set_rms_current(uint16_t rms_current) {
  p.rms_current = 800;
  storeParams();
}

void loadParams() {
  EEstore<storageVars>::getOrDefault(StoreAddress,p);
}

void storeParams() {
  EEstore<storageVars>::set(StoreAddress,p);
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
