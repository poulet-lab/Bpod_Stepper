#include "ArCOM.h"         // Import serial communication wrapper
#include "SmoothStepper.h" // Import stepper library

// Module setup
ArCOM Serial1COM(Serial1);              // Wrap Serial1 (UART on Arduino M0, Due + Teensy 3.X)
char moduleName[] = "StepperMotor";     // Name of module for manual override UI and state machine assembler
char* eventNames[] = {"Start", "Stop"};

// Constants
#define FirmwareVersion 1
#define pinDir     2
#define pinStep    3
#define pinSleep   4
#define pinReset   5
#define pinCFG3    6
#define pinCFG2    7
#define pinCFG1    8
#define pinEnable  9
#define pinLimit1 10
#define pinLimit2 11
#define pinLED    13

#define fullstepsPerRev 200
#define uStepRes 16

// Variables
byte nEventNames = (sizeof(eventNames) / sizeof(char *));
byte opCode      = 0;
long nSteps      = 0;
SmoothStepper stepper(pinStep, pinDir);

void setup()
{
  Serial1.begin(1312500);
  stepper.setPinEnable(pinEnable);                // We do want to use the enable pin
  stepper.setStepsPerRev(3200);                   // TMC2100 is driven by 16 microsteps/step
  stepper.setMaxSpeed(200 * uStepRes);            // Set a sensible max speed
  stepper.setAcceleration(200 * uStepRes);        // Set default acceleration
  stepper.disableDriver();
}

void loop()
{
  if (Serial1COM.available()) {
    opCode = Serial1COM.readByte();
    if ((opCode == 0)) {
      nSteps = Serial1COM.readByte() * uStepRes;
      runSteps();
    }
    else if ((opCode == 1)) {
      nSteps = -1 * (int)Serial1COM.readUint8() * uStepRes;
      runSteps();
    }
    else if ((opCode == 2)) {                                 // Set acceleration
      float acc = (float)Serial1COM.readUint8() * uStepRes;   // Read acceleration
      stepper.setAcceleration(acc);                           // Set Speed
      Serial.println(acc);
      Serial1COM.writeByte(65);
    }
    else if ((opCode == 3)) {                                 // Set speed
      float vmax = Serial1COM.readByte();                     // Read speed
      stepper.setMaxRPM(vmax);                                // Set Speed
      Serial1COM.writeByte(65);
    }
    else if (opCode == 255) {
      returnModuleInfo();
    }
  }
}

float rpm2sps(float rpm) {
  float sps = (rpm * fullstepsPerRev * uStepRes) / 60;
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
