#pragma once

typedef struct{
  uint16_t rms_current = 800;     // motor RMS current (mA)
  float vMax = 200;
  float a = 800;
  uint8_t chopper = 1;
  int32_t target[9] {0};
  float aTarget[9] {0};
  float vMaxTarget[9] {0};
  uint8_t IOmode[6] {0};
  uint8_t IOresistor[6] {0};
  uint16_t stepsPerRevolution = 200;
  uint16_t countsPerRevolution = 32768;
}storageVars;
