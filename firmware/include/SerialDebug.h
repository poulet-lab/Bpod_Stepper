#pragma once

#ifdef DEBUG
  #include <string.h>
  #include <time.h>
  #define DEBUG_WAIT()      for (uint8_t i = 0; i < 20; i++) { if (SerialUSB1) break; delay(100); }
  #define DEBUG_DELAY(x)    delay(x)
  #define DEBUG_PRINT(x)    SerialUSB1.print(x)
  #define DEBUG_PRINTF(...) SerialUSB1.printf("%010d   ", micros()); SerialUSB1.printf(__VA_ARGS__)
  #define DEBUG_PRINTLN(x)  SerialUSB1.printf("%010d   ", micros()); SerialUSB1.println(x)
  #define DEBUG_PRINTFUN(x) SerialUSB1.printf("%010d   ", micros()); SerialUSB1.print(strrchr("/" __FILE__, '/') + 1); SerialUSB1.print(":"); SerialUSB1.print(__LINE__); SerialUSB1.print(":"); SerialUSB1.print(__func__); SerialUSB1.print(" "); SerialUSB1.println(x)
  #define DEBUG_MICROS()    SerialUSB1.printf("%010d   ", micros());
#else
  #define DEBUG_WAIT()
  #define DEBUG_DELAY(x)
  #define DEBUG_PRINT(x)
  #define DEBUG_PRINTF(...)
  #define DEBUG_PRINTLN(x)
  #define DEBUG_PRINTFUN(x)
  #define DEBUG_MICROS()
#endif
