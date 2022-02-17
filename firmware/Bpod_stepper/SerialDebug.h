#define DEBUG
#ifdef DEBUG
  #include <string.h>
  #include <time.h>
  #define DEBUG_DELAY(x)    delay(1000)
  #define DEBUG_PRINT(x)    Serial.print(x)
  #define DEBUG_PRINTF(...) Serial.printf("%010d   ", micros()); Serial.printf(__VA_ARGS__)
  #define DEBUG_PRINTLN(x)  Serial.printf("%010d   ", micros()); Serial.println(x)
  #define DEBUG_PRINTFUN(x) Serial.printf("%010d   ", micros()); Serial.print(strrchr("/" __FILE__, '/') + 1); Serial.print(":"); Serial.print(__LINE__); Serial.print(":"); Serial.print(__func__); Serial.print(" "); Serial.println(x)
  #define DEBUG_MICROS()    Serial.printf("%010d   ", micros());
#else
  #define DEBUG_DELAY(x)
  #define DEBUG_PRINT(x)
  #define DEBUG_PRINTF(...)
  #define DEBUG_PRINTLN(x)
  #define DEBUG_PRINTFUN(x)
  #define DEBUG_MICROS()
#endif
