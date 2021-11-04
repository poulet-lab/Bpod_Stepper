//#define DEBUG
#ifdef DEBUG
  #include <string.h>
  #define DEBUG_DELAY(x)    delay(1000)
  #define DEBUG_PRINT(x)    Serial.print(x)
  #define DEBUG_PRINTLN(x)  Serial.println(x)
  #define DEBUG_PRINTFUN(x) Serial.print(strrchr("/" __FILE__, '/') + 1); Serial.print(":"); Serial.print(__LINE__); Serial.print(":"); Serial.print(__func__); Serial.print(" "); Serial.println(x);
#else
  #define DEBUG_DELAY(x)
  #define DEBUG_PRINT(x)
  #define DEBUG_PRINTLN(x)
  #define DEBUG_PRINTFUN(x)
#endif
