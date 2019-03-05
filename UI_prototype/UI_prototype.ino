#define PRINT_SERIAL
#ifdef PRINT_SERIAL
  #define PRINT(value) Serial.print(value);
  #define PRINTLN(value) Serial.println(value);
#else
  #define PRINT(value) ;
  #define PRINTLN(value) ;
#endif

// potentiometer pin
#define BRAKE_INPUT A0

#include "LCD_helpers.h"

void setup() {
  #ifdef PRINT_SERIAL
    Serial.begin(115200);
    while(!Serial); // wait until the Serial window has been opened
  #endif

  setupLCD();
}

void loop(void) {
  // test blind spot warning
  drawBlindSpotWarningL(true);
  drawBlindSpotWarningR(true);
  delay(2000);
  drawBlindSpotWarningL(false);
  drawBlindSpotWarningR(false);

  // test forward warning
  for (int i = LCD_MIN_RISK; i < LCD_MAX_RISK; i++) {
    drawRiskValue(i);
    delay(10);
  }
  delay(500);
  for (int i = LCD_MAX_RISK; i > LCD_MIN_RISK; i--) {
    drawRiskValue(i);
    delay(10);
  }
  while(true);
}
