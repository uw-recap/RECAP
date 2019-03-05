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
  for (int i = 0; i < 150; i++) {
    drawRiskValue(i);
  }
  for (int i = LCD_MAX_RISK; i > LCD_MIN_RISK; i--) {
    drawRiskValue(i);
  }
  while(true);
  // unsigned long start = micros();
  // int16_t potInput = analogRead(BRAKE_INPUT);
  //
  // PRINT("Analog read: ");
  // PRINTLN(potInput);
  //
  // int16_t risk = map(potInput, 0, 550, MIN_RISK, MAX_RISK);
  // int16_t lcdValue = GET_LCD_HEIGHT(MAX_RISK - risk);
  //
//   drawNewValue(0);
  // PRINT("Time to draw one value: ");
  // PRINTLN(micros() - start);
}
