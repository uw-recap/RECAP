//#define PRINT_SERIAL
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

// GLOBALS
Adafruit_HX8357 tft = Adafruit_HX8357(TFT_CS, TFT_DC, TFT_RST);
uint8_t lcdHeightFifo[LCD_WIDTH_LEFT];
uint8_t sectionHeight = (LCD_MAX_HEIGHT - LCD_MIN_HEIGHT) / LCD_NUM_SECTIONS;

void setup() {
  #ifdef PRINT_SERIAL
    Serial.begin(115200);
    while(!Serial); // wait until the Serial window has been opened
  #endif

  tft.begin(HX8357D);
  tft.setRotation(3);
  tft.fillScreen(BG_COLOR);
  drawStaticImages();
  delay(100);
}

void loop(void) {
  // unsigned long start = micros();
  // uint8_t potInput = analogRead(BRAKE_INPUT);
  //
  // PRINT("Analog read: ");
  // PRINTLN(potInput);
  //
  // uint8_t risk = map(potInput, 0, 550, MIN_RISK, MAX_RISK);
  // uint8_t lcdValue = GET_LCD_HEIGHT(MAX_RISK - risk);
  //
//   drawNewValue(0);
  // PRINT("Time to draw one value: ");
  // PRINTLN(micros() - start);
}
