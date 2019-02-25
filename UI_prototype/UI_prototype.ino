//#define PLATFORM_ARDUINO_UNO
#define PLATFORM_FEATHER_M0

#define BG_COLOR BLACK
#define GRID_COLOR WHITE

#define BRAKE_MIN 0
#define BRAKE_MAX 100

#define BRAKE_INPUT A0

// LCD Specs
#define LCD_MIN_HEIGHT 0
#define LCD_MAX_HEIGHT 300
#define LCD_WIDTH_LEFT 300
#define LCD_WIDTH_RIGHT 154
#define LCD_OFFSETX_LEFT 169
#define LCD_OFFSETX_RIGHT 10
#define LCD_OFFSETY 10
#define GET_LCD_HEIGHT(value) map(value, BRAKE_MIN, BRAKE_MAX, LCD_MIN_HEIGHT, LCD_MAX_HEIGHT)

// Colors
#define BLACK 0x0000
#define WHITE 0xFFFF
#define GREEN 0x6FE0
#define YELLOW 0xFFE0
#define LIGHT_ORANGE 0xFCC0
#define DARK_ORANGE 0xFBC0
#define RED 0xF800

#include <SPI.h>
#include "Adafruit_GFX.h"
#include "Adafruit_HX8357.h"

// SPI pinouts
#ifdef PLATFORM_ARDUINO_UNO
#define TFT_CS 10
#define TFT_DC 9
#define TFT_RST 8 // RST can be set to -1 if you tie it to Arduino's reset
#endif
#ifdef PLATFORM_FEATHER_M0
#define TFT_CS 9
#define TFT_DC 10
#define TFT_RST -1 // RST can be set to -1 if you tie it to Arduino's reset
#endif

// Use hardware SPI (on Uno, #13, #12, #11) and the above for CS/DC
Adafruit_HX8357 tft = Adafruit_HX8357(TFT_CS, TFT_DC, TFT_RST);

// SoftSPI - note that on some processors this might be *faster* than hardware SPI!
//Adafruit_HX8357 tft = Adafruit_HX8357(TFT_CS, TFT_DC, MOSI, SCK, TFT_RST, MISO);

// may be slow. consider writing a Fifo class for optimization
short lcdHeightFifo[LCD_WIDTH_LEFT + 1];

void setup() {
  Serial.begin(115200);
  while(!Serial);
//  Serial.println("HX8357D Test!"); 

  tft.begin(HX8357D);
  tft.setRotation(3);
  tft.fillScreen(BG_COLOR);
  drawFullGrid(GRID_COLOR);
  delay(500);

  // init LCD values array
  for (int i = 0; i < LCD_WIDTH_LEFT; i++) {
    lcdHeightFifo[i] = 0;
  }
}

void loop(void) {
    unsigned long start = micros();
    short potInput = analogRead(BRAKE_INPUT);
        
    Serial.print("Analog read: ");
    Serial.println(potInput);
    
    short brakeValue = map(potInput, 0, 550, BRAKE_MIN, BRAKE_MAX);  
    short lcdValue = GET_LCD_HEIGHT(BRAKE_MAX - brakeValue);

    drawNewValue(lcdValue);
    Serial.print("Time to draw one value: ");
    Serial.println(micros() - start);
}

// expects a value between LCD_HEIGHT_MIN and LCD_HEIGHT_MAX
void drawNewValue(short value) {
  short old_value = lcdHeightFifo[0];
  short diff = value - old_value;

  // draw the section on the right
  if (diff > 0) {
    tft.fillRect(LCD_OFFSETX_RIGHT, LCD_OFFSETY + old_value, LCD_WIDTH_RIGHT, diff, pickColor(value));
  } else {
    // draw lines "downwards"
    for (int i = 0; i > diff; i--) {
      tft.drawFastHLine(LCD_OFFSETX_RIGHT, LCD_OFFSETY + old_value + i, LCD_WIDTH_RIGHT, BG_COLOR);
    }
  }   
  drawMinorHGridRight(GRID_COLOR);

  // draw the section on the left and shift all elements in the FIFO one down
  for (int j = LCD_WIDTH_LEFT; j > 0; j--) {
    old_value = lcdHeightFifo[j]; // new value - old value
    short current_value = lcdHeightFifo[j-1];
    diff = current_value - old_value; // if diff > 0, draw in color, starting from old_val. if diff < 0, draw in black, starting from old_val + diff
    
    lcdHeightFifo[j] = current_value;
    if (diff > 0) {
      tft.drawFastVLine(LCD_OFFSETX_LEFT + j, LCD_OFFSETY + old_value, diff, pickColor(old_value));
    } else {
      tft.drawFastVLine(LCD_OFFSETX_LEFT + j, LCD_OFFSETY + old_value + diff, -diff, BG_COLOR);  
    }      
  }

  // push the latest element to the front of the FIFO
  lcdHeightFifo[0] = value;        
  tft.drawFastVLine(LCD_OFFSETX_LEFT, LCD_OFFSETY, value, pickColor(value));
  drawMinorHGridLeft(GRID_COLOR);

}

void drawMinorHGridRight(uint16_t gridColor) {
  for (int i = 1; i <= 5; i++) {
    tft.drawFastHLine(10, i*50 + 9, 154, gridColor);
  }
}

void drawMinorHGridLeft(uint16_t gridColor) {
  for (int i = 1; i <= 5; i++) {
    tft.drawFastHLine(169, i*50 + 9, 300, gridColor);
  }
}

void drawFullGrid(uint16_t gridColor) {
  // bottom horizontal
  tft.drawFastHLine(8, 8, 464, gridColor);
  tft.drawFastHLine(8, 9, 464, gridColor);

  // top horizontal
  tft.drawFastHLine(8, 310, 464, gridColor);
  tft.drawFastHLine(8, 311, 464, gridColor);

  // left vertical
  tft.drawFastVLine(8, 10, 300, gridColor);
  tft.drawFastVLine(9, 10, 300, gridColor);

  // right vertical
  tft.drawFastVLine(470, 10, 300, gridColor);
  tft.drawFastVLine(471, 10, 300, gridColor);
 
  // horizontal minor axes
  drawMinorHGridRight(gridColor);
  drawMinorHGridLeft(gridColor);
  
  for (int i = 0; i < 5; i++) {
    // vertical "minor" axis
    tft.drawFastVLine(164 + i, 10, 300, gridColor);
  }
}

// switches colors based on lcd value
uint16_t pickColor(short lcdValue) {
  if (lcdValue < 100) return GREEN;
  else if (lcdValue < 150) return YELLOW;
  else if (lcdValue < 200) return LIGHT_ORANGE;
  else if (lcdValue < 250) return DARK_ORANGE;
  else return RED;  
}
