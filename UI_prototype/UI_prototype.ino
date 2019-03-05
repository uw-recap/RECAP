//#define PRINT_SERIAL
#ifdef PRINT_SERIAL
  #define PRINT(value) Serial.print(value);
  #define PRINTLN(value) Serial.println(value);
#else
  #define PRINT(value) ;
  #define PRINTLN(value) ;
#endif

//#define PLATFORM_ARDUINO_UNO
#define PLATFORM_FEATHER_M0

// Colors
#define BLACK 0x0000
#define WHITE 0xFFFF
#define GREEN 0x05A3
#define YELLOW 0xFFE0
#define LIGHT_ORANGE 0xFCC0
#define DARK_ORANGE 0xFBC0
#define RED 0xF800
#define DARK_RED 0xB000
#define DARK_YELLOW 0xB580

#define BG_COLOR BLACK
#define GRID_COLOR WHITE

#define BRAKE_MIN 0
#define BRAKE_MAX 100

#define BRAKE_INPUT A0

// LCD Specs
#define LCD_MIN_HEIGHT 0
#define LCD_MAX_HEIGHT 300    // the height of the graph
#define LCD_WIDTH_LEFT 300    // the width of the left side of the screen (where the past data is shown)
#define LCD_WIDTH_RIGHT 154   // the width of the right side of the screen (where the current data is shown)
#define LCD_OFFSETX_LEFT 169  // where the left side of the graph starts
#define LCD_OFFSETX_RIGHT 10  // where the right side of the graph starts
#define LCD_OFFSETY 10        // where the grid ends and the actual graph starts
#define LCD_NUM_SECTIONS 3    // how many sections the LCD screen should be split into
#define GET_LCD_HEIGHT(value) map(value, BRAKE_MIN, BRAKE_MAX, LCD_MIN_HEIGHT, LCD_MAX_HEIGHT)

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

// GLOBALS
Adafruit_HX8357 tft = Adafruit_HX8357(TFT_CS, TFT_DC, TFT_RST);
short lcdHeightFifo[LCD_WIDTH_LEFT];
short sectionHeight = (LCD_MAX_HEIGHT - LCD_MIN_HEIGHT) / LCD_NUM_SECTIONS;

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
  // short potInput = analogRead(BRAKE_INPUT);
  //
  // PRINT("Analog read: ");
  // PRINTLN(potInput);
  //
  // short brakeValue = map(potInput, 0, 550, BRAKE_MIN, BRAKE_MAX);
  // short lcdValue = GET_LCD_HEIGHT(BRAKE_MAX - brakeValue);
  //
  // drawNewValue(lcdValue);
  // PRINT("Time to draw one value: ");
  // PRINTLN(micros() - start);
}

// expects a value between LCD_HEIGHT_MIN and LCD_HEIGHT_MAX
void drawNewValue(short newValue) {
  short oldValue = lcdHeightFifo[0];
  short diff = newValue - oldValue;
  short i;
  uint16_t color;

  // draw the section on the right
  if (diff > 0) {
    for (i = 0; i < diff; i++) {
      tft.drawFastHLine(LCD_OFFSETX_RIGHT, LCD_OFFSETY + oldValue + i, LCD_WIDTH_RIGHT, RED);
    }
  } else {
    for (i = 0; i > diff; i--) {
      tft.drawFastHLine(LCD_OFFSETX_RIGHT, LCD_OFFSETY + oldValue + i, LCD_WIDTH_RIGHT, BG_COLOR);
    }
  }

  // draw the section on the left while shifting all elements in the FIFO one down
  for (short j = LCD_WIDTH_LEFT - 1; j >= 0; j--) {
    oldValue = lcdHeightFifo[j];

    // if j == 0, then we're pushing the latest value to the beginning of the FIFO
    // otherwise, we're just shifting the FIFO values down one
    short currentValue = (j == 0) ? newValue : lcdHeightFifo[j-1];
    diff = currentValue - oldValue; // if diff > 0, draw in color, starting from old_val. if diff < 0, draw in black, starting from old_val + diff

    lcdHeightFifo[j] = currentValue;
    if (diff > 0) {
      color = pickColor(j);
      tft.drawFastVLine(LCD_OFFSETX_LEFT + j, LCD_OFFSETY + oldValue, diff, color);
    } else {
      tft.drawFastVLine(LCD_OFFSETX_LEFT + j, LCD_OFFSETY + oldValue + diff, -diff, BG_COLOR);
    }
  }
}

void drawStaticImages() {
  // vertical separator
  for (short i = 0; i < 5; i++) {
    tft.drawFastVLine(LCD_OFFSETX_RIGHT + LCD_WIDTH_RIGHT + i, 0, 320, GRID_COLOR);
  }
  
  // draw the red and yellow circles (not active)
  tft.fillCircle(80, 230, 50, DARK_RED);
  tft.fillCircle(80, 90, 50, DARK_YELLOW);

  drawCar();
  
  // draw forward warning area
  tft.drawLine(299, 170, 279, 302, GRID_COLOR);
  tft.drawLine(341, 170, 361, 302, GRID_COLOR);
  tft.drawLine(279, 302, 361, 302, GRID_COLOR);

  drawBlindSpotWarningL(false);
  drawBlindSpotWarningR(false);
}

void drawCar() {
  // bottom
  tft.drawLine(299, 54, 341, 54, GRID_COLOR);
  tft.drawLine(299, 55, 341, 55, GRID_COLOR);
  // top
  tft.drawLine(299, 169, 341, 169, GRID_COLOR);
  tft.drawLine(299, 170, 341, 170, GRID_COLOR);
  // left side
  tft.drawLine(299, 54, 299, 170, GRID_COLOR);
  tft.drawLine(300, 54, 300, 170, GRID_COLOR);
  // right side
  tft.drawLine(340, 54, 340, 170, GRID_COLOR);
  tft.drawLine(341, 54, 341, 170, GRID_COLOR);
  // front windshield
  tft.drawLine(305, 145, 335, 145, GRID_COLOR);
  tft.drawLine(335, 145, 332, 130, GRID_COLOR);
  tft.drawLine(332, 130, 308, 130, GRID_COLOR);
  tft.drawLine(308, 130, 305, 145, GRID_COLOR);
  // roof
  tft.drawLine(308, 128, 332, 128, GRID_COLOR);
  tft.drawLine(332, 128, 332, 98, GRID_COLOR);
  tft.drawLine(332, 98, 308, 98, GRID_COLOR);
  tft.drawLine(308, 98, 308, 128, GRID_COLOR);
  // rear windshield
  tft.drawLine(308, 96, 332, 96, GRID_COLOR);
  tft.drawLine(332, 96, 335, 81, GRID_COLOR);
  tft.drawLine(335, 81, 305, 81, GRID_COLOR);
  tft.drawLine(305, 81, 308, 96, GRID_COLOR);
  // mirrors
  tft.drawLine(341, 130, 346, 130, GRID_COLOR);
  tft.drawLine(299, 130, 294, 130, GRID_COLOR);
}

void drawBlindSpotWarningL(bool active) {
  tft.drawLine(341, 129, 396, 121, active ? LIGHT_ORANGE : GRID_COLOR);
  tft.drawLine(341, 129, 391, 30, active ? LIGHT_ORANGE : GRID_COLOR);
  // TODO: implement active signal
}

void drawBlindSpotWarningR(bool active) {
  tft.drawLine(299, 129, 244, 121, active ? LIGHT_ORANGE : GRID_COLOR);
  tft.drawLine(299, 129, 249, 30, active ? LIGHT_ORANGE : GRID_COLOR);  
  // TODO: implement active signal
}

// switches colors based on lcd value
uint16_t pickColor(short leftDistance) {
  // this maps a value between 0-300 to a 16-bit color gradient.
  // but only for red. don't ask.
  return RED - (leftDistance / 30) * 0x1800 - (((leftDistance / 3) % 10) / 3) * 0x0800;

  // well, since you asked...
  // here's an color value (as in HSV value) table for solid red (i.e. H = 0 and S = 100)
  // |   V | => | 16-bit |
  // |-----|----|--------|
  // | 100 | => | 0xF800 |
  // |  99 | => | 0xF800 |
  // |  98 | => | 0xF800 |

  // subtract 0x0800...

  // |  97 | => | 0xF000 |
  // |  96 | => | 0xF000 |
  // |  95 | => | 0xF000 |

  // subtract 0x0800 again...

  // |  94 | => | 0xE800 |
  // |  93 | => | 0xE800 |
  // |  92 | => | 0xE800 |
  // |  91 | => | 0xE800 |

  // subtract 0x0800 again...

  // |  90 | => | 0xE000 |

  // So for every 3 (or 4) steps of V, the 16-bit value changes by 0x1800

  // |  80 | => | 0xC800 |
  // |  70 | => | 0xB000 |
  // | ... | => | ...    |
  // |  10 | => | 0x1800 |
  // |   0 | => | 0x0000 |
}
