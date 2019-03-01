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
      
  PRINT("Analog read: ");
  PRINTLN(potInput);
  
  short brakeValue = map(potInput, 0, 550, BRAKE_MIN, BRAKE_MAX);  
  short lcdValue = GET_LCD_HEIGHT(BRAKE_MAX - brakeValue);

  drawNewValue(lcdValue);
  PRINT("Time to draw one value: ");
  PRINTLN(micros() - start);
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

void drawFullGrid(uint16_t gridColor) {
// These are defined here because they're not relevant to the rest of the program
#define LCD_GRID_START_X 8
#define LCD_GRID_START_Y 8
#define LCD_GRID_HEIGHT  303
#define LCD_GRID_WIDTH   462
#define LCD_GRID_BORDER  2
  
  // bottom horizontal
  for (short i = 0; i < LCD_GRID_BORDER; i++) {
    tft.drawFastHLine(LCD_GRID_START_X, LCD_GRID_START_Y + i, LCD_GRID_WIDTH, gridColor);
  }

  // top horizontal
  for (short i = 0; i < LCD_GRID_BORDER; i++) {
    tft.drawFastHLine(LCD_GRID_START_X, LCD_GRID_START_Y + LCD_GRID_HEIGHT - i, LCD_GRID_WIDTH, gridColor);
  }

  // right vertical
  for (short i = 0; i < LCD_GRID_BORDER; i++) {
    tft.drawFastVLine(LCD_GRID_START_X + i, LCD_GRID_START_Y, LCD_GRID_HEIGHT, gridColor);
  }

  // left vertical
  for (short i = 0; i < LCD_GRID_BORDER; i++) {
    tft.drawFastVLine(LCD_GRID_START_X + LCD_GRID_WIDTH - i, LCD_GRID_START_Y, LCD_GRID_HEIGHT, gridColor);
  }
   
  for (short i = 0; i < 5; i++) {
    // vertical separator
    tft.drawFastVLine(LCD_OFFSETX_RIGHT + LCD_WIDTH_RIGHT + i, 10, 300, gridColor);
  }
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
