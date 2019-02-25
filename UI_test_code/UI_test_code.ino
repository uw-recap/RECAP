//#define PLATFORM_ARDUINO_UNO
#define PLATFORM_FEATHER_M0

#define BG_COLOR BLACK
#define GRID_COLOR WHITE

#define BRAKE_MIN 0
#define BRAKE_MAX 100

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
  Serial.println("HX8357D Test!"); 

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
  benchmarkDrawingMain();
}

void benchmarkDrawingMain() {
  Serial.println("Running LS and RS diff once before so that the test is fair");
  benchmarkDrawing();
  delay(5000);

  Serial.println("Hopefully better looking RS erasing");
  benchmarkDrawing();  
//  delay(5000);

//  Serial.println("Testing LS and RS diff");
//  testDrawingWithLSAndRSDiff();
//  delay(5000);
  
//  Serial.println("Testing LS diff only");
//  testDrawingWithLSDiff();
//  delay(5000);
//  
//  Serial.println("Testing unoptimized code");
//  testUnoptimizedDrawing();

  while(1);
}

// edit this to change the function we're benchmarking against
short generateLcdTestValue(short x, short oldValue) {
  // full screen
//  return LCD_MAX_HEIGHT;

//  // empty screen
//  return LCD_MIN_HEIGHT;

  // linear increase
//  return x;

  // linear decrease
//  return LCD_MAX_HEIGHT - x;

  // random
//  return rand() % LCD_MAX_HEIGHT;

  // random diff (30% of full scale)
  float fractionFS = 0.3;
  short diff = rand() % ((int) ((double) LCD_MAX_HEIGHT * 2.0 * fractionFS)) - ((int) ((double) LCD_MAX_HEIGHT * fractionFS));
  short newValue = oldValue - diff;
  if (newValue < LCD_MIN_HEIGHT) return LCD_MIN_HEIGHT;
  if (newValue > LCD_MAX_HEIGHT) return LCD_MAX_HEIGHT;
  return newValue; 
}

// expects a value between LCD_HEIGHT_MIN and LCD_HEIGHT_MAX
void drawNewValue(short value) {
  short old_value = lcdHeightFifo[0];
  short diff = value - old_value;

  // draw the section on the right
  if (diff > 0) {
    tft.fillRect(LCD_OFFSETX_RIGHT, LCD_OFFSETY + old_value, LCD_WIDTH_RIGHT, diff, RED);
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
      tft.drawFastVLine(LCD_OFFSETX_LEFT + j, LCD_OFFSETY + old_value, diff, DARK_ORANGE);
    } else {
      tft.drawFastVLine(LCD_OFFSETX_LEFT + j, LCD_OFFSETY + old_value + diff, -diff, BG_COLOR);  
    }      
  }

  // push the latest element to the front of the FIFO
  lcdHeightFifo[0] = value;        
  tft.drawFastVLine(LCD_OFFSETX_LEFT, LCD_OFFSETY, value, LIGHT_ORANGE);
  drawMinorHGridLeft(GRID_COLOR);

}

void benchmarkDrawing() {
  unsigned long start;
  
  // draw empty screen (baseline test)
  for (int i = 0; i < 5; i++) {
    unsigned long start = micros();
    drawNewValue(LCD_MIN_HEIGHT);
    Serial.print("Baseline value: ");
    Serial.println(micros() - start);
  }

  short oldValue = 0;
  // draw full screen (left to right)
  for (short i = 0; i <= LCD_MAX_HEIGHT; i++) {
    start = micros();
    short newValue = generateLcdTestValue(i, oldValue);
    drawNewValue(newValue);
    oldValue = newValue;
    Serial.println(micros() - start);
  }  
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

unsigned long testFillScreen() {
  unsigned long start = micros();
  tft.fillScreen(HX8357_RED);
  tft.fillScreen(HX8357_GREEN);
  tft.fillScreen(HX8357_BLUE);
  tft.fillScreen(HX8357_WHITE);
  return micros() - start;
}

unsigned long testLines(uint16_t color) {
  unsigned long start, t;
  int           x1, y1, x2, y2,
                w = tft.width(),
                h = tft.height();

  tft.fillScreen(HX8357_BLACK);

  x1 = y1 = 0;
  y2    = h - 1;
  start = micros();
  for(x2=0; x2<w; x2+=6) tft.drawLine(x1, y1, x2, y2, color);
  x2    = w - 1;
  for(y2=0; y2<h; y2+=6) tft.drawLine(x1, y1, x2, y2, color);
  t     = micros() - start; // fillScreen doesn't count against timing


  return micros() - start;
}

unsigned long testFastLines(uint16_t color1, uint16_t color2) {
  unsigned long start;
  int           x, y, w = tft.width(), h = tft.height();

  tft.fillScreen(HX8357_BLACK);
  start = micros();
  for(y=0; y<h; y+=5) tft.drawFastHLine(0, y, w, color1);
  for(x=0; x<w; x+=5) tft.drawFastVLine(x, 0, h, color2);

  return micros() - start;
}

/************* UNOPTIMIZED VERSIONS OF CODE **************/
//// expects a value between LCD_HEIGHT_MIN and LCD_HEIGHT_MAX
//void drawUnoptimized(short value) {
//  // draw the section on the right
//  tft.fillRect(LCD_OFFSETX_RIGHT, LCD_OFFSETY, LCD_WIDTH_RIGHT, value, LIGHT_ORANGE);
//  drawMinorHGridLeft(GRID_COLOR);
//
//  // draw the section on the left and shift all elements in the FIFO one down
//  for (int j = LCD_WIDTH_LEFT; j > 0; j--) {
//    lcdHeightFifo[j] = lcdHeightFifo[j-1];
//    tft.drawFastVLine(LCD_OFFSETX_LEFT + j, LCD_OFFSETY, lcdHeightFifo[j], LIGHT_ORANGE);
//  }
//
//  // push the latest element to the front of the FIFO
//  lcdHeightFifo[0] = value;        
//  tft.drawFastVLine(LCD_OFFSETX_LEFT, LCD_OFFSETY, value, LIGHT_ORANGE);
//  drawMinorHGridRight(GRID_COLOR);
//}
//
//// expects a value between LCD_HEIGHT_MIN and LCD_HEIGHT_MAX
//void drawWithLSDiff(short value) {
//  // draw the section on the right
//  tft.fillRect(LCD_OFFSETX_RIGHT, LCD_OFFSETY, LCD_WIDTH_RIGHT, value, LIGHT_ORANGE);
//  drawMinorHGridLeft(GRID_COLOR);
//
//  // draw the section on the left and shift all elements in the FIFO one down
//  for (int j = LCD_WIDTH_LEFT; j > 0; j--) {
//    short old_val = lcdHeightFifo[j]; // new value - old value
//    short new_val = lcdHeightFifo[j-1];
//    short diff = new_val - old_val; // if diff > 0, draw in color, starting from old_val. if diff < 0, draw in black, starting from old_val + diff
//    
//    lcdHeightFifo[j] = new_val;
//    if (diff > 0) {
//      tft.drawFastVLine(LCD_OFFSETX_LEFT + j, LCD_OFFSETY + old_val, diff, LIGHT_ORANGE);
//    } else {
//      tft.drawFastVLine(LCD_OFFSETX_LEFT + j, LCD_OFFSETY + old_val + diff, -diff, BG_COLOR);  
//    }      
//  }
//
//  // push the latest element to the front of the FIFO
//  lcdHeightFifo[0] = value;        
//  tft.drawFastVLine(LCD_OFFSETX_LEFT, LCD_OFFSETY, value, LIGHT_ORANGE);
//  drawMinorHGridRight(GRID_COLOR);
//}
//
//
//void testUnoptimizedDrawing() {
//  unsigned long start;
//  
//  // draw empty screen (baseline test)
//  for (int i = 0; i < 5; i++) {
//    unsigned long start = micros();
//    drawWithLSAndRSDiff(LCD_MIN_HEIGHT);
//    Serial.print("Baseline value: ");
//    Serial.println(micros() - start);
//  }
//    
//  // draw full screen (left to right)
//  for (short i = 0; i <= LCD_MAX_HEIGHT; i++) {
//    start = micros();
//    drawUnoptimized(generateLcdTestValue(i));
//    Serial.println(micros() - start);
//  }
//}
//
//void testDrawingWithLSDiff() {
//  unsigned long start;
//  
//  // draw empty screen (baseline test)
//  for (int i = 0; i < 5; i++) {
//    unsigned long start = micros();
//    drawWithLSAndRSDiff(LCD_MIN_HEIGHT);
//    Serial.print("Baseline value: ");
//    Serial.println(micros() - start);
//  }
//    
//  // draw full screen (left to right)
//  for (short i = 0; i <= LCD_MAX_HEIGHT; i++) {
//    start = micros();
//    drawWithLSDiff(generateLcdTestValue(i));
//    Serial.println(micros() - start);
//  }  
//}
//
//// expects a value between LCD_HEIGHT_MIN and LCD_HEIGHT_MAX
//void drawWithLSAndRSDiff(short value) {
//  short old_value = lcdHeightFifo[0];
//  short diff = value - old_value;
//
//  // draw the section on the right
//  if (diff > 0) {
//    tft.fillRect(LCD_OFFSETX_RIGHT, LCD_OFFSETY + old_value, LCD_WIDTH_RIGHT, diff, LIGHT_ORANGE);
//  } else {
//    tft.fillRect(LCD_OFFSETX_RIGHT, LCD_OFFSETY + old_value + diff, LCD_WIDTH_RIGHT, -diff, BG_COLOR);
//  }   
//  drawMinorHGridLeft(GRID_COLOR);
//
//  // draw the section on the left and shift all elements in the FIFO one down
//  for (int j = LCD_WIDTH_LEFT; j > 0; j--) {
//    old_value = lcdHeightFifo[j]; // new value - old value
//    short current_value = lcdHeightFifo[j-1];
//    diff = current_value - old_value; // if diff > 0, draw in color, starting from old_val. if diff < 0, draw in black, starting from old_val + diff
//    
//    lcdHeightFifo[j] = current_value;
//    if (diff > 0) {
//      tft.drawFastVLine(LCD_OFFSETX_LEFT + j, LCD_OFFSETY + old_value, diff, LIGHT_ORANGE);
//    } else {
//      tft.drawFastVLine(LCD_OFFSETX_LEFT + j, LCD_OFFSETY + old_value + diff, -diff, BG_COLOR);  
//    }      
//  }
//
//  // push the latest element to the front of the FIFO
//  lcdHeightFifo[0] = value;        
//  tft.drawFastVLine(LCD_OFFSETX_LEFT, LCD_OFFSETY, value, LIGHT_ORANGE);
//  drawMinorHGridRight(GRID_COLOR);
//}
//
//void testDrawingWithLSAndRSDiff() {
//  unsigned long start;
//  
//  // draw empty screen (baseline test)
//  for (int i = 0; i < 5; i++) {
//    unsigned long start = micros();
//    drawWithLSAndRSDiff(LCD_MIN_HEIGHT);
//    Serial.print("Baseline value: ");
//    Serial.println(micros() - start);
//  }
//    
//  // draw full screen (left to right)
//  for (short i = 0; i <= LCD_MAX_HEIGHT; i++) {
//    start = micros();
//    drawWithLSAndRSDiff(generateLcdTestValue(i));
//    Serial.println(micros() - start);
//  }  
//}
