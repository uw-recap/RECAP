#include "LCD_helpers.h"

// GLOBALS
Adafruit_HX8357 tft = Adafruit_HX8357(TFT_CS, TFT_DC, TFT_RST);
uint8_t lcdHeightFifo[LCD_WIDTH_LEFT];
uint8_t sectionHeight = (LCD_MAX_HEIGHT - LCD_MIN_HEIGHT) / LCD_NUM_SECTIONS;

void setupLCD() {
  tft.begin(16E6);
  tft.setRotation(3);
  tft.fillScreen(BG_COLOR);
  drawStaticImages();
  delay(100);
}

// expects a value between LCD_HEIGHT_MIN and LCD_HEIGHT_MAX (defined in LCD_helpers.h)
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
  
  drawRiskCircles(MIN_RISK); // draw the red and yellow circles (not active)
  drawCar();
  
  // draw forward warning area
  tft.drawLine(299, 170, 279, 302, GRID_COLOR);
  tft.drawLine(341, 170, 361, 302, GRID_COLOR);
  tft.drawLine(279, 302, 361, 302, GRID_COLOR);

  drawBlindSpotWarningL(false);
  drawBlindSpotWarningR(false);
}

void drawRiskCircles(uint8_t maxRisk) {
  // always draws the circles. Might be optimizable by 
  // storing the previous state of the circles somewhere
  
  if (maxRisk > 50) {
    // draw bright yellow circle
    tft.fillCircle(80, 90, 50, YELLOW);
  } else {
    // draw dull yellow circle
    tft.fillCircle(80, 90, 50, DARK_YELLOW);
  }

  if (maxRisk > 80) {
    // draw bright red circle
    tft.fillCircle(80, 230, 50, RED);
  } else {
    // draw dull red circle
    tft.fillCircle(80, 230, 50, DARK_RED);
  }
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
