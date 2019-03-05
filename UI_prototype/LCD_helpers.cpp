#include "LCD_helpers.h"

/*********** GLOBALS ***********/
Adafruit_HX8357 tft = Adafruit_HX8357(TFT_CS, TFT_DC, TFT_RST);
int16_t fwWarningLeftBound[LCD_RISK_HEIGHT][2];
int16_t fwWarningRightBound[LCD_RISK_HEIGHT][2];
int16_t currentDisplayedRisk;

/*********** "PRIVATE" FUNCTIONS ***********/
// assume riskValue is between LCD_MIN_RISK and LCD_MAX_RISK
void drawRiskCircles(int16_t riskValue) {
  if (riskValue > LCD_MAX_RISK || riskValue < LCD_MIN_RISK) return;
  
  // always draws the circles. Might be optimizable by 
  // storing the previous state of the circles somewhere

  // light up YELLOW light if risk > 50% max risk
  if (riskValue > MED_RISK && currentDisplayedRisk < MED_RISK) {
    // draw bright yellow circle
    tft.fillCircle(80, 90, 50, YELLOW);
  } else if (riskValue < MED_RISK && currentDisplayedRisk > MED_RISK) {
    // draw dull yellow circle
    tft.fillCircle(80, 90, 50, DARK_YELLOW);
  }

  // light up RED light if risk > 75% max risk
  if (riskValue > HIGH_RISK && currentDisplayedRisk < HIGH_RISK) {
    // draw bright red circle
    tft.fillCircle(80, 230, 50, RED);
  } else if (riskValue < HIGH_RISK && currentDisplayedRisk > HIGH_RISK) {
    // draw dull red circle
    tft.fillCircle(80, 230, 50, DARK_RED);
  }
}

void drawCar() {
  // bottom
  tft.drawLine(299, 34, 341, 34, GRID_COLOR);
  tft.drawLine(299, 35, 341, 35, GRID_COLOR);
  // top
  tft.drawLine(299, 149, 341, 149, GRID_COLOR);
  tft.drawLine(299, 150, 341, 150, GRID_COLOR);
  // left side
  tft.drawLine(299, 34, 299, 150, GRID_COLOR);
  tft.drawLine(300, 34, 300, 150, GRID_COLOR);
  // right side
  tft.drawLine(340, 34, 340, 150, GRID_COLOR);
  tft.drawLine(341, 34, 341, 150, GRID_COLOR);
  // front windshield
  tft.drawLine(305, 125, 335, 125, GRID_COLOR);
  tft.drawLine(335, 125, 332, 110, GRID_COLOR);
  tft.drawLine(332, 110, 308, 110, GRID_COLOR);
  tft.drawLine(308, 110, 305, 125, GRID_COLOR);
  // roof
  tft.drawLine(308, 108, 332, 108, GRID_COLOR);
  tft.drawLine(332, 108, 332, 78, GRID_COLOR);
  tft.drawLine(332, 78, 308, 78, GRID_COLOR);
  tft.drawLine(308, 78, 308, 108, GRID_COLOR);
  // rear windshield
  tft.drawLine(308, 76, 332, 76, GRID_COLOR);
  tft.drawLine(332, 76, 335, 61, GRID_COLOR);
  tft.drawLine(335, 61, 305, 61, GRID_COLOR);
  tft.drawLine(305, 61, 308, 76, GRID_COLOR);
  // mirrors
  tft.drawLine(341, 110, 346, 110, GRID_COLOR);
  tft.drawLine(299, 110, 294, 110, GRID_COLOR);
}

// bresenham algorithm taken straight from Arduino GFX library with minor modifications
void storeLineCoordinates(int16_t x0, int16_t y0, int16_t x1, int16_t y1, 
  int16_t storage[LCD_RISK_HEIGHT][2]) 
{
  int16_t steep = abs(y1 - y0) > abs(x1 - x0);
  if (steep) {
      _swap_int16_t(x0, y0);
      _swap_int16_t(x1, y1);
  }

  if (x0 > x1) {
      _swap_int16_t(x0, x1);
      _swap_int16_t(y0, y1);
  }

  int16_t dx, dy;
  dx = x1 - x0;
  dy = abs(y1 - y0);

  int16_t err = dx / 2;
  int16_t ystep;

  if (y0 < y1) {
      ystep = 1;
  } else {
      ystep = -1;
  }

  uint16_t counter = 0;
  for (; x0<=x1; x0++) {
      if (steep) {
        if (counter == 0 || x0 != storage[counter - 1][1]) {
          storage[counter][0] = y0;
          storage[counter][1] = x0;
          counter++;
        }
      } else {
        if (counter == 0 || y0 != storage[counter - 1][1]) {
          storage[counter][0] = x0;
          storage[counter][1] = y0;
          counter++;
        }
      }
      err -= dy;
      if (err < 0) {
          y0 += ystep;
          err += dx;
      }
  }
}

void drawStaticImages() {
  // vertical separator
  for (uint8_t i = 0; i < 5; i++) {
    tft.drawFastVLine(LCD_OFFSETX_RIGHT + LCD_WIDTH_RIGHT + i, 0, 320, GRID_COLOR);
  }
  
  drawRiskCircles(LCD_MIN_RISK); // draw the red and yellow circles (not active)
  drawCar();
  
  // draw forward warning area
  tft.drawLine(299, 150, 279, 301, GRID_COLOR);
  tft.drawLine(341, 150, 361, 301, GRID_COLOR);
  tft.drawLine(279, 301, 361, 301, GRID_COLOR);

  storeLineCoordinates(279, 300, 299, 150, fwWarningRightBound);
  storeLineCoordinates(361, 300, 341, 150, fwWarningLeftBound);

  drawBlindSpotWarningL(false);
  drawBlindSpotWarningR(false);
}

// switches colors based on lcd value
uint16_t pickColor(int16_t leftDistance) {
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

/*********** PUBLIC API ***********/
void setupLCD() {
  tft.begin(16E6);
  tft.setRotation(3);
  tft.fillScreen(BG_COLOR);
  currentDisplayedRisk = LCD_MAX_RISK;
  drawStaticImages();
  currentDisplayedRisk = LCD_MIN_RISK;
  delay(100);
}

// risk value should be between LCD_MIN_RISK and LCD_MAX_RISK to proceed
void drawRiskValue(int16_t riskValue) {
  if (riskValue > LCD_MAX_RISK || riskValue < LCD_MIN_RISK) return;
  
  int16_t diff = riskValue - currentDisplayedRisk;
  if (diff > 0) {
    for (int i = currentDisplayedRisk; i < riskValue; i++) {
      int displayIndex = LCD_RISK_HEIGHT - 1 - i;
      tft.drawFastHLine(fwWarningRightBound[displayIndex][0] + 1, fwWarningRightBound[displayIndex][1] + 1, 
      (fwWarningLeftBound[displayIndex][0] - fwWarningRightBound[displayIndex][0] - 1), RED);
    }
  } else {
    for(int i = currentDisplayedRisk; i > riskValue; i--) {
      int displayIndex = LCD_RISK_HEIGHT - 1 - i;
      tft.drawFastHLine(fwWarningRightBound[displayIndex][0] + 1, fwWarningRightBound[displayIndex][1] + 1, 
      (fwWarningLeftBound[displayIndex][0] - fwWarningRightBound[displayIndex][0] - 1), BG_COLOR);
    }
  }  
  
  drawRiskCircles(riskValue);
  currentDisplayedRisk = riskValue;
}

void drawBlindSpotWarningL(bool active) {
  tft.drawLine(341, 109, 396, 101, active ? LIGHT_ORANGE : GRID_COLOR);
  tft.drawLine(341, 109, 391, 10, active ? LIGHT_ORANGE : GRID_COLOR);
  // TODO: implement active signal
}

void drawBlindSpotWarningR(bool active) {
  tft.drawLine(299, 109, 244, 101, active ? LIGHT_ORANGE : GRID_COLOR);
  tft.drawLine(299, 109, 249, 10, active ? LIGHT_ORANGE : GRID_COLOR);  
  // TODO: implement active signal
}
