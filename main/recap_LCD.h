#ifndef _RECAP_LCD_H_
#define _RECAP_LCD_H_

// Colors
#define BLACK 0x0000
#define WHITE 0xFFFF
#define YELLOW 0xFFE0
#define ORANGE 0xFCC0
#define RED 0xF800
#define GREY 0x7BEF

#define BG_COLOR BLACK
#define GRID_COLOR WHITE
#define CAR_COLOR WHITE
#define BOUND_COLOR GREY

// risk percentage that we get from the data processing algorithm
#define MIN_RISK 0
#define MAX_RISK 100

// risk values in px as shown on the LCD screen
#define LCD_MIN_RISK 0
#define LCD_MAX_RISK 150
#define LCD_RISK_HEIGHT (LCD_MAX_RISK - LCD_MIN_RISK)
#define LCD_RISK_OFFSETY 150
#define MED_RISK  (LCD_MAX_RISK / 3) // 33% of max risk
#define HIGH_RISK (LCD_MAX_RISK / 2) // 50% of max risk

// map between percentage risk and LCD risk
#define GET_LCD_RISK(value) map(value, MIN_RISK, MAX_RISK, LCD_MIN_RISK, LCD_MAX_RISK)

// SPI pinouts
#define TFT_CS 9
#define TFT_DC 10
#define TFT_RST -1 // RST can be set to -1 if you tie it to Arduino's reset

// helper macro (originally from GFX library)
#ifndef _swap_int16_t
#define _swap_int16_t(a, b) { int16_t t = a; a = b; b = t; }
#endif

#include <SPI.h>
#include "Adafruit_GFX.h"
#include "Adafruit_HX8357.h"
#include "recap_common.h"

// PUBLIC API
// draw the car and other static images
void setupLCD();

// draw a new risk value to the forward warning section of the LCD screen
void drawRiskValue(int16_t riskValue);

// draw the blind spot warning sections. active = if there is a car in the blind spot
void drawBlindSpotWarningL(bool active);
void drawBlindSpotWarningR(bool active);
#endif
