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

#define MIN_RISK 0
#define MAX_RISK 100

// LCD Specs
#define LCD_MIN_HEIGHT 0
#define LCD_MAX_HEIGHT 300    // the height of the graph
#define LCD_WIDTH_LEFT 300    // the width of the left side of the screen (where the past data is shown)
#define LCD_WIDTH_RIGHT 154   // the width of the right side of the screen (where the current data is shown)
#define LCD_OFFSETX_LEFT 169  // where the left side of the graph starts
#define LCD_OFFSETX_RIGHT 10  // where the right side of the graph starts
#define LCD_OFFSETY 10        // where the grid ends and the actual graph starts
#define LCD_NUM_SECTIONS 3    // how many sections the LCD screen should be split into
#define GET_LCD_HEIGHT(value) map(value, MIN_RISK, MAX_RISK, LCD_MIN_HEIGHT, LCD_MAX_HEIGHT)


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

#include <SPI.h>
#include "Adafruit_GFX.h"
#include "Adafruit_HX8357.h"

// GLOBALS
extern Adafruit_HX8357 tft;
extern uint8_t lcdHeightFifo[LCD_WIDTH_LEFT];
extern uint8_t sectionHeight;

// draw a new risk value to the forward warning section of the LCD screen
void drawNewValue(uint8_t newValue);
void drawStaticImages();
void drawCar();
void drawBlindSpotWarningL(bool active);
void drawBlindSpotWarningR(bool active);
void drawRiskCircles(uint8_t maxRisk);
uint16_t pickColor(uint8_t leftDistance);
