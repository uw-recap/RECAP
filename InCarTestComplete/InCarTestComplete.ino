#include <Arduino.h>
#include "wiring_private.h"
#include <Adafruit_GPS.h>
#include <OBD2UART.h>
#include <LoRa.h>

#include <SPI.h>
#include "Adafruit_GFX.h"
#include "Adafruit_HX8357.h"

#define USE_OBD 0
#define USE_GPS 1
#define USE_LORA 1
#define USE_LCD 0
#define USE_DATA_PROC 1
#define USE_USB 1

#if USE_USB
  #define PRINT(value) Serial.print(value);
  #define PRINTLN(value) Serial.println(value);
#else
  #define PRINT(value) ;
  #define PRINTLN(value) ;
#endif

#define GPSSerial Serial1
#define OBDUART Serial2

#define LORA_CS 8
#define LORA_RST 4
#define LORA_IRQ 3
#define LORA_FREQ 915E6
#define LORA_TRANS_INT 100

//LCD
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

//LCD GRID
#define LCD_GRID_START_X 8
#define LCD_GRID_START_Y 8
#define LCD_GRID_HEIGHT  303
#define LCD_GRID_WIDTH   462
#define LCD_GRID_BORDER  2

//Data Processing
#define WORLD_RADIUS 6371000 //Radius of earth in m
#define VARIANCE 10.0
#define MAX_VALID_TIME 2.0
#define MAX_VALID_DIST 10.0
#define MAX_VALID_ANGLE PI/6
#define BRAKING_ACCELERATION 3.4
#define REACTION_TIME 0.15
#define TIMESTEPS 100
#define UNCERTAINTY 0.2
#define RISK_SCALE 100.0
#define DATA_PROC_TRANS_INT 200
#define CAR_ID 13

typedef struct {
  int id;
  int sequence;
  long microseconds; // usec
  long seconds; //sec
  float acceleration; // m/s^2
  float velocity; //m/s
  double xPosition; //m
  double yPosition; //m
  float heading; //radians
} Car_t;

void printCar(Car_t car) {
  PRINT("Car ID: "); PRINTLN(car.id);
  PRINT("Sequence: "); PRINTLN(car.sequence);
  PRINT("Time: "); PRINTLN((float) car.seconds + ((float) car.microseconds) / 1000000 );
  PRINT("Accel: "); PRINTLN(car.acceleration);
  PRINT("Vel: "); PRINTLN(car.velocity);
  PRINT("(X,Y): ("); PRINT(car.xPosition); PRINT(", "); PRINT(car.yPosition); PRINTLN(")");
  PRINT("Heading: "); PRINTLN(car.heading);
}

typedef struct CarNode {
  Car_t car;
  struct CarNode *next;
  struct CarNode *prev;
} CarNode_t;

// GLOBALS
Car_t currentData;
Car_t receivedData;
CarNode_t *carList = NULL;
bool addedDataEnded = true;

void printCarList() {
  int count = 0;
  CarNode_t* currentNode = carList;
  while(currentNode != NULL) {
    printCar(currentNode->car);
    count++;
    PRINT("Count = ");
    PRINTLN(count);
  }
}

////////////////////////////////////////////////////////////////////////////////
// LoRa
////////////////////////////////////////////////////////////////////////////////
bool newPacket = false;
int packetsReceived = 0;
int LoRaLastTransmitTime = 0;
const int LoRaTransmitRate = 100; // ms

void onReceive(int packetSize) {
  if (packetSize != 0) {
    packetsReceived++;
  }
}

void setupLoRa() {
  LoRa.setPins(LORA_CS, LORA_RST, LORA_IRQ);
  if (!LoRa.begin(LORA_FREQ)) {             // initialize ratio at 915 MHz
    PRINTLN("LoRa init failed. Check your connections.");
    while (true);                       // if failed, do nothing
  }
  LoRa.setSPIFrequency(16E6);

  LoRa.onReceive(onReceive);
  LoRa.receive();
  PRINTLN("LoRa init succeeded.");
}

void addNewData(Car_t newData) {
  CarNode_t *currentNode = carList;
  if (currentNode == NULL) {
    carList = new CarNode_t();
    carList->car = newData;
    carList->next = NULL;
    carList->prev = NULL;
  } else if (currentNode->car.id == newData.id) {
    currentNode->car = newData;
  } else {
     while (currentNode->next != NULL) {
      currentNode = currentNode->next;
      if (currentNode->car.id == newData.id) {
        currentNode->car = newData;
        return;
      }
    }

    CarNode_t *newNode = new CarNode_t();
    newNode->car = newData;
    newNode->prev = currentNode;
    newNode->next = NULL;

    currentNode->next = newNode;
  }
}

void LoRaTx() {
  if ((millis() - LoRaLastTransmitTime) > LORA_TRANS_INT) {
    PRINT("Sending... ");
    PRINTLN(currentData.sequence);
    LoRa.beginPacket();
    LoRa.write((uint8_t*)&currentData, sizeof(currentData));
    LoRa.waitCAD();
    LoRaLastTransmitTime = millis();
    currentData.sequence++;
    LoRa.receive(); // Return to receive mode
  }
}

bool LoRaRx() {
  if (packetsReceived > 0) {
    PRINT("Packets Received: ");
    PRINTLN(packetsReceived);
    if(LoRa.read((uint8_t*)&receivedData, sizeof(receivedData)) == sizeof(receivedData)) {
      packetsReceived--;
      newPacket = true;
      addNewData(receivedData);
      PRINT("Received... ");
      PRINTLN(receivedData.sequence);
      return true;
    } else {
      PRINTLN("Incomplete Message!");
    }
  }
  return false;
}

////////////////////////////////////////////////////////////////////////////////
// GPS
////////////////////////////////////////////////////////////////////////////////
Adafruit_GPS GPS(&GPSSerial);

bool isLeapYear(int yr) {
  if (yr % 4 == 0 && yr % 100 != 0 || yr % 400 == 0) return true;
  else return false;
}


byte daysInMonth(int yr,int m) {
  byte days[12]={31,28,31,30,31,30,31,31,30,31,30,31};
  if (m==2 && isLeapYear(yr)) { 
    return 29;
  }
  else {
    return days[m-1];
  }
}

long epochTime(int years, int months, int days, int hours, int minutes, int seconds) {
  long epoch=0;
  if (years < 2000) {
    years += 2000;
  }
  
  for (int yr=1970;yr<years;yr++) {
    if (isLeapYear(yr)) {
      epoch+=366*86400L;
    } else {
      epoch+=365*86400L;
    }
  }
  
  for(int m=1;m<months;m++) {
    epoch+=daysInMonth(years,m)*86400L;
  }
  
  epoch += (days-1)*86400L;
  epoch += hours*3600L;
  epoch += minutes*60;
  epoch += seconds;

  return epoch;
}

void setupGPS() {
  GPS.begin(9600);
  GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCGGA);
  GPS.sendCommand(PMTK_SET_NMEA_UPDATE_5HZ);
  GPS.sendCommand(PMTK_API_SET_FIX_CTL_5HZ);
  delay(1000);
  GPSSerial.println(PMTK_Q_RELEASE);
}

bool updateGPS() {
  char c = GPS.read();
  if (GPS.newNMEAreceived()) {
    GPS.parse(GPS.lastNMEA());
    currentData.seconds = epochTime(GPS.year, GPS.month, GPS.day, GPS.hour, GPS.minute, GPS.seconds);
    currentData.microseconds = GPS.milliseconds*1000;

    double latRad = (floor(GPS.latitude/100) + fmod(GPS.latitude, 100)/60.0) * PI/180;
    double lonRad = -(floor(GPS.longitude/100) + fmod(GPS.longitude, 100)/60.0) * PI/180;

    currentData.xPosition = WORLD_RADIUS*lonRad*cos(latRad);
    currentData.yPosition = WORLD_RADIUS*latRad;
    currentData.heading = abs(fmod((GPS.angle/180.0 + 0.5)*PI,(2*PI)) - PI);
    
    return true;
  }
  return false;
}

////////////////////////////////////////////////////////////////////////////////
// OBD II
////////////////////////////////////////////////////////////////////////////////
// Serial1 uses Pins 0/1 for RX/TX
// Serial2 uses Pins 21/20 for RX/TX
Uart Serial2 (&sercom3, 21, 20, SERCOM_RX_PAD_1, UART_TX_PAD_0);

void SERCOM3_Handler()
{
  Serial2.IrqHandler();
}

COBD obd(&OBDUART);

void setupOBD() {
  while (true) {
    delay(1000);
    obd.begin();
    pinPeripheral(20, PIO_SERCOM);
    pinPeripheral(21, PIO_SERCOM);
    byte version = obd.getVersion();
    PRINT("Freematics OBD-II Adapter ");
    if (version > 0) {
      PRINTLN("detected");
      PRINT("OBD firmware version ");
      PRINT(version / 10);
      PRINT('.');
      PRINTLN(version % 10);
      break;
    } else {
      PRINTLN("not detected");
    }
  }

  do {
    PRINTLN("Connecting...");
  } while (!obd.init());
  PRINTLN("OBD connected!");

  char buf[64];
  if (obd.getVIN(buf, sizeof(buf))) {
    PRINT("VIN:");
    PRINTLN(buf);
  }
}

int obdSpeed;
int lastOBDTime;

void updateOBD() {
  obd.readPID(PID_SPEED, obdSpeed);
  currentData.acceleration = 1000 * ((obdSpeed / 3.6) - currentData.velocity) / (millis() - lastOBDTime);
  lastOBDTime = millis();
  currentData.velocity = obdSpeed / 3.6;
}

////////////////////////////////////////////////////////////////////////////////
// LCD Screen
////////////////////////////////////////////////////////////////////////////////
Adafruit_HX8357 tft = Adafruit_HX8357(TFT_CS, TFT_DC, TFT_RST);
short lcdHeightFifo[LCD_WIDTH_LEFT];
//short sectionHeight = (LCD_MAX_HEIGHT - LCD_MIN_HEIGHT) / LCD_NUM_SECTIONS;

void setupLCD() {
  tft.begin(16E6);
  tft.setRotation(3);
  tft.fillScreen(BG_COLOR);
  drawFullGrid(GRID_COLOR);
//  delay(500);

  // init LCD values array
  for (int i = 0; i < LCD_WIDTH_LEFT; i++) {
    lcdHeightFifo[i] = 0;
  }
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

////////////////////////////////////////////////////////////////////////////////
// Data Processing
////////////////////////////////////////////////////////////////////////////////
int lastDataProcessingTime = 0;
double currentX[TIMESTEPS+1];
double currentY[TIMESTEPS+1];
float currentU[TIMESTEPS+1];

double tempX[TIMESTEPS+1];
double tempY[TIMESTEPS+1];
float tempU[TIMESTEPS+1];

inline double dist(double x0, double y0, double x1, double y1) {
  return sqrt(pow(x1-x0,2)+pow(y1-y0,2));
}

inline double relativeAngle(double x0, double y0, double x1, double y1) {
  return atan2((y1-y0), (x1-x0));
}

void calculateCarTrajectory(Car_t c, double x[], double y[], float u[], float dt, float initialT) {
  float r[TIMESTEPS+1];
  float xHeading = cos(c.heading);
  float yHeading = sin(c.heading);

  r[0] = c.velocity*initialT+1/2*c.acceleration*pow(initialT,2);
  x[0] = c.xPosition + r[0]*xHeading;
  y[0] = c.yPosition + r[0]*yHeading;
  u[0] = r[0]*UNCERTAINTY;

  bool carStopped = false;

  for (int i=1;i<=TIMESTEPS;i++) {
    if (!carStopped) {
      float t = initialT+i*dt;
      r[i] = c.velocity*t+1/2*c.acceleration*pow(t,2);
      if ((r[i]-r[i-1])<0) {
        carStopped = true;
        x[i] = x[i-1];
        y[i] = y[i-1];
        u[i] = u[i-1];
      } else {
        x[i] = c.xPosition+r[i]*xHeading;
        y[i] = c.yPosition+r[i]*yHeading;
        u[i] = r[i]*UNCERTAINTY;
      }
    } else {
      x[i] = x[i-1];
      y[i] = y[i-1];
      u[i] = u[i-1];
    }
  }
}

float calculateRisk(double x0[], double y0[], float u0[], double x1[], double y1[], float u1[], float dt, float tMax) {
  float maxRisk = 0;

  for (int i=1;i<=TIMESTEPS;i++) {
    float separation = dist(x0[i], y0[i], x1[i], y1[i]);
    float uncertainty = u0[i]+u1[i];

    if (separation < uncertainty) {
      float risk = (1-(i*dt/tMax*separation/uncertainty))*RISK_SCALE;
      maxRisk = risk > maxRisk ? risk : maxRisk;
    }
  }

  return maxRisk;
}

void loopDataProcessing() {
  if ((millis() - lastDataProcessingTime) > DATA_PROC_TRANS_INT) {
    lastDataProcessingTime = millis();

    float tMax = currentData.velocity/BRAKING_ACCELERATION + REACTION_TIME;
    float dt = tMax/TIMESTEPS;

    calculateCarTrajectory(currentData, currentX, currentY, currentU, dt, 0);

    float maxRisk = 0.0;
    CarNode_t *currentNode = carList;

    while (currentNode != NULL) {
      Car_t tempCar = currentNode->car;
      float initialT = (currentData.seconds-tempCar.seconds)+(currentData.microseconds-tempCar.microseconds)/1000000.0;
      if (initialT > MAX_VALID_TIME) {
        CarNode_t *tempNode = currentNode;
        currentNode = currentNode->next;

        if (tempNode->prev != NULL) {
          tempNode->prev->next = tempNode->next;
        } else {
          carList = tempNode->next;
        }

        if (tempNode->next != NULL) {
          tempNode->next->prev = tempNode->prev;
        }
        delete(tempNode);

      } else {
        float startR = tempCar.velocity*initialT+1/2*tempCar.acceleration*pow(initialT,2);
        float startX = tempCar.xPosition+startR*cos(tempCar.heading);
        float startY = tempCar.yPosition+startR*sin(tempCar.heading);

        float relAngle = currentData.heading - relativeAngle(currentData.xPosition, currentData.yPosition, startX, startY);
        relAngle = abs(fmod((relAngle + PI),(2*PI)) - PI);

        float diffHeading = currentData.heading - tempCar.heading;
        diffHeading = abs(fmod((diffHeading + PI),(2*PI)) - PI);

        if (relAngle < MAX_VALID_ANGLE && diffHeading < PI/2) {
          calculateCarTrajectory(tempCar, tempX, tempY, tempU, dt, initialT);
          float carRisk = calculateRisk(currentX, currentY, currentU, tempX, tempY, tempU, dt, tMax);
          maxRisk = carRisk > maxRisk ? carRisk : maxRisk;
        }

        currentNode = currentNode->next;
      }
    }
  #if USE_LCD
    drawNewValue(GET_LCD_HEIGHT(random(100)));
   #endif
  }
}

////////////////////////////////////////////////////////////////////////////////
// Main
////////////////////////////////////////////////////////////////////////////////
bool ledON = false;

void setup() {
  Serial.begin(115200);
#if USE_USB
  while (!Serial);
#endif

#if USE_GPS
  PRINTLN("Initializing GPS");
  setupGPS();
#endif

#if USE_OBD
  PRINTLN("Initializing OBDII");
  setupOBD();
#endif

  PRINTLN("Initialization Complete");


#if USE_LCD
  PRINTLN("Initializing LCD");
  setupLCD();
#endif


#if USE_LORA
  PRINTLN("Initializing LoRa");
  setupLoRa();
#endif

currentData.id = CAR_ID;

}

void loop() {
  bool newData = false;
  if(LoRaRx()) {
    newData = true;
  }
  
  if(updateGPS()) {
    PRINT("x: ");
    PRINT(currentData.xPosition);
    PRINT(" y: ");
    PRINT(currentData.yPosition);
    PRINT(" h: ");
    PRINTLN(currentData.heading);
    
    digitalWrite(13, ledON);
    ledON = !ledON;
    #if USE_OBD
    updateOBD();
    PRINT("Speed: ");
    PRINT(currentData.velocity);
    PRINT(" Accel: ");
    PRINTLN(currentData.acceleration);
    #endif
    
    LoRaTx();
    newData = true;
  }

  #if USE_DATA_PROC
  if(newData){
    loopDataProcessing();
  }
  #endif
}
