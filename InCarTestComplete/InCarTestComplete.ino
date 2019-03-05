#include <Arduino.h>
#include "wiring_private.h"
#include <Adafruit_GPS.h>
#include <OBD2UART.h>
#include <LoRa.h>
#include "LCD_helpers.h"

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
    drawRiskValue(GET_LCD_HEIGHT(maxRisk));
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

  tft;

  #if USE_DATA_PROC
  if(newData){
    loopDataProcessing();
  }
  #endif
}
