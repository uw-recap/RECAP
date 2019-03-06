#include "recap.h"

////////////////////////////////////////////////////////////////////////////////
// Main
////////////////////////////////////////////////////////////////////////////////
Car_t currentData;
Car_t otherData;
bool newData = false;
int lastTransmitTime = 0;
const int maxTransmitRate = 100;

void setup() {
#if USE_USB
  Serial.begin(115200);
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

#if USE_LCD
  PRINTLN("Initializing LCD");
  setupLCD();
#endif

#if USE_LORA
  PRINTLN("Initializing LoRa");
  setupLoRa();
#endif

  PRINTLN("Initialization Complete");
  currentData.id = CAR_ID;
}

void loop() {
  // 403 Tamarack
  double lat1 = 43 + 28.9950/60.0;
  double long1 = -(80 + 31.9480/60.0);

  // 110 Columbia
  double lat2 = 43 + 28.7220/60.0;
  double long2 = -(80 + 32.1590/60.0);
  // expected distance: 560-575 m


  // bridgeport x hwy 85
  lat1 = 43.474005;
  long1 = -80.493757;

  // university x hwy 85
  lat2 = 43.486170;
  long2 = -80.501401;
  // expected distance: 1.47 km

  double distance;

  long timeStart = micros();

  for (int i = 0; i < 1000; i++) {
    double lat1Rad = radians(lat1);
    double lat2Rad = radians(lat2);
    double dLon = radians(long2 - long1);
    // forumla from here: https://www.movable-type.co.uk/scripts/latlong.html
    distance = acos(sin(lat1Rad)*sin(lat2Rad) + cos(lat1Rad)*cos(lat2Rad)*cos(dLon))*WORLD_RADIUS;
  }

  long timeStop = micros();

  PRINT("Dist: "); PRINTLN(distance);
  PRINT("Total Calc time: "); PRINTLN(timeStop - timeStart);
  PRINT("Avg calc time: "); PRINTLN((timeStop-timeStart)/1000.0);
  while(true);

  #if USE_LORA
  if(receiveLoRa(&otherData)>=0) {
    newData = true;

    #if USE_DATA_PROC
    //addNewData(otherData);
    #endif

//     PRINT("OTHER DATA: ");
//     printCar(otherData);
  }
  #endif

  #if USE_GPS
  if(readGPS(&currentData)==0) {
    //PRINT("x: ");
    //PRINT(currentData.xPosition);
    //PRINT(" y: ");
    //PRINT(currentData.yPosition);
    //PRINT(" h: ");
    //PRINTLN(currentData.heading);

    #if USE_OBD
    readOBD(&currentData);
    #endif

    #if USE_LORA
    if((millis() - lastTransmitTime) > maxTransmitRate) {
      transmitLoRa(&currentData);
      lastTransmitTime = millis();
    }
    #endif

//     PRINT("   MY DATA: ");
//     printCar(currentData);

    newData = true;
  }
  #endif

  #if USE_DATA_PROC
  if(newData){
    PRINTLN(dist(currentData, otherData));
    //drawRiskValue(processData(currentData));
    newData = false;
  }
  #endif
}
