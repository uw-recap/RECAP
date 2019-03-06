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
  #if USE_LORA
  if(receiveLoRa(&otherData)>=0) {
    newData = true;
    
    #if USE_DATA_PROC
    addNewData(otherData);
    #endif

    //PRINT("OTHER DATA: ");
    //printCar(otherData);
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
    //PRINT("Speed: ");
    //PRINT(currentData.velocity);
    //PRINT(" Accel: ");
    //PRINTLN(currentData.acceleration);
    #endif
    
    #if USE_LORA
    if((millis() - lastTransmitTime) > maxTransmitRate) {
      transmitLoRa(&currentData);
      lastTransmitTime = millis();
    }
    #endif

    //PRINT("   MY DATA: ")
    //printCar(currentData);

    newData = true;
  }
  #endif

  #if USE_DATA_PROC
  if(newData){
    PRINT("X1: ");
    PRINT(currentData.xPosition);
    PRINT(" Y1: ");
    PRINT(currentData.yPosition);
    PRINT(" X2: ");
    PRINT(otherData.xPosition);
    PRINT(" Y2: ");
    PRINT(otherData.yPosition);
    PRINT(" Dist: ");
    PRINTLN(dist(currentData,otherData));
    //drawRiskValue(processData(currentData));
    newData = false;
  }
  #endif
}
