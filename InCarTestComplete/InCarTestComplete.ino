#include "recap.h"

////////////////////////////////////////////////////////////////////////////////
// Main
////////////////////////////////////////////////////////////////////////////////
Car_t currentData;
Car_t tempData;
bool newData = false;

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
  if(receiveLoRa(&tempData)) {
    newData = true;
    addNewData(tempData);
  }
  
  if(readGPS(&currentData)) {
    PRINT("x: ");
    PRINT(currentData.xPosition);
    PRINT(" y: ");
    PRINT(currentData.yPosition);
    PRINT(" h: ");
    PRINTLN(currentData.heading);
    
    #if USE_OBD
    readOBD(&currentData);
    PRINT("Speed: ");
    PRINT(currentData.velocity);
    PRINT(" Accel: ");
    PRINTLN(currentData.acceleration);
    #endif
    
    transmitLoRa(&currentData);
    newData = true;
  }
  
  #if USE_DATA_PROC
  if(newData){
    drawRiskValue(processData(currentData));
    newData = false;
  }
  #endif
}
