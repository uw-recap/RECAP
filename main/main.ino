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
    //addNewData(otherData);
    #endif

//     PRINT("OTHER DATA: ");
//     printCar(otherData);
  }
  #endif
  
  #if USE_GPS
  if(readGPS(&currentData)==0) {

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
    PRINTLN(assessRisk(currentData, otherData));
    //drawRiskValue(processData(currentData));
    newData = false;
  }
  #endif
}
