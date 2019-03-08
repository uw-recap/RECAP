#include "recap.h"

////////////////////////////////////////////////////////////////////////////////
// Main
////////////////////////////////////////////////////////////////////////////////
Car_t currentData;
Car_t otherData;
bool newData = false;
int lastTransmitTime = 0;
const int maxTransmitRate = 100;
float previousDistance = 0;

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
  #ifdef TRANSMITTER
    PRINTLN("Initializing LoRa TRANSMIT ONLY");
    setupLoRa(LoRa_TX);
  #else
    PRINTLN("Initializing LoRa RECEIVE ONLY");
    setupLoRa(LoRa_RX);
  #endif
#endif

  PRINTLN("Initialization Complete");
  currentData.id = CAR_ID;
}

void loop() {
  #if USE_LORA
  #ifndef TRANSMITTER 
  if(receiveLoRa(&otherData)>=0) {
    newData = true;

    #if USE_DATA_PROC
    //addNewData(otherData);
    #endif
  }
  #endif
  #endif

  #if USE_GPS
  if(readGPS(&currentData)==0) {

    #if USE_OBD
    readOBD(&currentData);
    #endif

    #if USE_LORA
    #ifdef TRANSMITTER
    if((millis() - lastTransmitTime) > maxTransmitRate) {
      transmitLoRa(&currentData);
      lastTransmitTime = millis();
    }
    #endif
    #endif

    newData = true;
  }
  #endif

  #if USE_DATA_PROC
  if(newData){
    newData = false;
    const float a = 0.2;

    // Use the riskHeadway function directly to avoid weirdness with assessRisk
    // We can switch to the commented out code to test the assessRisk function.
    float distance = dist(currentData, otherData);
    if (distance > 750) return;

    // infinite impulse respoinse filter
    distance = a * distance + (1-a) * previousDistance;

    drawRiskValue(riskHeadway(currentData, otherData, distance));

    previousDistance = distance;
    //drawRiskValue(assessRisk(currentData, otherData));
  }
  #endif
}
