#include "recap.h"

////////////////////////////////////////////////////////////////////////////////
// Main
////////////////////////////////////////////////////////////////////////////////
Car_t currentData;
Car_t otherData;
bool newDataGPS = false;
bool newDataLoRa = false;
int lastTransmitTime = 0;
const int maxTransmitRate = 100;

#define AVG_FILTER_SIZE 2
float previousDistances[AVG_FILTER_SIZE];

void setup() {
  for (int i = 0; i < AVG_FILTER_SIZE; i++) {
    previousDistances[i] = 0;
  }

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

  pinMode(STATUS_LED, OUTPUT);
  digitalWrite(STATUS_LED, HIGH);

  PRINTLN("Initialization Complete");
  currentData.id = CAR_ID;
}

void loop() {
  #if USE_LORA
  #ifndef TRANSMITTER
  if(receiveLoRa(&otherData)>=0) {
    newDataLoRa = true;

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

    newDataGPS = true;
  }
  #endif

  #if USE_DATA_PROC
  if(newDataLoRa && newDataGPS){
    newDataLoRa = newDataGPS = false;

    // Use the riskHeadway function directly to avoid weirdness with assessRisk
    // We can switch to the commented out code to test the assessRisk function.
    float distance = dist(currentData, otherData);
    float relAngle = bearing(currentData, otherData);
    if (distance > 750) {
//      PRINTLN("OUT OF RANGE");
      return;
    }

    if(abs(fmod(relAngle - currentData.heading + 360, 360) - 180) < 90) {
//      PRINTLN("LEAD CAR");
      drawRiskValue(0);
      return;
    }

    // moving average filter: shift one down and insert latest value
    float averageDistance = 0;
    for (int i = AVG_FILTER_SIZE - 1; i > 0; i--) {
      previousDistances[i] = previousDistances[i-1];
    }
    previousDistances[0] = distance;

    // take average
    for (int i = 0; i < AVG_FILTER_SIZE; i++) {
      averageDistance += previousDistances[i];
    }
    averageDistance /= AVG_FILTER_SIZE;

    
    
    drawRiskValue(reqStopAccelRisk(currentData, otherData, averageDistance));
  }
  #endif
}
