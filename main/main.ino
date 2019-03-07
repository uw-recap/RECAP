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

    newData = true;
  }
  #endif

  #if USE_DATA_PROC
  if(newData){
    // PRINTLN(dist(currentData, otherData));
    // drawRiskValue(processData(currentData));
    newData = false;
  }
  #endif

  for (int j = 0; j < 3; j++) {
    // demo: sweep risk up, then sweep risk down
    for (int i = MIN_RISK; i < MAX_RISK; i++) {
      drawRiskValue(GET_LCD_RISK(i));
      delay(50);
    }

    delay(5000);

    for (int i = MAX_RISK; i > MIN_RISK; i--) {
      drawRiskValue(GET_LCD_RISK(i));
      delay(50);
    }

    delay(5000);
  }

  drawBlindSpotWarningL(true);
  delay(1000);
  drawBlindSpotWarningL(false);
  delay(1000);
  drawBlindSpotWarningL(true);
  delay(1000);
  drawBlindSpotWarningL(false);
  delay(1000);

  drawBlindSpotWarningR(true);
  delay(1000);
  drawBlindSpotWarningR(false);
  delay(1000);
  drawBlindSpotWarningR(true);
  delay(1000);
  drawBlindSpotWarningR(false);
  delay(1000);

}
