#include "recap_LoRa.h"

void onReceive(int packetSize) {
  if (packetSize != 0) {
    packetsReceived++;
  }
}

int setupLoRa() {
  LoRa.setPins(LORA_CS, LORA_RST, LORA_IRQ);
  if (!LoRa.begin(LORA_FREQ)) {             // initialize ratio at 915 MHz
    PRINTLN("LoRa init failed. Check your connections.");
    return -1;
  }

  LoRa.setSPIFrequency(SPI_FREQ);

  LoRa.onReceive(onReceive);
  LoRa.receive();
  PRINTLN("LoRa init succeeded.");
  return 0;
}

int transmitLoRa(Car_t* car)
{
  if(car == NULL) {
    return -1;
  }

  LoRa.beginPacket();
  LoRa.write((uint8_t*)car, sizeof(Car_t));
  LoRa.waitCAD();
 
  car->sqeuence++;

  return 0;
}

int receiveLoRa(Car_t* car)
{
  if(car == NULL) {
    return -1;
  }

  if(LoRa.read((uint8_t*)car, sizeof(Car_t))==sizeof(Car_t)){
    return sizeof(Car_t);
  } else {
    return -1;
  }
}
