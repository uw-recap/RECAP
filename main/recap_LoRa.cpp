#include "recap_LoRa.h"

int packetsReceived;

static LoRaMode_t mode;

void onReceive(int packetSize) {
  if (packetSize != 0) {
    packetsReceived++;
  }
}

int setupLoRa(LoRaMode_t m) {
  mode = m;

  LoRa.setPins(LORA_CS, LORA_RST, LORA_IRQ);
  if (!LoRa.begin(LORA_FREQ)) {             // initialize ratio at 915 MHz
    PRINTLN("LoRa init failed. Check your connections.");
    return -1;
  }

  packetsReceived = 0;

  LoRa.setSPIFrequency(SPI_FREQ);

  if(mode == LoRa_Duplex || mode == LoRa_RX) {
    LoRa.onReceive(onReceive);
    LoRa.receive();
  } else {
    LoRa.idle();
  }

  LoRa.enableCrc();

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

  if(mode != LoRa_TX) {
    LoRa.waitCAD();
    // Go Back to Receive Mode after Transmitting
    LoRa.receive();
  } else {
    LoRa.endPacket();
  }

  car->sequence++;

  return 0;
}

int receiveLoRa(Car_t* car)
{
  if(car == NULL) {
    return -1;
  }

  if(packetsReceived > 0) {
    packetsReceived--;
    return LoRa.read((uint8_t*)car, sizeof(Car_t));
  }
  return -1;
}
