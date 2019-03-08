#include "recap_LoRa.h"

int packetsReceived;

static LoRaMode_t mode;

//CRC-8 - based on the CRC8 formulas by Dallas/Maxim
//code released under the therms of the GNU GPL 3.0 license
uint8_t CRC8(const uint8_t *data, uint8_t len) {
  uint8_t crc = 0x00;
  while (len--) {
    uint8_t extract = *data++;
    for (uint8_t tempI = 8; tempI; tempI--) {
      uint8_t sum = (crc ^ extract) & 0x01;
      crc >>= 1;
      if (sum) {
        crc ^= 0x8C;
      }
      extract >>= 1;
    }
  }
  return crc;
}

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

  PRINTLN("LoRa init succeeded.");
  return 0;
}

int transmitLoRa(Car_t* car)
{
  if(car == NULL) {
    return -1;
  }

  uint8_t crc = CRC8((uint8_t*)car, sizeof(Car_t));

  LoRa.beginPacket();
  LoRa.write((uint8_t*)car, sizeof(Car_t));
  LoRa.write(crc);

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

  Car_t tmp;

  if(packetsReceived > 0) {
    packetsReceived--;
    if(LoRa.read((uint8_t*)&tmp, sizeof(Car_t)) > 0){
      uint8_t crc = (uint8_t)LoRa.read();
      if(CRC8((uint8_t*)&tmp, sizeof(Car_t))==crc) {
        *car = tmp;
        return sizeof(Car_t);
      }
    }
  }
  return -1;
}
