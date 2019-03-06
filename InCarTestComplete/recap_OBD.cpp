#include "recap_OBD.h"

#define OBDUART Serial2

Uart Serial2 (&sercom3, OBD_RX_PIN, OBD_TX_PIN, OBD_RX_PAD, OBD_TX_PAD);

void SERCOM3_Handler()
{
  Serial2.IrqHandler();
}

COBD obd(&OBDUART);

int lastOBDTime;

int setupOBD() {
  while (true) {
    delay(1000);
    obd.begin();
    pinPeripheral(20, PIO_SERCOM);
    pinPeripheral(21, PIO_SERCOM);
    byte version = obd.getVersion();
    PRINT("Freematics OBD-II Adapter ");
    if (version > 0) {
      PRINTLN("detected");
      PRINT("OBD firmware version ");
      PRINT(version / 10);
      PRINT('.');
      PRINTLN(version % 10);
      break;
    } else {
      PRINTLN("not detected");
    }
  }

  do {
    PRINTLN("Connecting...");
  } while (!obd.init());
  PRINTLN("OBD connected!");

  char buf[64];
  if (obd.getVIN(buf, sizeof(buf))) {
    PRINT("VIN:");
    PRINTLN(buf);
  }

  return 0;
}

int readOBD(Car_t* car) {
  int obdSpeed;
  if(obd.readPID(PID_SPEED, obdSpeed)) {
    car->acceleration = 1000 * ((obdSpeed / 3.6) - car->velocity) / (millis() - lastOBDTime);
    lastOBDTime = millis();
    car->velocity = obdSpeed / 3.6;
    return 0;
  }
}
