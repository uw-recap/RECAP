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
  const float a = 0.7;
  int obdSpeed;
  if(obd.readPID(PID_SPEED, obdSpeed)) {
    // Infinite Impulse Response filter with characteristic parameter 'a'
    car->acceleration = a * (1000 * ((obdSpeed / 3.6) - car->velocity) / (millis() - lastOBDTime)) + (1-a)*car->acceleration;
    // Truncate the IIR result if it gets really small to avoid loss-of-precision issues
    car->acceleration = car->acceleration < 0.001 ? 0 : car->acceleration;

    lastOBDTime = millis();
    car->velocity = obdSpeed / 3.6;
    return 0;
  }
}
