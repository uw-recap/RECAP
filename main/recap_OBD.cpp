#include "recap_OBD.h"

#define ACCEL_FIR_SIZE 3
#define ACCEL_IIR_CONST 0.5
float previousAcceleration[ACCEL_FIR_SIZE];

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

  for(int i = 0; i < ACCEL_FIR_SIZE; i++) {
   previousAcceleration[i] = 0;
  }

  return 0;
}

int readOBD(Car_t* car) {
  int obdSpeed;
  if(obd.readPID(PID_SPEED, obdSpeed)) {
    float currentAccel = (1000 * ((obdSpeed / 3.6) - car->velocity) / (millis() - lastOBDTime));
    lastOBDTime = millis();

    // Finite impulse response filter
    for (int i = ACCEL_FIR_SIZE - 1; i > 0; i--) {
      previousAcceleration[i] = previousAcceleration[i-1];
    }
    previousAcceleration[0] = currentAccel;

    float accelAvg = 0;
    for (int i = 0; i < ACCEL_FIR_SIZE; i++) {
      accelAvg += previousAcceleration[i];
    }
    accelAvg /= ACCEL_FIR_SIZE;

    // Infinite Impulse Response filter
    car->acceleration = ACCEL_IIR_CONST * accelAvg + (1-ACCEL_IIR_CONST) * car->acceleration;

    // Truncate the IIR result if it gets really small to avoid loss-of-precision issues
    car->acceleration = abs(car->acceleration) < 0.001 ? 0 : car->acceleration;

    car->velocity = obdSpeed / 3.6;

    return 0;
  }
}
