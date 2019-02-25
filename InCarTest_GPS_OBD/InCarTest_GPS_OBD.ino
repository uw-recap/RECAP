#include <Arduino.h>
#include "wiring_private.h"
#include <Scheduler.h>
#include <Adafruit_GPS.h>
#include <OBD2UART.h>

#define USE_OBD 1
#define USE_GPS 1

#define GPSSerial Serial1
#define OBDUART Serial2

// Serial1 uses Pins 0/1 for RX/TX
// Serial2 uses Pins 11/10 for RX/TX
Uart Serial2 (&sercom1, 11, 10, SERCOM_RX_PAD_0, UART_TX_PAD_2);

void SERCOM1_Handler()
{
  Serial2.IrqHandler();
}

Adafruit_GPS GPS(&GPSSerial);
COBD obd(&OBDUART);

void setupGPS() {
  GPS.begin(9600);
  GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCGGA);
  GPS.sendCommand(PMTK_SET_NMEA_UPDATE_1HZ);
  delay(1000);
  GPSSerial.println(PMTK_Q_RELEASE);
}

void loopGPS() {
  char c = GPS.read();
  if (GPS.newNMEAreceived()) {
    GPS.parse(GPS.lastNMEA());
  }
  yield();
}

void setupOBD() {
  while(true){
    delay(1000);
    obd.begin();
    pinPeripheral(10, PIO_SERCOM);
    pinPeripheral(11, PIO_SERCOM);
    byte version = obd.getVersion();
    Serial.print("Freematics OBD-II Adapter ");
    if (version > 0) {
      Serial.println("detected");
      Serial.print("OBD firmware version ");
      Serial.print(version / 10);
      Serial.print('.');
      Serial.println(version % 10);
      break;
    } else {
      Serial.println("not detected");
    }
  }
  
  do {
    Serial.println("Connecting...");
  } while (!obd.init());
  Serial.println("OBD connected!");
  
  char buf[64];
  if (obd.getVIN(buf, sizeof(buf))) {
    Serial.print("VIN:");
    Serial.println(buf);
  }
}

int obdSpeed;

void loopOBD() {
  obd.readPID(PID_SPEED, obdSpeed); 
  delay(100);
}

void setup() {
  Serial.begin(115200);
  while(!Serial);
#if USE_GPS
  Serial.println("Initializing GPS");
  setupGPS();
  Scheduler.startLoop(loopGPS);
#endif
#if USE_OBD
  Serial.println("Initializing OBDII");
  setupOBD();
  Scheduler.startLoop(loopOBD);
#endif
  Serial.println("Initialization Complete");
}

void loop() {
#if USE_GPS
  Serial.print("\nTime: ");
  Serial.print(GPS.hour, DEC); Serial.print(':');
  Serial.print(GPS.minute, DEC); Serial.print(':');
  Serial.print(GPS.seconds, DEC); Serial.print('.');
  Serial.println(GPS.milliseconds);
  Serial.print("Date: ");
  Serial.print(GPS.day, DEC); Serial.print('/');
  Serial.print(GPS.month, DEC); Serial.print("/20");
  Serial.println(GPS.year, DEC);
  Serial.print("Fix: "); Serial.print((int)GPS.fix);
  Serial.print(" quality: "); Serial.println((int)GPS.fixquality);
  if (GPS.fix) {
    Serial.print("Location: ");
    Serial.print(GPS.latitude, 4); Serial.print(GPS.lat);
    Serial.print(", ");
    Serial.print(GPS.longitude, 4); Serial.println(GPS.lon);
    Serial.print("Speed (km/h): "); Serial.println((int)(GPS.speed*1.852)); // Knots to Kilometers per Hour
    Serial.print("Angle: "); Serial.println(GPS.angle);
    Serial.print("Altitude: "); Serial.println(GPS.altitude);
    Serial.print("Satellites: "); Serial.println((int)GPS.satellites);
  }
#endif
#if USE_OBD
  Serial.print("OBD Speed (km/h): "); Serial.println(obdSpeed);
#endif
  delay(1000);
}
