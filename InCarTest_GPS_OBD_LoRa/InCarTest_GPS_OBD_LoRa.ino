#include <Arduino.h>
#include "wiring_private.h"
#include <Scheduler.h>
#include <Adafruit_GPS.h>
#include <OBD2UART.h>
#include <LoRa.h>
#include <TimeLib.h>

#define USE_OBD 1
#define USE_GPS 1
#define USE_LORA 1

#define GPSSerial Serial1
#define OBDUART Serial2

#define LORA_CS 8
#define LORA_RST 4
#define LORA_IRQ 3
#define LORA_FREQ 915E6

typedef struct {
  int id;
  int sequence;
  int microseconds; // us
  int seconds; //sec
  float acceleration; // m/s^2
  float velocity; //m/s
  float xPosition; //unitless (need to multiply by radius of earth)
  float yPosition; //unitless (need to multiply by radius of earth)
  float heading; //radians
} Car_t;

Car_t currentData;
Car_t receivedData;

////////////////////////////////////////////////////////////////////////////////
// LoRa
////////////////////////////////////////////////////////////////////////////////
bool newPacket = false;
int packetsReceived = 0;
int LoRaLastTransmitTime = 0;
const int LoRaTransmitRate = 100; // ms

void onReceive(int packetSize) {
  if (packetSize != 0) {
    packetsReceived++;
  }
}

void setupLoRa() {
  LoRa.setPins(LORA_CS, LORA_RST, LORA_IRQ);
  if (!LoRa.begin(LORA_FREQ)) {             // initialize ratio at 915 MHz
    Serial.println("LoRa init failed. Check your connections.");
    while (true);                       // if failed, do nothing
  }

  LoRa.onReceive(onReceive);
  LoRa.receive();
  Serial.println("LoRa init succeeded.");
}

void loopLoRa() {
  // Receive
  if (packetsReceived > 0) {
    LoRa.read((uint8_t*)&receivedData, sizeof(receivedData));
    packetsReceived--;
    newPacket = true;
  }

  // Transmit
  if ((millis() - LoRaLastTransmitTime) > LoRaTransmitRate) {
    LoRa.beginPacket();
    LoRa.write((uint8_t*)&currentData, sizeof(currentData));
    LoRa.waitCAD();
    LoRaLastTransmitTime = millis();
    currentData.sequence++;
    LoRa.receive(); // Return to receive mode
  }

  yield();
}

////////////////////////////////////////////////////////////////////////////////
// GPS
////////////////////////////////////////////////////////////////////////////////
Adafruit_GPS GPS(&GPSSerial);

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
    setTime(GPS.hour, GPS.minute, GPS.seconds, GPS.day, GPS.month, GPS.year);
    currentData.seconds = now();
    currentData.microseconds = micros();
    currentData.xPosition = GPS.longitude;
    currentData.yPosition = GPS.latitude;
    currentData.heading = GPS.angle;
  }
  yield();
}

////////////////////////////////////////////////////////////////////////////////
// OBD II
////////////////////////////////////////////////////////////////////////////////
// Serial1 uses Pins 0/1 for RX/TX
// Serial2 uses Pins 21/20 for RX/TX
Uart Serial2 (&sercom3, 21, 20, SERCOM_RX_PAD_1, UART_TX_PAD_0);

void SERCOM3_Handler()
{
  Serial2.IrqHandler();
}

COBD obd(&OBDUART);

void setupOBD() {
  while (true) {
    delay(1000);
    obd.begin();
    pinPeripheral(20, PIO_SERCOM);
    pinPeripheral(21, PIO_SERCOM);
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
int lastOBDTime;

void loopOBD() {
  obd.readPID(PID_SPEED, obdSpeed);
  currentData.acceleration = (currentData.velocity - (obdSpeed / 3.6)) / ((millis() - lastOBDTime) / 1000);
  lastOBDTime = millis();
  currentData.velocity = obdSpeed / 3.6;
  delay(100);
}

////////////////////////////////////////////////////////////////////////////////
// Main
////////////////////////////////////////////////////////////////////////////////
void setup() {
  Serial.begin(115200);
  while (!Serial);

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

#if USE_LORA
  Serial.println("Initializing LoRa");
  setupLoRa();
  Scheduler.startLoop(loopLoRa);
#endif
}

void loop() {
  if (newPacket) {
    newPacket = false;
    Serial.println("Received Packet:");
    Serial.print("  ID:    "); Serial.println(receivedData.id);
    Serial.print("  Seq:   "); Serial.println(receivedData.sequence);
    Serial.print("  Sec:   "); Serial.println(receivedData.seconds);
    Serial.print("  uSec:  "); Serial.println(receivedData.microseconds);
    Serial.print("  Accel: "); Serial.println(receivedData.acceleration);
    Serial.print("  Speed: "); Serial.println(receivedData.velocity);
    Serial.print("  Lat:   "); Serial.println(receivedData.yPosition);
    Serial.print("  Long:  "); Serial.println(receivedData.xPosition);
    Serial.print("  Hdg:   "); Serial.println(receivedData.heading);
  }
  yield();
}
