#include "recap_GPS.h"

Adafruit_GPS GPS(&GPSSerial);

int setupGPS() {
  GPS.begin(9600);
  GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCGGA);
  GPS.sendCommand(PMTK_SET_NMEA_UPDATE_5HZ);
  GPS.sendCommand(PMTK_API_SET_FIX_CTL_5HZ);
  delay(1000);
  GPSSerial.println(PMTK_Q_RELEASE);
  return 0;
}

int readGPS(Car_t* car) {
  char c = GPS.read();
  if (GPS.newNMEAreceived()) {
    GPS.parse(GPS.lastNMEA());
    car->seconds = epochTime(GPS.year, GPS.month, GPS.day, GPS.hour, GPS.minute, GPS.seconds);
    car->microseconds = GPS.milliseconds*1000;

    double latRad = (floor(GPS.latitude/100) + fmod(GPS.latitude, 100)/60.0) * PI/180;
    double lonRad = -(floor(GPS.longitude/100) + fmod(GPS.longitude, 100)/60.0) * PI/180;

    car->xPosition = WORLD_RADIUS*lonRad*cos(latRad);
    car->yPosition = WORLD_RADIUS*latRad;
    car->heading = abs(fmod((GPS.angle/180.0 + 0.5)*PI,(2*PI)) - PI);
    
    return 0;
  } else {
    return -1;
  }
}
