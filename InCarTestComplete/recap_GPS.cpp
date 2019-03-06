#include "recap_GPS.h"

// Epoch Time Helpers
bool isLeapYear(int yr) {
  if (yr % 4 == 0 && yr % 100 != 0 || yr % 400 == 0) return true;
  else return false;
}

byte daysInMonth(int yr,int m) {
  byte days[12]={31,28,31,30,31,30,31,31,30,31,30,31};
  if (m==2 && isLeapYear(yr)) {
    return 29;
  }
  else {
    return days[m-1];
  }
}

long epochTime(int years, int months, int days, int hours, int minutes, int seconds) {
  long epoch=0;
  if (years < 2000) {
    years += 2000;
  }

  for (int yr=1970;yr<years;yr++) {
    if (isLeapYear(yr)) {
      epoch+=366*86400L;
    } else {
      epoch+=365*86400L;
    }
  }

  for(int m=1;m<months;m++) {
    epoch+=daysInMonth(years,m)*86400L;
  }

  epoch += (days-1)*86400L;
  epoch += hours*3600L;
  epoch += minutes*60;
  epoch += seconds;

  return epoch;
}

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
