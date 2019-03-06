#ifndef _RECAP_GPS_H_
#define _RECAP_GPS_H_

#include "recap_common.h"
#include <Adafruit_GPS.h>

#define GPSSerial Serial1

int setupGPS();

int readGPS(Car_t* car);

#endif;
