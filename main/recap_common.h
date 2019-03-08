#ifndef _RECAP_COMMON_H_
#define _RECAP_COMMON_H_

#include <Arduino.h>

#define USE_OBD 0
#define USE_GPS 1
#define USE_LORA 1
#define USE_LCD 1
#define USE_DATA_PROC 1
#define USE_USB 1

// If this macro is defined, then the device is 
// TRANSMIT ONLY otherwise it is RECEIVE ONLY
#define TRANSMITTER

#if USE_USB
  #define PRINT(value) Serial.print(value);
  #define PRINTLN(value) Serial.println(value);
#else
  #define PRINT(value) ;
  #define PRINTLN(value) ;
#endif

#define SPI_FREQ 16E6

#define WORLD_RADIUS 6372795.0 //Radius of earth in m

typedef struct {
  int id;
  int sequence;
  long microseconds; // usec
  long seconds; //sec
  float acceleration; // m/s^2
  float velocity; //m/s
  double xPosition; //m
  double yPosition; //m
  float heading; //radians
} Car_t;

void printCar(Car_t car);
void printCarLn(Car_t car);

#endif
