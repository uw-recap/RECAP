#include "recap_common.h"

void printCar(Car_t car) {
  PRINT("Id: "); PRINTLN(car.id);
  PRINT("Seq: "); PRINTLN(car.sequence);
  PRINT("Time: "); PRINTLN(car.seconds + car.microseconds / 1E6);
  PRINT("Accel: "); PRINTLN(car.acceleration);
  PRINT("Speed: "); PRINTLN(car.velocity);
  PRINT("(x,y): ("); PRINT(car.xPosition); PRINT(", "); PRINT(car.yPosition); PRINTLN(")");
  PRINT("Hdg: "); PRINTLN(car.heading);
}
