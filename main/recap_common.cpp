#include "recap_common.h"

void printCar(Car_t car) {
  PRINT("Id: "); PRINT(car.id);
  PRINT(" Seq: "); PRINT(car.sequence);
  PRINT(" Time: "); PRINT(car.seconds + car.microseconds / 1E6);
  PRINT(" Accel: "); PRINT(car.acceleration);
  PRINT(" Speed: "); PRINT(car.velocity);
  PRINT(" (x,y): ("); PRINT(car.xPosition); PRINT(", "); PRINT(car.yPosition); PRINT(")");
  PRINT(" Hdg: "); PRINTLN(car.heading);
}

void printCarLn(Car_t car) {
  PRINT("Id: "); PRINTLN(car.id);
  PRINT("Seq: "); PRINTLN(car.sequence);
  PRINT("Time: "); PRINTLN(car.seconds + car.microseconds / 1E6);
  PRINT("Accel: "); PRINTLN(car.acceleration);
  PRINT("Speed: "); PRINTLN(car.velocity);
  PRINT("(x,y): ("); PRINT(car.xPosition); PRINT(", "); PRINT(car.yPosition); PRINTLN(")");
  PRINT("Hdg: "); PRINTLN(car.heading);
}
