#ifndef __TEST_CARS_H__
#define __TEST_CARS_H__

#include "recap_common.h"

#define NUM_TESTS 14

extern Car_t selfCar[NUM_TESTS];
extern Car_t otherCar[NUM_TESTS];
extern int expectedRiskStopping[NUM_TESTS];
extern int expectedRiskHeadway[NUM_TESTS];

void setupTests();

#endif
