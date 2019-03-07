#include "test_cars.h"

Car_t selfCar[NUM_TESTS];
Car_t otherCar[NUM_TESTS];
int expectedRiskStopping[NUM_TESTS];
int expectedRiskHeadway[NUM_TESTS];

void setupTests() {
  for (int i = 0; i < NUM_TESTS; i++) {
    // id, sequence, micros, seconds, acceleration, velocity, x, y, hdg
    selfCar[i] = {0,0,0,0,0,0,0,0,0};
    otherCar[i] = {0,0,0,0,0,0,0,0,0};
    expectedRiskStopping[i] = 0;
    expectedRiskHeadway[i] = 0;
  }

  int test_num = 0;
  // test #0: velocity = 0, far away => risk = 0
  selfCar[test_num].velocity  = 0; selfCar[test_num].acceleration  = 0;
  otherCar[test_num].velocity = 0; otherCar[test_num].acceleration = 0;

  selfCar[test_num].xPosition  = 0; selfCar[test_num].yPosition  = 0;
  otherCar[test_num].xPosition = 0; otherCar[test_num].yPosition = 1000;

  expectedRiskStopping[test_num] = 0;
  expectedRiskHeadway[test_num]  = 0;

  // test #1: velocity = 0, 10 m away => risk = ...?
  test_num++;
  selfCar[test_num].velocity  = 0; selfCar[test_num].acceleration  = 0;
  otherCar[test_num].velocity = 0; otherCar[test_num].acceleration = 0;

  selfCar[test_num].xPosition  = 0; selfCar[test_num].yPosition  = 0;
  otherCar[test_num].xPosition = 0; otherCar[test_num].yPosition = 10;

  expectedRiskStopping[test_num] = 0;
  expectedRiskHeadway[test_num]  = 0;

  // test #2: velocity = 0, 0 m away => risk = 0
  test_num++;
  selfCar[test_num].velocity  = 0; selfCar[test_num].acceleration  = 0;
  otherCar[test_num].velocity = 0; otherCar[test_num].acceleration = 0;

  selfCar[test_num].xPosition  = 0; selfCar[test_num].yPosition  = 0;
  otherCar[test_num].xPosition = 0; otherCar[test_num].yPosition = 0;

  expectedRiskStopping[test_num] = 0;
  expectedRiskHeadway[test_num]  = 0;

  // test #3: same velocity, far away => risk = 0
  test_num++;
  selfCar[test_num].velocity  = 28; selfCar[test_num].acceleration  = 0;
  otherCar[test_num].velocity = 28; otherCar[test_num].acceleration = 0;

  selfCar[test_num].xPosition  = 0; selfCar[test_num].yPosition  = 0;
  otherCar[test_num].xPosition = 0; otherCar[test_num].yPosition = 1000;

  expectedRiskStopping[test_num] = 0;
  expectedRiskHeadway[test_num]  = 0;

  // test #4: same velocity = 100.8 km/h, close => risk = 50
  test_num++;
  selfCar[test_num].velocity  = 28; selfCar[test_num].acceleration  = 0;
  otherCar[test_num].velocity = 28; otherCar[test_num].acceleration = 0;

  selfCar[test_num].xPosition  = 0; selfCar[test_num].yPosition  = 0;
  otherCar[test_num].xPosition = 0; otherCar[test_num].yPosition = 50;

  expectedRiskStopping[test_num] = 0;
  expectedRiskHeadway[test_num]  = 50;

  // test #5: same velocity = 100.8 km/h, very close => risk = 100
  test_num++;
  selfCar[test_num].velocity  = 28; selfCar[test_num].acceleration  = 0;
  otherCar[test_num].velocity = 28; otherCar[test_num].acceleration = 0;

  selfCar[test_num].xPosition  = 0; selfCar[test_num].yPosition  = 0;
  otherCar[test_num].xPosition = 0; otherCar[test_num].yPosition = 10;

  expectedRiskStopping[test_num] = 0;
  expectedRiskHeadway[test_num]  = 100;

  // test #6: same velocity = 100.8 km/h, dist = 0 => risk = 100
  test_num++;
  selfCar[test_num].velocity  = 28; selfCar[test_num].acceleration  = 0;
  otherCar[test_num].velocity = 28; otherCar[test_num].acceleration = 0;

  selfCar[test_num].xPosition  = 0; selfCar[test_num].yPosition  = 0;
  otherCar[test_num].xPosition = 0; otherCar[test_num].yPosition = 0;

  expectedRiskStopping[test_num] = 0;
  expectedRiskHeadway[test_num]  = 100;

  // test #7: different velocities (we are slower), far away => risk = 0
  test_num++;
  selfCar[test_num].velocity  = 20; selfCar[test_num].acceleration  = 0;
  otherCar[test_num].velocity = 28; otherCar[test_num].acceleration = 0;

  selfCar[test_num].xPosition  = 0; selfCar[test_num].yPosition  = 0;
  otherCar[test_num].xPosition = 0; otherCar[test_num].yPosition = 1000;

  expectedRiskStopping[test_num] = 0;
  expectedRiskHeadway[test_num]  = 0;

  // test #8: different velocities (we are slower), close => risk = 0
  test_num++;
  selfCar[test_num].velocity  = 20; selfCar[test_num].acceleration  = 0;
  otherCar[test_num].velocity = 28; otherCar[test_num].acceleration = 0;

  selfCar[test_num].xPosition  = 0; selfCar[test_num].yPosition  = 0;
  otherCar[test_num].xPosition = 0; otherCar[test_num].yPosition = 100;

  expectedRiskStopping[test_num] = 0;
  expectedRiskHeadway[test_num]  = 0;

  // test #9: different velocities (we are slower), closer => risk = 25
  test_num++;
  selfCar[test_num].velocity  = 20; selfCar[test_num].acceleration  = 0;
  otherCar[test_num].velocity = 28; otherCar[test_num].acceleration = 0;

  selfCar[test_num].xPosition  = 0; selfCar[test_num].yPosition  = 0;
  otherCar[test_num].xPosition = 0; otherCar[test_num].yPosition = 50;

  expectedRiskStopping[test_num] = 0;
  expectedRiskHeadway[test_num]  = 25;

  // test #10: different velocities (we are faster), far away => risk = 0
  test_num++;
  selfCar[test_num].velocity  = 28; selfCar[test_num].acceleration  = 0;
  otherCar[test_num].velocity = 20; otherCar[test_num].acceleration = 0;

  selfCar[test_num].xPosition  = 0; selfCar[test_num].yPosition  = 0;
  otherCar[test_num].xPosition = 0; otherCar[test_num].yPosition = 1000;

  expectedRiskStopping[test_num] = 0;
  expectedRiskHeadway[test_num]  = 0;

  // test #11: different velocities (we are faster), close => risk = 50
  test_num++;
  selfCar[test_num].velocity  = 28; selfCar[test_num].acceleration  = 0;
  otherCar[test_num].velocity = 20; otherCar[test_num].acceleration = 0;

  selfCar[test_num].xPosition  = 0; selfCar[test_num].yPosition  = 0;
  otherCar[test_num].xPosition = 0; otherCar[test_num].yPosition = 100;

  expectedRiskStopping[test_num] = 0;
  expectedRiskHeadway[test_num]  = 0;

  // test #12: different velocities (we are faster), closer => risk = 50
  test_num++;
  selfCar[test_num].velocity  = 28; selfCar[test_num].acceleration  = 0;
  otherCar[test_num].velocity = 20; otherCar[test_num].acceleration = 0;

  selfCar[test_num].xPosition  = 0; selfCar[test_num].yPosition  = 0;
  otherCar[test_num].xPosition = 0; otherCar[test_num].yPosition = 50;

  expectedRiskStopping[test_num] = 0;
  expectedRiskHeadway[test_num]  = 50;

  // test #13: different velocities and we are accelerating, closer => risk = 56
  test_num++;
  selfCar[test_num].velocity  = 28; selfCar[test_num].acceleration  = 3;
  otherCar[test_num].velocity = 20; otherCar[test_num].acceleration = 0;

  selfCar[test_num].xPosition  = 0; selfCar[test_num].yPosition  = 0;
  otherCar[test_num].xPosition = 0; otherCar[test_num].yPosition = 50;

  expectedRiskStopping[test_num] = 0;
  expectedRiskHeadway[test_num]  = 56;
}
