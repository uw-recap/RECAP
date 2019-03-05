#ifndef _RECAP_DATA_PROC_H_
#define _RECAP_DATA_PROC_H_

#include "recap_common.h"

#define VARIANCE 10.0
#define MAX_VALID_TIME 2.0
#define MAX_VALID_DIST 10.0
#define MAX_VALID_ANGLE PI/6
#define BRAKING_ACCELERATION 3.4
#define REACTION_TIME 0.15
#define TIMESTEPS 100
#define UNCERTAINTY 0.2
#define RISK_SCALE 100.0
#define DATA_PROC_TRANS_INT 200
#define CAR_ID 13

double dist(double x0, double y0, double x1, double y1);
double dist(const Car_t& car1, const Car_t& car2);

double relativeAngle(double x0, double y0, double x1, double y1);
double relativeAngle(const Car_t& car1, const Car_t& car2);

void addNewData(Car_t newData);

void calculateCarTrajectory(Car_t c, double x[], double y[], float dt, float initialT);
void calculateRisk(double x0[], double y0[], float u0[], double x1[], double y1[], double u1[], float dt, float tMax);

int processData(Car_t myData);

#endif
