#include "recap_DataProc.h"

typedef struct CarNode {
  Car_t car;
  struct CarNode *next;
  struct CarNode *prev;
} CarNode_t;

CarNode_t *carList = NULL;

bool addedDataEnded = true;
int lastDataProcessingTime = 0;
double currentX[TIMESTEPS+1];
double currentY[TIMESTEPS+1];
float currentU[TIMESTEPS+1];

double tempX[TIMESTEPS+1];
double tempY[TIMESTEPS+1];
float tempU[TIMESTEPS+1];

float dist(float x0, float y0, float x1, float y1) {
  return sqrt(pow(x1-x0,2)+pow(y1-y0,2));
}

float haversine(float lat1, float long1, float lat2, float long2) {
  float p = PI/180;
  float a = 0.5 - cos((lat2-lat1)*p)/2.0 + cos(lat1*p) * cos(lat2*p) * (1.0-cos((long2-long1)*p))/2.0;
  return 2 * WORLD_RADIUS * asin(sqrt(a));
}

float dist(const Car_t& car1, const Car_t& car2) {
  return haversine(car1.xPosition, car1.yPosition, car2.xPosition, car2.yPosition);
}

double relativeAngle(double x0, double y0, double x1, double y1) {
  return atan2((y1-y0), (x1-x0));
}

double relativeAngle(const Car_t& car1, const Car_t& car2) {
  return relativeAngle(car1.xPosition, car1.yPosition, car2.xPosition, car2.yPosition);
}

void addNewData(Car_t newData) {
  CarNode_t *currentNode = carList;
  if (currentNode == NULL) {
    carList = new CarNode_t();
    carList->car = newData;
    carList->next = NULL;
    carList->prev = NULL;
  } else if (currentNode->car.id == newData.id) {
    currentNode->car = newData;
  } else {
     while (currentNode->next != NULL) {
      currentNode = currentNode->next;
      if (currentNode->car.id == newData.id) {
        currentNode->car = newData;
        return;
      }
    }

    CarNode_t *newNode = new CarNode_t();
    newNode->car = newData;
    newNode->prev = currentNode;
    newNode->next = NULL;

    currentNode->next = newNode;
  }
}

void calculateCarTrajectory(Car_t c, double x[], double y[], float u[], float dt, float initialT) {
  float r[TIMESTEPS+1];
  float xHeading = cos(c.heading);
  float yHeading = sin(c.heading);

  r[0] = c.velocity*initialT+1/2*c.acceleration*pow(initialT,2);
  x[0] = c.xPosition + r[0]*xHeading;
  y[0] = c.yPosition + r[0]*yHeading;
  u[0] = r[0]*UNCERTAINTY;

  bool carStopped = false;

  for (int i=1;i<=TIMESTEPS;i++) {
    if (!carStopped) {
      float t = initialT+i*dt;
      r[i] = c.velocity*t+1/2*c.acceleration*pow(t,2);
      if ((r[i]-r[i-1])<0) {
        carStopped = true;
        x[i] = x[i-1];
        y[i] = y[i-1];
        u[i] = u[i-1];
      } else {
        x[i] = c.xPosition+r[i]*xHeading;
        y[i] = c.yPosition+r[i]*yHeading;
        u[i] = r[i]*UNCERTAINTY;
      }
    } else {
      x[i] = x[i-1];
      y[i] = y[i-1];
      u[i] = u[i-1];
    }
  }
}

float calculateRisk(double x0[], double y0[], float u0[], double x1[], double y1[], float u1[], float dt, float tMax) {
  float maxRisk = 0;

  for (int i=1;i<=TIMESTEPS;i++) {
    float separation = dist(x0[i], y0[i], x1[i], y1[i]);
    float uncertainty = u0[i]+u1[i];

    if (separation < uncertainty) {
      float risk = (1-(i*dt/tMax*separation/uncertainty))*RISK_SCALE;
      maxRisk = risk > maxRisk ? risk : maxRisk;
    }
  }

  return maxRisk;
}

int processData(Car_t myData) {
  if ((millis() - lastDataProcessingTime) > DATA_PROC_TRANS_INT) {
    lastDataProcessingTime = millis();

    float tMax = myData.velocity/BRAKING_ACCELERATION + REACTION_TIME;
    float dt = tMax/TIMESTEPS;

    calculateCarTrajectory(myData, currentX, currentY, currentU, dt, 0);

    float maxRisk = 0.0;
    CarNode_t *currentNode = carList;

    while (currentNode != NULL) {
      Car_t tempCar = currentNode->car;
      float initialT = (myData.seconds-tempCar.seconds)+(myData.microseconds-tempCar.microseconds)/1000000.0;
      if (initialT > MAX_VALID_TIME) {
        CarNode_t *tempNode = currentNode;
        currentNode = currentNode->next;

        if (tempNode->prev != NULL) {
          tempNode->prev->next = tempNode->next;
        } else {
          carList = tempNode->next;
        }

        if (tempNode->next != NULL) {
          tempNode->next->prev = tempNode->prev;
        }
        delete(tempNode);

      } else {
        float startR = tempCar.velocity*initialT+1/2*tempCar.acceleration*pow(initialT,2);
        float startX = tempCar.xPosition+startR*cos(tempCar.heading);
        float startY = tempCar.yPosition+startR*sin(tempCar.heading);

        float relAngle = myData.heading - relativeAngle(myData.xPosition, myData.yPosition, startX, startY);
        relAngle = abs(fmod((relAngle + PI),(2*PI)) - PI);

        float diffHeading = myData.heading - tempCar.heading;
        diffHeading = abs(fmod((diffHeading + PI),(2*PI)) - PI);

        if (relAngle < MAX_VALID_ANGLE && diffHeading < PI/2) {
          calculateCarTrajectory(tempCar, tempX, tempY, tempU, dt, initialT);
          float carRisk = calculateRisk(currentX, currentY, currentU, tempX, tempY, tempU, dt, tMax);
          maxRisk = carRisk > maxRisk ? carRisk : maxRisk;
        }

        currentNode = currentNode->next;
      }
    }
    return maxRisk;
  }
}

float lastRiskDistance = 0;
float lastRiskTime = 0;
float dx_dt = 0;

float riskHeadway(Car_t self, Car_t other, float distance) {
  float diffA = other.acceleration - self.acceleration;
  float diffV = other.velocity - self.velocity;
  float diffD = distance;

  float a1 = sq(diffV) - 2*diffA*diffD;
  float a2 = sq(self.velocity) - 2*self.acceleration*diffD;
  
  float t;

  if (a1 >= 0) {
    t = -(diffV+sqrt(a1))/diffA;
  } else if (a2 >= 0) {
    t = -(self.velocity+sqrt(a2))/self.acceleration;
  } else {
    return 0;
  }

  float tbraking = self.velocity/BRAKING_ACCELERATION*0.75;
  float risk = (tbraking + 0.15) - t;

  return risk;
}

int assessRisk(Car_t self, Car_t other) {
  float distance = dist(self, other);
  float t = millis()/1000.0;
  const float a = 0.7;
  dx_dt = (a * ((distance - lastRiskDistance) / (t - lastRiskTime))) + ((1-a) * dx_dt);
  
  lastRiskTime = t;
  lastRiskDistance = distance;

  if(abs(dx_dt) > max(self.velocity, other.velocity)) {
    return 0;
  }

  float safetyMargin = 20.0;
  // Required acceleration so that following vehicle will attain the same speed 
  // as the lead vehicle before the relative distance reaches 0.
  float requiredAcceleration = other.acceleration - (sq(self.velocity - other.velocity) / (2 * min(distance-safetyMargin, safetyMargin)));

  float c = 1.0; // Tuning Parameter
  float risk = c * (-requiredAcceleration/BRAKING_ACCELERATION*75);

  return constrain(risk, 0, 100);
}

