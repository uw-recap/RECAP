// the setup function runs once when you press reset or power the board
typedef struct {
  int id;
  int sequence;
  int microseconds; // usec
  int seconds; //sec
  float acceleration; // m/s^2
  float velocity; //m/s
  float xPosition; //unitless (need to multiply by radius of earth)
  float yPosition; //unitless (need to multiply by radius of earth)
  float heading; //radians
} Car_t;

typedef struct CarNode {
  Car_t car;
  struct CarNode *next;
  struct CarNode *prev;
} CarNode_t;

//Testing
const int TOTAL_CARS = 100;

const int R = 6371; //Radius of earth in km
const float VARIANCE = 10;
const float MAX_VALID_TIME = 2.0;
const float MAX_VALID_DIST = 10.0;
const float MAX_VALID_ANGLE = PI/6;
const float BRAKING_ACCELERATION = 3.4;
const float REACTION_TIME = 0.15;
const int TIMESTEPS = 100;
const float UNCERTAINTY = 0.2;
const float RISK_SCALE = 100;

long startTime = 0;
long endTime = 0;
long duration = 0;

Car_t car0;
CarNode_t *carList = NULL;

void setup() {
  Serial.begin(9600);
  randomSeed(analogRead(0));

  carList = new CarNode_t();
  carList->car = randomCar(0);
  carList->next = NULL;
  carList->prev = NULL;
  
  CarNode_t *currentNode = carList;
  
  for (int i=1; i<TOTAL_CARS; i++) {
    CarNode_t *tempNode = new CarNode_t();

    tempNode->car = randomCar(i);
    tempNode->prev = currentNode;
    tempNode->next = NULL;

    currentNode->next = tempNode;
    currentNode = currentNode->next;
    
    delay(20);
  }

  car0.id = -1;
  car0.xPosition = 0;
  car0.yPosition = 0;
  car0.heading = 0;
  car0.seconds = 0;
  car0.microseconds = millis()*1000;
  car0.acceleration = 0;
  car0.velocity = random(10);
}

// the loop function runs over and over again forever
void loop() {
  car0.velocity = random(1000)/100.0;
  startTime = millis();

  float tMax = car0.velocity/BRAKING_ACCELERATION + REACTION_TIME;
  float dt = tMax/TIMESTEPS;

  float x0[TIMESTEPS+1];
  float y0[TIMESTEPS+1];
  float uncertainty0[TIMESTEPS+1];

  float x1[TIMESTEPS+1];
  float y1[TIMESTEPS+1];
  float uncertainty1[TIMESTEPS+1];

  calculateCarTrajectory(car0, x0, y0, uncertainty0, dt, 0);
  CarNode_t *currentNode = carList;

  float maxRisk = 0.0;

  while (currentNode != NULL) {
    Car_t tempCar = currentNode->car;
    float initialT = (car0.seconds-tempCar.seconds)+(car0.microseconds-tempCar.microseconds)/1000000.0;
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
      float startX = startR*cos(tempCar.heading);
      float startY = startR*sin(tempCar.heading);
      float relAngle = car0.heading - relativeAngle(car0.xPosition, car0.yPosition, startX, startY);
      relAngle = abs(fmod((relAngle + PI),(2*PI)) - PI);
      float diffHeading = car0.heading - tempCar.heading;
      diffHeading = abs(fmod((diffHeading + PI),(2*PI)) - PI);
      if (relAngle < MAX_VALID_ANGLE && diffHeading < PI/2) {
        calculateCarTrajectory(tempCar, x1, y1, uncertainty1, dt, initialT);
        float carRisk = calculateRisk(x0, y0, uncertainty0, x1, y1, uncertainty1, dt, tMax);
        maxRisk = carRisk > maxRisk ? carRisk : maxRisk;
      }
      
      currentNode = currentNode->next;
    }
  }

  Serial.println(maxRisk);
  
  endTime = millis();
  duration = endTime - startTime;
  Serial.println(duration); 
  delay(1000);
}

Car_t randomCar(int id) {
  Car_t c;
  c.id = id;
  c.seconds = 0;
  c.microseconds = millis()*1000;
  c.acceleration = -random(10)/5.0;
  c.velocity = random(10)-5.0;
  c.xPosition = (random(200)-100)/10.0;
  c.yPosition = (random(200)-100)/10.0;
  c.heading = random(200)/200.0*PI-PI/2;
  
  return c;
}

void calculateCarTrajectory(Car_t c, float x[], float y[], float u[], float dt, float initialT) {
  float r[TIMESTEPS+1];
  
  x[0] = c.xPosition;
  y[0] = c.yPosition;
  r[0] = c.velocity*initialT+1/2*c.acceleration*pow(initialT,2);
  u[0] = r[0]*UNCERTAINTY;

  float xHeading = cos(c.heading);
  float yHeading = sin(c.heading);
  
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

float dist(float x0, float y0, float x1, float y1) {
  return sqrt(pow(x1-x0,2)+pow(y1-y0,2));
}

float relativeAngle(float x0, float y0, float x1, float y1) {
  return atan2((y1-y0), (x1-x0));
}

float calculateRisk(float x0[], float y0[], float u0[], float x1[], float y1[], float u1[], float dt, float tMax) {
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

float normalDistribution(float x, float mean, float variance) {
  return 1/(variance*sqrt(2*PI))*exp(-0.5*pow(((x-mean)/variance),2));
}

float gaussianDistribution(float x, float mean, float variance) {
  return exp(-0.5*pow(((x-mean)/variance),2));
}
