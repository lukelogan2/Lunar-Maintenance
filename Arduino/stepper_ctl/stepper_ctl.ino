// Stepper Motor Control Program
// Created by Luke Logan
// September 6, 2022

// Include the AccelStepper library:
#include <AccelStepper.h>

// X-axis stepper
const int PUL_PIN1 = 2; // Yellow
const int DIR_PIN1 = 3; // Green
const int ENA_PIN1 = 22; // Blue
// Y-axis stepper
const int PUL_PIN2 = 4; // Yellow
const int DIR_PIN2 = 5; // Green
const int ENA_PIN2 = 24; // Blue
// Z-axis stepper
const int PUL_PIN3 = 6; // Yellow
const int DIR_PIN3 = 7; // Green
const int ENA_PIN3 = 26; // Blue

// Define stepper motor connections and motor interface type. Motor interface type must be set to 1 when using a driver:
const int motorInterfaceType = 1;

// Create an instance of each stepper
AccelStepper steppers[] = {
  AccelStepper(motorInterfaceType, PUL_PIN1, DIR_PIN1), // X-axis stepper
  AccelStepper(motorInterfaceType, PUL_PIN2, DIR_PIN2), // Y-axis stepper
  AccelStepper(motorInterfaceType, PUL_PIN3, DIR_PIN3)  // Z-axis stepper
};

int stepsPerRev = 200;  // steps per revolution
int distancePerRev = 8; // 8 mm per revolution

// Structure to help define digital pins
typedef struct {
    uint8_t pinNum;
    bool pinVal;
} pinInit_t;

pinInit_t digitPins[] = {
    {PUL_PIN1, LOW},
    {DIR_PIN1, LOW},
    {ENA_PIN1, LOW},
    {PUL_PIN2, LOW},
    {DIR_PIN2, LOW},
    {ENA_PIN2, LOW},
    {PUL_PIN3, LOW},
    {DIR_PIN3, LOW},
    {ENA_PIN3, LOW}
};
pinInit_t pulPins[] = {
  {PUL_PIN1, LOW},
  {PUL_PIN2, LOW},
  {PUL_PIN3, LOW}
};
pinInit_t dirPins[] = {
  {DIR_PIN1, LOW},
  {DIR_PIN2, LOW},
  {DIR_PIN3, LOW}
};
pinInit_t enaPins[] = {
  {ENA_PIN1, LOW},
  {ENA_PIN2, LOW},
  {ENA_PIN3, LOW}
};

void setup() {
  
  for (int i=0;i<sizeof(digitPins)/sizeof(pinInit_t);i++) {
    pinMode(digitPins[i].pinNum, OUTPUT);
    digitalWrite(digitPins[i].pinNum,digitPins[i].pinVal);
  }
  for (int i=0;i<sizeof(steppers)/sizeof(AccelStepper);i++) {
    steppers[i].setMaxSpeed(1000.0);
    steppers[i].setAcceleration(100.0);
  }
  
  
  Serial.begin(9600); // opens serial port, sets data rate to 9600 bps
}

void loop() {
  /*
  stepperX.moveTo(stepsPerRev);
  stepperX.setSpeed(200);
  stepperX.runSpeedToPosition();
  */

  setPosition(20.0,20.0,20.0);
}

/*
 * setPosition moves the tool to an (x,y,z) coordinate
 * (x,y,z) in mm
 */
void setPosition(double x, double y, double z) {
  steppers[0].moveTo(x*stepsPerRev/distancePerRev);
  steppers[1].moveTo(y*stepsPerRev/distancePerRev);
  steppers[2].moveTo(z*stepsPerRev/distancePerRev);
  
  for (int i=0;i<sizeof(steppers)/sizeof(AccelStepper);i++) {
    steppers[i].setSpeed(200);
    steppers[i].runSpeedToPosition();
  }
  
}
