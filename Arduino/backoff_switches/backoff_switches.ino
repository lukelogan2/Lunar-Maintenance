// Stepper Motor Control Program
// Created by Luke Logan
// September 6, 2022

// Include the AccelStepper library:
#include <AccelStepper.h>
#include <Servo.h>

// X-axis stepper
const int PUL_PINX = 4; // Yellow
const int DIR_PINX = 5; // Green
const int ENA_PINX = 22; // Blue
// Y-axis stepper
const int PUL_PINY = 6; // Yellow
const int DIR_PINY = 7; // Green
const int ENA_PINY = 24; // Blue
// Z-axis stepper
const int PUL_PINZ = 8; // Yellow
const int DIR_PINZ = 9; // Green
const int ENA_PINZ = 26; // Blue
// Servo Motor Tool
const int TOOL_PIN = 10;
// Pins for limit switch interrupts
const byte LIMIT_PIN_XMIN = 2;
const byte LIMIT_PIN_YMIN = 3;
const byte LIMIT_PIN_ZMIN = 18;
const byte LIMIT_PIN_XMAX = 19;
const byte LIMIT_PIN_YMAX = 20;
const byte LIMIT_PIN_ZMAX = 21;
// Solenoid Pin
const byte valve_pin = 50;
byte sol_pos = 0;

// Define stepper motor connections and motor interface type. Motor interface type must be set to 1 when using a driver:
const int motorInterfaceType = 1;

// Create an instance of each stepper
AccelStepper steppers[] = {
  AccelStepper(motorInterfaceType, PUL_PINX, DIR_PINX), // X-axis stepper
  AccelStepper(motorInterfaceType, PUL_PINY, DIR_PINY), // Y-axis stepper
  AccelStepper(motorInterfaceType, PUL_PINZ, DIR_PINZ)  // Z-axis stepper
};

Servo myservo;  // create servo object to control a servo

int stepsPerRev = 200;  // steps per revolution
int distancePerRev = 8; // 8 mm per revolution
String data;            // Data input from python
bool DataRead = false;

// Position Variables in mm
int xpos = 0;
int ypos = 0;
int zpos = 0;
int tool_pos = 0;

// Structure to help define digital pins
typedef struct {
    uint8_t pinNum;
    bool pinVal;
} pinInit_t;

pinInit_t digitPins[] = {
    {PUL_PINX, LOW},
    {DIR_PINX, LOW},
    {ENA_PINX, LOW},
    {PUL_PINY, LOW},
    {DIR_PINY, LOW},
    {ENA_PINY, LOW},
    {PUL_PINZ, LOW},
    {DIR_PINZ, LOW},
    {ENA_PINZ, LOW}
};
pinInit_t pulPins[] = {
  {PUL_PINX, LOW},
  {PUL_PINY, LOW},
  {PUL_PINZ, LOW}
};
pinInit_t dirPins[] = {
  {DIR_PINX, LOW},
  {DIR_PINY, LOW},
  {DIR_PINZ, LOW}
};
pinInit_t enaPins[] = {
  {ENA_PINX, LOW},
  {ENA_PINY, LOW},
  {ENA_PINZ, LOW}
};

// Limit switch flags X,Y,Z
bool flags[] = {false,false,false,false,false,false};

// Zeroing Variables for X,Y,Z
//bool zeroed[] = {false,false,false};
bool zeroed[] = {true,true,true}; // Wait for the python code to initialize zeroing
bool allZeroed = true;
long zeroPosition[] {0,0,0};
int backoff[] = {
  -5*stepsPerRev/distancePerRev, // Xmin
  5*stepsPerRev/distancePerRev,  // Ymin
  -3*stepsPerRev/distancePerRev, // Zmin
  5*stepsPerRev/distancePerRev,  // Xmax
  -5*stepsPerRev/distancePerRev, // Ymax
  3*stepsPerRev/distancePerRev  // Zmax
};
byte motor_zeroing = 1; // Initialize Y-axis to first motor zeroing

void setup() {
  // Set the output of each motor pin to LOW
  for (int i=0;i<sizeof(digitPins)/sizeof(pinInit_t);i++) {
    pinMode(digitPins[i].pinNum, OUTPUT);
    digitalWrite(digitPins[i].pinNum,digitPins[i].pinVal);
  }
  // Set the max speed and acceleration of the motors
  for (int i=0;i<sizeof(steppers)/sizeof(AccelStepper);i++) {
    steppers[i].setMaxSpeed(1000.0);
    steppers[i].setAcceleration(100.0);
  }
  myservo.attach(TOOL_PIN);  // attaches the servo on pin 9 to the servo object

  // Initialize solenoid to close
  pinMode(valve_pin,OUTPUT);
  digitalWrite(valve_pin,sol_pos);
  
  Serial.begin(115200);
}

void loop() {
  driveMotors(10,10,10,0.5);
}

/*
 * Function driveMotors(x,y,z)
 *  Drives motors to a position x,y,z
 *  v is the speed in revolutions per second
 */
void driveMotors(double x, double y, double z, double v) {
  // A negative input moves the gantry up
  // A positive input moves the gantry down
  steppers[0].moveTo(x*stepsPerRev/distancePerRev);
  steppers[1].moveTo(-y*stepsPerRev/distancePerRev);
  steppers[2].moveTo(z*stepsPerRev/distancePerRev);
  
  for (int i=0;i<sizeof(steppers)/sizeof(AccelStepper);i++) {
    if (i == 2) {
      steppers[i].setSpeed(stepsPerRev*0.5);
    }
    else {
      steppers[i].setSpeed(stepsPerRev*v);
    }
    steppers[i].runSpeedToPosition();
  }
}
