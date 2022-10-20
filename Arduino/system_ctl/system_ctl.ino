// Stepper Motor Control Program
// Created by Luke Logan
// September 6, 2022

// Include the AccelStepper library:
#include <AccelStepper.h>
#include <Servo.h>

Servo myservo;  // create servo object to control a servo

// X-axis stepper
const int PUL_PINX = 2; // Yellow
const int DIR_PINX = 3; // Green
const int ENA_PINX = 22; // Blue
// Y-axis stepper
const int PUL_PINY = 4; // Yellow
const int DIR_PINY = 5; // Green
const int ENA_PINY = 24; // Blue
// Z-axis stepper
const int PUL_PINZ = 6; // Yellow
const int DIR_PINZ = 7; // Green
const int ENA_PINZ = 26; // Blue

// Define stepper motor connections and motor interface type. Motor interface type must be set to 1 when using a driver:
const int motorInterfaceType = 1;

// Create an instance of each stepper
AccelStepper steppers[] = {
  AccelStepper(motorInterfaceType, PUL_PINX, DIR_PINX), // X-axis stepper
  AccelStepper(motorInterfaceType, PUL_PINY, DIR_PINY), // Y-axis stepper
  AccelStepper(motorInterfaceType, PUL_PINZ, DIR_PINZ)  // Z-axis stepper
};

int stepsPerRev = 200;  // steps per revolution
int distancePerRev = 2; // 2 mm per revolution
String data;            // Data input from python
bool DataRead = false;

// Position Variables
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

// Pins for limit switch interrupts
const byte LIMIT_PINX = 18;
const byte LIMIT_PINY = 19;
const byte LIMIT_PINZ = 20;

// Limit switch flags X,Y,Z
bool flags[] = {false,false,false};

// Zeroing Variables for X,Y,Z
bool zeroed[] = {false,false,false};
long zeroPosition[] {0,0,0};

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
  myservo.attach(9);  // attaches the servo on pin 9 to the servo object
  
  // Create interrupt service routines for limit switches
  pinMode(LIMIT_PINX, INPUT_PULLUP);
  pinMode(LIMIT_PINY, INPUT_PULLUP);
  pinMode(LIMIT_PINZ, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(LIMIT_PINX), limit_switchX, CHANGE);
  attachInterrupt(digitalPinToInterrupt(LIMIT_PINY), limit_switchY, CHANGE);
  attachInterrupt(digitalPinToInterrupt(LIMIT_PINZ), limit_switchZ, CHANGE);
  
  Serial.begin(115200);
}

void loop() {
  // Read position from Python driver program
  if (DataRead) {
    DataRead = false;
    xpos = data.substring(0,3).toInt();
    ypos = data.substring(3,6).toInt();
    zpos = data.substring(6,9).toInt();
    tool_pos = data.substring(9,12).toInt();
    char msg[50];
    sprintf(msg,"X = %d\nY = %d\nZ = %d\nTool = %d",xpos,ypos,zpos,tool_pos);
    Serial.println(msg);
  }

  //if (zeroed[0] && zeroed[1] && zeroed[2]) {
  if (zeroed[1]) {
    setPosition(xpos,ypos,zpos);
    myservo.write(tool_pos);
  }
  else {
    zeroMotors();
  }
  //setPosition(xpos,ypos,zpos);
}

/*
 * Function driveMotors(x,y,z)
 *  Drives motors to a position x,y,z
 */
void driveMotors(double x, double y, double z) {
  steppers[0].moveTo(x*stepsPerRev/distancePerRev);
  steppers[1].moveTo(y*stepsPerRev/distancePerRev);
  steppers[2].moveTo(z*stepsPerRev/distancePerRev);
  
  for (int i=0;i<sizeof(steppers)/sizeof(AccelStepper);i++) {
    steppers[i].setSpeed(200);
    steppers[i].runSpeedToPosition();
  }
}

/*
 * setPosition moves the tool to an (x,y,z) coordinate
 * (x,y,z) in mm
 */
void setPosition(double x, double y, double z) {
  if (!flags[0] && !flags[1] && !flags[2]) {
    driveMotors(x,y,z);
  }
  else {
    for (int i=1;i<sizeof(flags)/sizeof(flags[0]);i++) {
      if (flags[i] == true) {
        digitalWrite(enaPins[i].pinNum,HIGH);
        steppers[i].setCurrentPosition(steppers[i].targetPosition());
        flags[i] = false;
      }
    }
  }
}

/*
 * zeroMotors() Function
 *    Drive motors until they hit limit switches, 
 *    then set the zero positions for each motor
 */
void zeroMotors() {
  if (!flags[0] && !flags[1] && !flags[2]) {
    driveMotors(xpos,ypos,zpos);
  }
  else {
    for (int i=1;i<sizeof(flags)/sizeof(flags[0]);i++) {
      if (flags[i] == true) {
        digitalWrite(enaPins[i].pinNum,HIGH);
        zeroPosition[i] = steppers[i].currentPosition();
        steppers[i].setCurrentPosition(0);
        flags[i] = false;
        zeroed[i] = true;
      }
    }
  }
  if (!zeroed[0]) { xpos = 1000; }
  else { xpos = 0; }
  if (!zeroed[1]) { ypos = 1000; }
  else { ypos = 0; }
  if (!zeroed[2]) { zpos = 1000; }
  else { zpos = 0; }
}

/*
 * Serial Communication Handler
 *   Reads data from Python driver program
 */
void serialEvent() {
  // If a message from the Raspberry Pi is present, read the data
  if (Serial.available() > 0) {
    data = Serial.readString();
    DataRead = true;
  }
  // Reset the serial buffer
  Serial.flush();
}

/*
 * Interrupt Service Routines for limit switches
 */
void limit_switchX() {
  flags[0] = digitalRead(LIMIT_PINX);
}
void limit_switchY() {
  flags[1] = digitalRead(LIMIT_PINY);
}
void limit_switchZ() {
  flags[2] = digitalRead(LIMIT_PINZ);
}
