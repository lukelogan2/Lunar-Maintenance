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
bool startFlag = false; // Wait until the user makes a decision to zero
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
byte motor_zeroing = 0; // Initialize Z-axis to first motor zeroing

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
  
  // Create interrupt service routines for limit switches
  pinMode(LIMIT_PIN_XMIN, INPUT_PULLUP);
  pinMode(LIMIT_PIN_XMAX, INPUT_PULLUP);
  pinMode(LIMIT_PIN_YMIN, INPUT_PULLUP);
  pinMode(LIMIT_PIN_YMAX, INPUT_PULLUP);
  pinMode(LIMIT_PIN_ZMIN, INPUT_PULLUP);
  pinMode(LIMIT_PIN_ZMAX, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(LIMIT_PIN_XMIN), limit_xmin, RISING);
  attachInterrupt(digitalPinToInterrupt(LIMIT_PIN_XMAX), limit_xmax, RISING);
  attachInterrupt(digitalPinToInterrupt(LIMIT_PIN_YMIN), limit_ymin, RISING);
  attachInterrupt(digitalPinToInterrupt(LIMIT_PIN_YMAX), limit_ymax, RISING);
  attachInterrupt(digitalPinToInterrupt(LIMIT_PIN_ZMIN), limit_zmin, RISING);
  attachInterrupt(digitalPinToInterrupt(LIMIT_PIN_ZMAX), limit_zmax, RISING);
  
  Serial.begin(115200);
}

void loop() {
  if (startFlag) {
    // Read position from Python driver program
    if (DataRead) {
      DataRead = false;
      xpos = data.substring(0,6).toInt();
      ypos = data.substring(6,12).toInt();
      zpos = data.substring(12,18).toInt();
      tool_pos = data.substring(18,24).toInt();
      sol_pos = data.substring(24,25).toInt();
      char msg[50];
      sprintf(msg,"X = %d\nY = %d\nZ = %d\nTool = %d\nSolenoid = %d",xpos,ypos,zpos,tool_pos,sol_pos);
      //Serial.println(msg);
    }
    if (zeroed[0] && zeroed[1] && zeroed[2]) {
        setPosition(xpos,ypos,zpos);
        myservo.write(tool_pos);
        digitalWrite(valve_pin,sol_pos);
    }
    else {
      if (!zeroed[2]) {
        zeroZ();
      }
      else if (!zeroed[0]) {
        zeroX();
      }
      else if (!zeroed[1]) {
        zeroY();
      }
    }
  }
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

/*
 * setPosition moves the tool to an (x,y,z) coordinate
 * (x,y,z) in mm
 */
void setPosition(double x, double y, double z) {
  bool stopFlag = false;
  for (int i=0; i<sizeof(flags)/sizeof(flags[0]);i++) {
    if (flags[i] == true) {
      stopFlag = true;
    }
  }
  if (stopFlag == false) {
    const byte rps = 1; // Set rotations per second
    driveMotors(x,y,z,rps);
  }
  else {
    for (int i=0;i<sizeof(flags)/sizeof(flags[0]);i++) {
      if (flags[i] == true) {
        byte motor = -1;
        switch (i) {
          case 0:
            motor = 0; // Xmin
          break;
          case 1:
            motor = 1; // Ymin
          break;
          case 2:
            motor = 2; // Zmin
          break;
          case 3: // Xmax
            motor = 0;
          break;
          case 4: // Ymax
            motor = 1;
          break;
          case 5: // Zmax
            motor = 2;
          break;
          default:
            // None
          break;
        }
        //digitalWrite(enaPins[i].pinNum,HIGH);
        steppers[motor].setCurrentPosition(steppers[motor].targetPosition());
        flags[i] = false;
        char msg[50];
        sprintf(msg,"Limit Switch %d has triggered",i);
        Serial.println(msg);
      }
    }
  }
}


void zeroX() {
  byte motor = 0;
  if (flags[motor] == false) {
    driveMotors(xpos,ypos,zpos,0.5);
  }
  else {
    steppers[motor].setCurrentPosition(steppers[motor].targetPosition());
    zeroed[motor] = true;
    flags[motor] = false;
  }
  // X-axis zeroing
  if (!zeroed[motor]) { 
    xpos = -1000; 
  }
  else { 
    xpos = 0; 
    steppers[motor].setCurrentPosition(0);
    Serial.println("zeroX");
  }
}

void zeroY() {
  byte motor = 1;
  if (flags[motor] == false) {
    driveMotors(xpos,ypos,zpos,0.5);
  }
  else {
    steppers[motor].setCurrentPosition(steppers[motor].targetPosition());
    zeroed[motor] = true;
    flags[motor] = false;
  }
  // Y-axis zeroing
  if (!zeroed[motor]) { 
    ypos = -1000; 
  }
  else { 
    ypos = 0; 
    steppers[motor].setCurrentPosition(0);
    Serial.println("zeroY");
  }
}

void zeroZ() {
  byte motor = 2;
  if (flags[motor] == false) {
    driveMotors(xpos,ypos,zpos,0.5);
  }
  else {
    steppers[motor].setCurrentPosition(steppers[motor].targetPosition());
    zeroed[motor] = true;
    flags[motor] = false;
  }
  // Z-axis zeroing
  if (!zeroed[motor]) { 
    zpos = -1000; 
  }
  else { 
    zpos = 0; 
    steppers[motor].setCurrentPosition(0);
    Serial.println("zeroZ");
  }
}


/*
 * Serial Communication Handler
 *   Reads data from Python driver program
 */
void serialEvent() {
  // If a message from the Raspberry Pi is present, read the data
  if (Serial.available() > 0) {
    data = Serial.readString();
    if (data.equals("zero")) {
      zeroed[0] = false;
      zeroed[1] = false;
      zeroed[2] = false;
      flags[0] = false;
      flags[1] = false;
      flags[2] = false;
      allZeroed = false;
      startFlag = true;
    }
    else if (data.equals("nozero")) {
      startFlag = true;
    }
    else {
      DataRead = true;
    }
  }
  // Reset the serial buffer
  Serial.flush();
}

/*
 * Interrupt Service Routines for limit switches
 */
void limit_xmin() {
  if (!zeroed[0]) {
    flags[0] = digitalRead(LIMIT_PIN_XMIN);
  }
}
void limit_ymin() {
  if (!zeroed[1]) {
    flags[1] = digitalRead(LIMIT_PIN_YMIN);
  }
}
void limit_zmin() {
  if (!zeroed[2]) {
    flags[2] = digitalRead(LIMIT_PIN_ZMIN);
  }
}
void limit_xmax() {
  flags[3] = digitalRead(LIMIT_PIN_XMAX);
}
void limit_ymax() {
  flags[4] = digitalRead(LIMIT_PIN_YMAX);
}
void limit_zmax() {
  flags[5] = digitalRead(LIMIT_PIN_ZMAX);
}
