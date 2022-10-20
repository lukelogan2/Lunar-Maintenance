#include <Servo.h>

Servo myservo;  // create servo object to control a servo
// twelve servo objects can be created on most boards

int pos = 0;    // variable to store the servo position
bool DataRead = false;

void setup() {
  myservo.attach(9);  // attaches the servo on pin 9 to the servo object
  Serial.begin(115200);
}

void loop() {
  // Position 100 = clamp fully closed
  // Position 0 = clamp fully open
  myservo.write(pos);
  if (DataRead) {
    DataRead = false;
    Serial.println(pos);
  }
}

// Serial Communication Handler
void serialEvent() {
  // If a message from the Raspberry Pi is present, read the data
  if (Serial.available() > 0) {
    pos = Serial.readString().toInt();
    DataRead = true;
  }
  // Reset the serial buffer
  Serial.flush();
}
