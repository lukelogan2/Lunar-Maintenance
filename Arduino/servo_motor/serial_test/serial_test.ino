int x=1;
bool DataRead = false;
void setup() {
 Serial.begin(115200);
 Serial.setTimeout(1);
}
void loop() {
 //while (!Serial.available());
 if (DataRead) {
  Serial.println(x);
  DataRead = false;
 }
}

void serialEvent() {
  // If a message from the Raspberry Pi is present, read the data
  if (Serial.available() > 0) {
    x = Serial.readString().toInt();
    DataRead = true;
  }
  // Reset the serial buffer
  Serial.flush();
}
