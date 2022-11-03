const byte relay_pin = 50;

void setup() {
  // put your setup code here, to run once:
  pinMode(relay_pin,OUTPUT);
}

void loop() {
  // put your main code here, to run repeatedly:
  digitalWrite(relay_pin,HIGH);
}
