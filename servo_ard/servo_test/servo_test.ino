const int IN1 = 3;

void setup() {
  // put your setup code here, to run once:
  pinMode (IN1, OUTPUT);
}

void loop() {
  // put your main code here, to run repeatedly:
  analogWrite(IN1, 90);
}
