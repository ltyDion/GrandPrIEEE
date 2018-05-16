#define in1 7
#define in2 8
#define out1 45
void setup() {
  // put your setup code here, to run once:
  pinMode(out1, OUTPUT);
  pinMode(in1, OUTPUT);
  pinMode(in2, OUTPUT);

  digitalWrite(in1, LOW);
  digitalWrite(in2, HIGH);
}

void loop() {
  // put your main code here, to run repeatedly:
  analogWrite(out1, 50); //pwm funciton to control the speed
  delay(100);
}
