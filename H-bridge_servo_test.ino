#include <Servo.h>
Servo myServo;

void setup() {
  // put your setup code here, to run once:
  pinMode(12, OUTPUT);
  pinMode(45, OUTPUT);
  pinMode(47, OUTPUT);
  myServo.attach(9);
  myServo.write(0);
}

void loop() {
  // put your main code here, to run repeatedly:
  
  digitalWrite(45, HIGH);
  digitalWrite(47, LOW);
  analogWrite(12, 25);

  for(int i = 0 ; i < 180; ++i){
    myServo.write(i);  
  }

  
  delay(200);
}
