#include <SoftwareSerial.h>
#include <SPI.h>

#define o1 7
#define o2 8
SoftwareSerial BTserial(5,6); // 5 for tx, 4 rx

int state = 0;

void setup() {
  // put your setup code here, to run once:
  BTserial.begin(38400);
  Serial.begin(9600);
  Serial.println("Hello");
  
  SPI.begin();
  pinMode(o1, OUTPUT);
  pinMode(o2, OUTPUT);
}

void loop() {
  // put your main code here, to run repeatedly:
  if(Serial.available() > 0){ // Checks whether data is comming from the serial port
    state = Serial.read(); // Reads the data from the serial port
  }
  if (state == '1') {
    digitalWrite(o1, LOW); // Turn H-bridge OFF
    digitalWrite(o2, LOW);
    Serial.println("OFF");
  }
  else if (state == '0') {
    analogWrite(o1, 50);
    digitalWrite(o2, LOW);
    Serial.println("ON");
  } 
  delay(200);
}
