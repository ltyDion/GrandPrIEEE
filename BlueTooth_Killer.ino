#define o1 4
#define o2 5
#define motorOut 45
int state = 0;

void setup() {
  // put your setup code here, to run once:
  Serial.begin(38400);
  pinMode(motorOut, OUTPUT);
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
    digitalWrite(o1, HIGH);
    digitalWrite(o2, LOW);
    Serial.println("ON");
    analogWrite(motorOut, 50); //pwm funciton to control the speed
  } 
  delay(200);
}
