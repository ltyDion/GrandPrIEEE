#define in1 7
#define in2 8
void setup() {
  // put your setup code here, to run once:
  pinMode(A0, OUTPUT);
  pinMode(in1, OUTPUT);
  pinMode(in2, OUTPUT);

  digitalWrite(in1, LOW);
  digitalWrite(in2, HIGH);
}

void loop() {
  // put your main code here, to run repeatedly:
  
  analogWrite(A0, 127);
  delay(200);
}
