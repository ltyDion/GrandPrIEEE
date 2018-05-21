
  // THE PINS OF THE LINE SCAN CAMERA TSL-1401DB.
 #include <Servo.h>
 #define CLK   2   //camera clock input is connected to pin 2 of arduino. 
 #define SI    3   //camera serial input is connected to pin 3 of arduino.
 #define A0  A0  // the analog output of the camera is connected to the A0 pin of the arduino, and also connected to oscilloscope.
 
int camera[128];             //FOR STORE THE DIFFENTS VALUES OF EACH PIXEL.
int median[3];               // FOR finding the median
int gradientF2[128];           // for gradient filtering
int cameraF1[128];           // for first filtering
double dt = 0;
double currentTime = 0;
int Error = 0;
double dError = 0;
double iError = 0;
double output = 0;
int turnAngle = 0;
double prevError = 0;
double lastTime = 0;
double Kd = 0;
double Kp = 5;
double Ki = 0;
int middle = 0;
double servoMiddle = 90;
boolean cross = false;
Servo myservo;

    
void setup() 
{
  pinMode(12, OUTPUT);      // 12 is the motor
  pinMode(7, OUTPUT);
  //pinMode(8, OUTPUT);

  digitalWrite(7, LOW);
  //digitalWrite(8, HIGH);
  myservo.attach(10);  // attaches the servo on pin 9 to the servo object
  pinMode(CLK,OUTPUT);
  pinMode(SI,OUTPUT);
  pinMode(A0,INPUT);        //THE ANALOG OUTPUT OF THE CAMERA is connected to the arduino board.
  Serial.begin(9600);         // IF YOU WANT TO SEE THE ANALOG OPPUTS VALUES OF THE CAMERA OF EACH PIXEL

// Start the first dimension
  digitalWrite(SI, HIGH);
  ClockPulse(); 
  digitalWrite(SI, LOW);

  Serial.begin(9600);
  myservo.write(90); 

}
/************************************************************************/

 void loop() 
{      

  //start the motor
  analogWrite(12, 50);

  

  
  // Clean the sensor register from debris
  for(int i = 0; i < 128; i++){
    ClockPulse(); 
  }

  // Exposition 3ms
  //delay(3);

  digitalWrite(CLK,LOW);
  digitalWrite(SI,HIGH);
  digitalWrite(CLK,HIGH);
  digitalWrite(SI,LOW);
  digitalWrite(CLK,LOW);

  for (int j = 0; j < 128 ; j++){
    digitalWrite(CLK, HIGH);
    digitalWrite(CLK, LOW);  
  }

  delayMicroseconds(20000);
  
  // Read 128 pixels
  digitalWrite(SI, HIGH);
  digitalWrite(CLK, HIGH);
  digitalWrite(SI, LOW);
  digitalWrite(CLK, LOW);
  for(int i=0; i < 128; i++){
    delayMicroseconds(20); // pause for fixing the value to the ADC
    camera[i] = analogRead(A0);
    //Serial.println(camera[i]);
    digitalWrite(CLK, HIGH);
    digitalWrite(CLK, LOW);
  }

    //Serial.println("Camera value");
    //for (int i = 0; i < 128; i++){
    //  Serial.print(camera[0] + " ");  
    //}
    for (int i = 0; i < 128; i++){
    Serial.println(cameraF1[i]); 
    } 

  //filter
  Filter();

  gradientF();
  
  int maxIndex = 0;
  int maxValue = 0;
  // find middle
  // find max
  for(int i = 0; i < 128; i++){
     if(gradientF2[i] > maxValue){
        maxValue = gradientF2[i];
        maxIndex = i;
     }
  }

  //find min
  int minIndex = 0;
  int minValue = 0;
   for(int k = 0; k < 128; k++){
     if(gradientF2[k] < minValue){
        minValue = gradientF2[k];
        minIndex = k;
     }
  }

  middle = (maxIndex + minIndex) /2;

  //time at the beginning of your loop
  currentTime = millis();

  //find the difference in time
  dt = double(currentTime - lastTime);

  //PID Control Code starts here

  //After filtering, you should have some sort of output "middle", containing the current 
  //center of the white line 63 is where you want the middle to be, since the camera output
  //is in the form of an array from indexes 0-127

  //calculate the error
  //middle = maxIndex;
  //Serial.println("middle: " + middle);
  Error = middle - 63;
  dError = (Error - prevError)/dt;
  iError = (Error + prevError) * dt;
  output = Kp*Error + Kd * dError + Ki * iError;
  //After becoming familiar wth the Arduiuno Servo Library, figure out what "servoMiddle" 
  //should be
  turnAngle = output + servoMiddle;

  // when crossroad
  cross = false;
  crossroad();
  if(cross == true){
     turnAngle = 90;
  }
  
  //Serial.println("turnAngle: " + turnAngle);
  
  //delay(50);
  myservo.write(turnAngle);

  prevError = Error;
  lastTime = currentTime;
  
 // timming();              // if this function is activated, dont active the ADC_READ_CAMERA() and impre() functions. use this function when the
                          // A0 pin of the camera is connected to the oscilloscope.
  //ADC_READ_CAMERA();
  // impre();             // enable this function if you want the 128 output analog values in the serial monitor of arduino, but
                          // if you use this function you can not see the analog output signal in the osiclloscope.
}


// Function that generates a sync
void ClockPulse(){
  delayMicroseconds(1);
  digitalWrite(CLK, HIGH);
  digitalWrite(CLK, LOW);
}


// Function that filter the array
void Filter(){
  cameraF1[0] = camera[0];
  cameraF1[127] = camera[127];
  for(int i = 1; i < 127; i++){
     int x1 = camera[i-1];
     int x2 = camera[i];
     int x3 = camera[i+1]; 
     int maxX = 0;
     int medianX = 0;
     
     // find the median
     int maxXT = max(x1, x2);
     maxX = max(maxXT, x3);
     if(x1 == maxX){
        medianX = max(x2,x3);
     }
     else if(x2 = maxX){
        medianX = max(x1,x3);
     }
     else{
      medianX = max(x1, x2);
     }

     cameraF1[i] = medianX;
     
  }
     
}

//Function that filter the gradient
// highest positive is left and highest negative is right
void gradientF(){
   for(int i = 0; i < 127; i++){
      gradientF2[i] = cameraF1[i+1] - camera[i];
   }
   gradientF2[127] = 0;
}

// determine if there is a crossroad
void crossroad(){
  int count = 0;
  for(int i = 0; i < 128; i++){
     if(camera[i] > 300){
        count ++;
     }
  }
  if(count > 100){
     cross = true;
  }
   
  
}





