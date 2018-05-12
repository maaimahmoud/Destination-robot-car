//--------------------------------------
//#include <PID_v1.h>
///////////Car Variables////////////

const int leftForward = 9;
const int leftBackward = 10;
const int rightForward = 5;
const int rightBackward = 6;

const float carRadius = 7.65;
const float wheelRadius = 3.15;

const int cellSize = 400;

int carSpeed = 255;

/// Motion Control///
const float r=3.15;
const float R=7.65;

float Voltr;
float Voltl;

float Vcar;
float Omegacar = 0;

const float k1 = 2;
const float k3 = 1;
const unsigned int deltaTime = 350;


//--------------------------------------

/////////Ultrasonic Variables//////////
const int trigPin = 7;
const int echoPin = 8;

long duration;
int distance;

//--------------------------------------

///////Rotary encoder Variables///////

//Left Rotary encoder
const int RotaryLeft=12;

int angleLeft=0;
bool prevStateLeft=0;
bool currentStateLeft=0;

//Right Rotary encoder
const int RotaryRight=13;

int angleRight=0;
bool prevStateRight=0;
bool currentStateRight=0;

////////////////////////////////
float Kp = 3;
float Ki = 1;
float Kd = 2;

//--------------------------------------

void PID_control()
{

  analogWrite(leftForward,carSpeed);
  analogWrite(rightForward,carSpeed);  
  analogWrite(leftBackward,0);
  analogWrite(rightBackward,0);  

  float old_error_v = 0;
  float E_v = 0;

  float old_error_omega = 0;
  float E_omega = 0;

  float Dr = 0;
  float Dl = 0;  

  unsigned long begginningTime = millis();
    angleLeft = 0;
    angleRight = 0;

    float currentTime = millis() - begginningTime;
    while( currentTime < deltaTime)
    {
       //Angles for both motors
        //for Left Motor
        ReadLeftRotary();
        //for Right Motor
        ReadRightRotary();
        
        currentTime = millis() - begginningTime;
    }
    currentTime /= 1000;
  
    float VrRef = radians(angleRight)/currentTime;
    float VlRef = radians(angleLeft)/currentTime;

    int voltrRef = carSpeed;
    int voltlRef = carSpeed;
  
    float Vr=VrRef;
    float Vl=VlRef;

    Serial.println(Vr);
    Serial.println(Vl);

    //Vcar = (Vr + Vl) / 2.0;

    Vcar = 28;

    //Serial.println(Vcar);
      
    Dr = radians(angleRight) * r;
    Dl = radians(angleLeft) * r;

    
  while(((Dr + Dl) / 2 ) <= cellSize)
  {
    float omega = r*(Vr-Vl)/(2*R);
    float v = r*(Vr+Vl)/2;

    Serial.println(v);
    Serial.println(omega);

    float error_v = abs(Vcar - v);
    float error_omega = abs(Omegacar - omega);

    Serial.println(error_v);
    Serial.println(error_omega);

    float e_dot_v = error_v - old_error_v;
    float e_dot_omega = error_omega - old_error_omega;

    Serial.println(e_dot_v);
    Serial.println(error_omega);

    E_v += error_v;
    E_omega += error_omega;

    v = v - (0.5 * error_v + 0.5 * E_v + 0.5 * e_dot_v);
    omega = omega - (1 * error_omega + 0.5 * E_omega + 1 * e_dot_omega);

    old_error_v = error_v;
    old_error_omega = error_omega;

    Serial.println(v);
    Serial.println(omega);

    //get new vr and vl
    Vr = v / r + R / r * omega;
    Vl = v / r - R / r * omega;

    Serial.println(Vr);
    Serial.println(Vl);

    Voltr = (0.05692461 * Vr + 0.18672734) * 255;
    Voltl = (0.00200427 * Vl * Vl + 0.01608561 * Vl + 0.20535106) * 255;

    //get volt of right and left
    //Voltr = 255 / 17 * Vr;
    //Voltl = 255 / 17 * Vl;

    Serial.println(Voltr);
    Serial.println(Voltl);
 

    analogWrite(leftForward,Voltl);
    analogWrite(rightForward,Voltr);

    begginningTime=millis();
    int oldAngleLeft = angleLeft;
    int oldAngleRight = angleRight;

    currentTime = millis() - begginningTime;
    while( currentTime < deltaTime)
    {
       //Angles for both motors
        //for Left Motor
        ReadLeftRotary();
        //for Right Motor
        ReadRightRotary();
        
        currentTime = millis() - begginningTime;
    }
    currentTime /= 1000;
  
    Vr = radians(angleRight-oldAngleRight)/currentTime;
    Vl = radians(angleLeft-oldAngleLeft)/currentTime;

    Serial.println(Vr);
    Serial.println(Vl);

    delay(3000);

    Dr = radians(angleRight) * r;
    Dl = radians(angleLeft ) * r;
  }
  
    analogWrite(leftForward,0);
    analogWrite(rightForward,0);
}


void ReadUltrasonic() {
  
  digitalWrite(trigPin, LOW); // Clears the trigPin
  delayMicroseconds(2);
  // Sets the trigPin on HIGH state for 10 micro seconds
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);
  // Reads the echoPin, returns the sound wave travel time in microseconds
  duration = pulseIn(echoPin, HIGH);
  distance= duration*0.034/2; // Calculating the distance
}

void ReadLeftRotary()
{
      prevStateLeft=currentStateLeft;
    currentStateLeft=digitalRead(RotaryLeft);
  if (currentStateLeft != prevStateLeft)
     angleLeft=(angleLeft+9);
}

void ReadRightRotary()
{
      prevStateRight=currentStateRight;
  currentStateRight=digitalRead(RotaryRight);
   if (currentStateRight != prevStateRight)
     angleRight=(angleRight+9);
}

void RunForward(float distance)
{
  angleRight=0;
  angleLeft=0;
  float degree = distance/wheelRadius *180.0 / 3.14;
  //Serial.print(degree);
  //delay(2000);
  while (angleRight <= degree || angleLeft <= degree)
  {

      //Angles for both motors
      //for Left Motor
      ReadLeftRotary();
      //for Right Motor
      ReadRightRotary();

    //delay(3000);
      
      if (angleRight == angleLeft)
      {
         analogWrite(leftForward,carSpeed);
         analogWrite(rightForward,carSpeed);  
         //Serial.println("1111111111111111111111");
      }
      else if (angleRight>angleLeft)
      { 
         analogWrite(leftForward,carSpeed);
         analogWrite(rightForward,0);  
         //Serial.println("22222222222222");
         //delay(2000);
      }
      else if (angleLeft>angleRight)
      {
         analogWrite(leftForward,0);
         analogWrite(rightForward,carSpeed);
         //Serial.println("333"); 
         //delay(2000);
      }
        
      analogWrite(rightBackward,0);
      analogWrite(leftBackward,0);

    //Serial.println(angleRight);
    //Serial.println(angleLeft);
  }
   analogWrite(leftForward,0);
   analogWrite(rightForward,0);   
}

void RunBackward(float distance)
{
  angleRight=0;
  angleLeft=0;
  float degree = distance/wheelRadius *180.0 / 3.14;
  while (angleRight <= degree || angleLeft <= degree )
  {
    if (angleRight == angleLeft)
      {
         analogWrite(leftBackward,carSpeed);
         analogWrite(rightBackward,carSpeed);  
      }
      else if (angleRight>angleLeft)
      { 
         analogWrite(leftBackward,carSpeed);
         analogWrite(rightBackward,0);  
      }
      else if (angleLeft>angleRight)
      {
         analogWrite(leftBackward,0);
         analogWrite(rightBackward,carSpeed);  
      }
        
      analogWrite(rightForward,0);
      analogWrite(leftForward,0);

      
    //Angles for both motors
    //for Left Motor
     ReadLeftRotary();
    //for Right Motor
    ReadRightRotary();
  }
  analogWrite(leftBackward,0);
   analogWrite(rightBackward,0); 
}


void RunLeftward(int degree)
{
  angleRight=0;
  angleLeft=0;
  degree=(carRadius*degree)/wheelRadius;
  degree -= degree%9;
  while (angleRight <= degree || angleLeft <= degree )
  {
    if (angleRight == angleLeft)
    {
       analogWrite(leftForward,carSpeed);
       analogWrite(rightBackward,carSpeed);  
    }
    else if (angleRight>angleLeft)
    { 
       analogWrite(leftForward,carSpeed);
       analogWrite(rightBackward,0);  
    }
    else if (angleLeft>angleRight)
    {
       analogWrite(leftForward,0);
       analogWrite(rightBackward,carSpeed);  
    }
    
    analogWrite(leftBackward,0);  
    analogWrite(rightForward,0);


    //Angles for both motors
    //for Left Motor
     ReadLeftRotary();
    //for Right Motor
    ReadRightRotary();
  }
     analogWrite(leftForward,0);
     analogWrite(rightBackward,0);  
}

void RunRightward(int degree)
{
  angleRight=0;
  angleLeft=0;
  degree=(carRadius*degree)/wheelRadius;
  degree -= degree%9;
  while (angleRight <= degree || angleLeft <= degree )
  {
    
    //Angles for both motors
    //for Left Motor
     ReadLeftRotary();
    //for Right Motor
    ReadRightRotary();
    
    if (angleRight == angleLeft)
    {
       analogWrite(leftBackward,carSpeed);
       analogWrite(rightForward,carSpeed);  
    }
    else if (angleRight>angleLeft)
    { 
       analogWrite(leftBackward,carSpeed);
       analogWrite(rightForward,0);  
    }
    else if (angleLeft>angleRight)
    {
       analogWrite(leftBackward,0);
       analogWrite(rightForward,carSpeed);  
    }
    
    analogWrite(rightBackward,0);  
    analogWrite(leftForward,0);

  }
      analogWrite(leftBackward,0);
       analogWrite(rightForward,0);  
    
}

void setup() 
{
  //Car Pins
  pinMode(leftForward, OUTPUT);
  pinMode(leftBackward, OUTPUT);
  pinMode(rightForward, OUTPUT);
  pinMode(rightBackward, OUTPUT);

  //Ultrasonic Pins
  pinMode(trigPin, OUTPUT); // Sets the trigPin as an Output
  pinMode(echoPin, INPUT); // Sets the echoPin as an Input

  //Rotary encoder Pins
  pinMode(RotaryLeft,INPUT); //Set pins for left rotary encoder as an input to get angle
  pinMode(RotaryRight,INPUT); //Set pins for right rotary encoder as an input to get angle

  Serial.begin(9600);

  //RunForward(100);

}

void loop() {
  //ReadUltrasonic();
  //delay(5000);
  /*RunForward(cellSize);
  RunRightward(90);
  RunForward(cellSize);
  RunLeftward(90);
  RunForward(cellSize);
  RunLeftward(180);
  RunForward(2*cellSize);*/

  /*
  RunRightward(90);
  //Angles for both motors
    //for Left Motor
     ReadLeftRotary();
    //for Right Motor
    ReadRightRotary();
    Serial.println(angleRight);
    Serial.println(angleLeft);
  delay(1000);
  RunLeftward(90);
  */
  /*
  RunForward(cellSize);
  delay(2000);
  RunBackward(cellSize);
   delay(2000);
   
  
  
  //Angles for both motors
    //for Left Motor
     ReadLeftRotary();
    //for Right Motor
    ReadRightRotary();
    Serial.println(angleRight);
    Serial.println(angleLeft);
  delay(1000);
  */

  PID_control();
  delay(2000);
  
}
