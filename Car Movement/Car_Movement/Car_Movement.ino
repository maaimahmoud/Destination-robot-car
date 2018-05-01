//--------------------------------------

///////////Car Variables////////////

const int leftForward = 9;
const int leftBackward = 10;
const int rightForward = 5;
const int rightBackward = 6;

const float carRadius = 7.65;
const float wheelRadius = 3.15;

const int cellSize = 5000;

int carSpeed = 255;

/// Motion Control///
const float r=3;
const float R=7.5;

float Voltr;
float Voltl;

float Vcar;
float Omegacar = 0;

const float k1 = 1;
const float k3 = 1;

const unsigned int deltaTime = 250;


//--------------------------------------

/////////Ultrasonic Variables//////////
const int trigPin = 7;
const int echoPin = 8;

long duration;
int distance;

//--------------------------------------

///////Rotary encoder Variables///////

//Left Rotary encoder
const int RotaryLeft=13;

int angleLeft=0;
bool prevStateLeft=0;
bool currentStateLeft=0;

//Right Rotary encoder
const int RotaryRight=12;

int angleRight=0;
bool prevStateRight=0;
bool currentStateRight=0;

//--------------------------------------

void ControlMotion()
{

  analogWrite(leftForward,carSpeed);
  analogWrite(rightForward,carSpeed);  
  analogWrite(leftBackward,0);
  analogWrite(rightBackward,0);  

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
  
    float Vrmax = radians(angleRight)/currentTime;
    float Vlmax = radians(angleLeft)/currentTime;
  
    float Vr=Vrmax;
    float Vl=Vlmax;

    Serial.println(Vrmax);
    Serial.println(Vlmax);

    if (Vr > Vl)
      Vcar = Vl;
    else
      Vcar = Vr;

    Vcar *= r;

    Serial.println(Vcar);
      
    Dr = radians(angleRight) * r;
    Dl = radians(angleLeft) * r;

    float theta = 0;
    float x = 0;
    float y = 0;

    float thetaCar = 0;
    float xCar = 0;
    float yCar = 0;
    
  while(((Dr + Dl) / 2 ) <= cellSize)
  {
    float omega = r*(Vr-Vl)/(2*R);
    float v = r*(Vr+Vl)/2;

    Serial.println(omega);
    Serial.println(v);
    
    // current position of the car
    theta = omega * currentTime;
    x = cos(theta) * v * currentTime;
    y = sin(theta) * v * currentTime;

    Serial.println(theta);
    Serial.println(x);
    Serial.println(y);

    //desired position of the car
    thetaCar = Omegacar * currentTime;
    xCar = Vcar * cos(thetaCar) * currentTime;
    yCar = Vcar * sin(thetaCar) * currentTime;

    Serial.println(thetaCar);
    Serial.println(xCar);
    Serial.println(yCar);

    //Errors happened
    float e1 = (xCar - x) * cos(theta) + (yCar - y) * sin(theta);
    float e2 = (xCar - x) * sin(theta) * -1.0 + (yCar - y) * cos(theta);
    float e3 = thetaCar - theta;

    Serial.println(e1);
    Serial.println(e2);
    Serial.println(e3);
    

    //set v and omega to the car
    v = Vcar * cos(e3) + k1 * e1;
    if (e3 != 0)
      omega = Omegacar + k3 * e3;
    else
      omega = Omegacar;

    Serial.println(v);
    Serial.println(omega);

    //get new vr and vl
    Vr = v / r + R / r * omega;
    Vl = v / r - R / r * omega;

    Serial.println(Vr);
    Serial.println(Vl);
   

    //get volt of right and left
    Voltr = 255 / Vrmax * Vr;
    Voltl = 255 / Vlmax * Vl;

    Serial.println(Voltr);
    Serial.println(Voltl);

     delay(3000);

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

  ControlMotion();
  delay(2000);
  
}
