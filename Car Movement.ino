//--------------------------------------

///////////Car Variables////////////

const int leftForward = 5;
const int leftBackward = 6;
const int rightForward = 9;
const int rightBackward = 10;

const float carRadius = 7.5;
const float wheelRadius = 3;

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
const int RotaryLeft=12;

int angleLeft=0;
bool prevStateLeft=0;
bool currentStateLeft=0;

//Right Rotary encoder
const int RotaryRight=13;

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

  unsigned long begginningTime=millis();
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
  
    float Vrmax = (r*angleRight)/currentTime;
    float Vlmax = (r*angleLeft)/currentTime;
  
    float Vr=Vrmax;
    float Vl=Vlmax;

    if (Vr > Vl)
      Vcar = Vl;
    else
      Vcar = Vr;
      
    Dr = angleRight * 3.14 / 180 * r;
    Dl = angleLeft * 3.14 / 180 * r;
    
  while(((Dr + Dl) / 2 ) <= cellSize)
  {
    float omega = r*(Vr-Vl)/2*R;
    float v = r*(Vr+Vl)/2;
    
    // current position of the car
    float theta = omega * currentTime;
    float x = cos(theta) * v;
    float y = sin(theta) * v;

    //desired position of the car
    float thetaCar = Omegacar * currentTime;
    float xCar = Vcar * cos(thetaCar);
    float yCar = Vcar * sin(thetaCar);

    //Errors happened
    float e1 = (xCar - x) * cos(theta) + (yCar - y) * sin(theta);
    float e2 = (xCar - x) * sin(theta) * -1 + (yCar - y) * cos(theta);
    float e3 = thetaCar - theta;

    //set v and omega to the car
    v = Vcar * cos(e3) + k1 * e1;
    omega = Omegacar + k3 * e3 + Vcar * e2 * sin(e3) / e3;

    //get new vr and vl
    Vr = v / r + R / r * omega;
    Vl = v / r - R / r * omega;

    //get volt of right and left
    Voltr = 255 / Vrmax * Vr;
    Voltl = 255 / Vlmax * Vl;

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
  
    Vr = (r*(angleRight-oldAngleRight))/currentTime;
    Vl = (r*(angleLeft-oldAngleLeft))/currentTime;

    Dr = angleRight * 3.14 / 180 * r;
    Dl = angleLeft * 3.14 / 180 * r;
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
