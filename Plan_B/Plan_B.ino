//--------------------------------------
///////////Car Variables////////////

const int leftForward = 9;
const int leftBackward = 10;
const int rightForward = 5;
const int rightBackward = 6;

const int cellSize = 400;

int carSpeed = 255;

const float wheelRadius=3.15;
const float carRadius=7.65;

/// Motion Control///
float Vrmax;
float Vlmax;

float Voltr;
float Voltl;

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
void intialize()
{ 
  //Calculate both Vrmax,Vlmax
    unsigned long begginningTime = millis();
    angleLeft = 0;
    angleRight = 0;

    float currentTime = millis() - begginningTime;
    while( currentTime < deltaTime)
    {
       //Angles for both motors
        ReadLeftRotary(); //for Left Motor
        ReadRightRotary(); //for Right Motor
        
        currentTime = millis() - begginningTime;
    }
    currentTime /= 1000;
  
    Vrmax = radians(angleRight)/currentTime;
    Vlmax = radians(angleLeft)/currentTime;
}

void ControlMotion()
{
    unsigned long begginningTime = millis();
    float currentTime = millis() - begginningTime;
    while( currentTime < deltaTime)
    {
       //Angles for both motors
        ReadLeftRotary(); //for Left Motor
        ReadRightRotary(); //for Right Motor
        
        currentTime = millis() - begginningTime;
    }
    currentTime /= 1000;

    float Vr = radians(angleRight)/currentTime;
    float Vl = radians(angleLeft)/currentTime;

    float Vcar;
    
    if (Vr > Vl)
      Vcar = Vl;
    else
      Vcar = Vr;

    Voltr = 255 / Vrmax * Vcar;
    Voltl = 255 / Vlmax * Vcar;
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

void RunForward()
{
  analogWrite(leftForward,carSpeed);
  analogWrite(rightForward,carSpeed);
  analogWrite(rightBackward,0);
  analogWrite(leftBackward,0);
  
  intialize();
  float degree = cellSize /wheelRadius * 180.0 / 3.14;
  //degree -= degree % 9;

  
  while (angleRight <= degree || angleLeft <= degree)
  {
    ControlMotion();

     analogWrite(leftForward,Voltl);
     analogWrite(rightForward,Voltr); 
     
    //Angles for both motors
      ReadLeftRotary(); //for Left Motor
      ReadRightRotary(); //for Right Motor
      
  }
   analogWrite(leftForward,0);
   analogWrite(rightForward,0);   
}

void RunBackward()
{
  analogWrite(leftBackward,carSpeed);
  analogWrite(rightBackward,carSpeed);  
  analogWrite(rightForward,0);
  analogWrite(leftForward,0);
  
  intialize();
  float degree = cellSize /wheelRadius *180.0 / 3.14;
  //degree -= degree % 9;

  while (angleRight <= degree || angleLeft <= degree )
  {
      ControlMotion();
      
      analogWrite(leftBackward,Voltl);
      analogWrite(rightBackward,Voltr);  

    //Angles for both motors
      ReadLeftRotary(); //for Left Motor
      ReadRightRotary(); //for Right Motor
  }
  
  analogWrite(leftBackward,0);
  analogWrite(rightBackward,0); 
}


void RunLeftward(int degree)
{
  analogWrite(leftForward,carSpeed);
  analogWrite(rightBackward,carSpeed); 
  analogWrite(leftBackward,0);  
  analogWrite(rightForward,0);
  
  intialize();
  degree=(carRadius*degree)/wheelRadius;
  degree -= degree%9;
  
  while (angleRight <= degree || angleLeft <= degree )
  {
    ControlMotion();
    
     analogWrite(leftForward,Voltl);
     analogWrite(rightBackward,Voltr);  
  
    //Angles for both motors
      ReadLeftRotary(); //for Left Motor
      ReadRightRotary(); //for Right Motor
  }
     analogWrite(leftForward,0);
     analogWrite(rightBackward,0);  
}

void RunRightward(int degree)
{
  intialize();
  
  degree=(carRadius*degree)/wheelRadius;
  degree -= degree%9;

   analogWrite(rightBackward,0);  
   analogWrite(leftForward,0);
  while (angleRight <= degree || angleLeft <= degree )
  {

      ControlMotion();
      
      //Angles for both motors
        ReadLeftRotary(); //for Left Motor
        ReadRightRotary(); //for Right Motor
    
       analogWrite(leftBackward,Voltl);
       analogWrite(rightForward,Voltr);   

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

  RunForward();
  delay(500);
  RunBackward();
  delay(1000);
  
}
