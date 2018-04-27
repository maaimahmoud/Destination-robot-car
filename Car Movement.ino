//--------------------------------------

///////////Car Variables////////////

const int leftForward = 5;
const int leftBackward = 6;
const int rightForward = 9;
const int rightBackward = 10;
const float carRadius = 6.5;
const float wheelRadius = 3;

int carSpeed = 255;

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

void readLeftRotary()
{
      prevStateLeft=currentStateLeft;
    currentStateLeft=digitalRead(RotaryLeft);
  if (currentStateLeft != prevStateLeft)
     angleLeft=(angleLeft+9)%360;
}

void readRightRotary()
{
      prevStateRight=currentStateRight;
  currentStateRight=digitalRead(RotaryRight);
   if (currentStateRight != prevStateRight)
     angleRight=(angleRight+9)%360;
}

void RunForward()
{
/*
  if (angleLeft<=angleRight)
     analogWrite(leftForward,carSpeed);
  else
     analogWrite(leftForward,0);
  analogWrite(leftBackward,0);
 if (angleRight<=angleLeft) 
     analogWrite(rightForward,carSpeed);
  else 
     analogWrite(rightForward,0);
  analogWrite(rightBackward,0);
*/
  if (angleRight == angleLeft)
  {
     analogWrite(leftForward,carSpeed);
     analogWrite(rightForward,carSpeed);  
  }
  else if (angleRight>angleLeft)
  { 
     analogWrite(leftForward,carSpeed);
     analogWrite(rightForward,0);  
  }
  else if (angleLeft>angleRight)
  {
     analogWrite(leftForward,0);
     analogWrite(rightForward,carSpeed);  
  }
    
  analogWrite(rightBackward,0);
  analogWrite(leftBackward,0);
    /*
   Serial.println(angleRight);
   Serial.println(angleLeft);
   */
}

void RunBackward()
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
}


void RunLeftward()
{
  
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
  // put your main code here, to run repeatedly:
  
  //ReadUltrasonic();

    //Angles for both motors
    //for Left Motor
     readLeftRotary();
    //for Right Motor
    readRightRotary();
    
    
   RunForward();

   //RunBackward();

}
