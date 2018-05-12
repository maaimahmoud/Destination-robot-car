const int leftForward = 9;
const int leftBackward = 10;
const int rightForward = 5;
const int rightBackward = 6;

const int IRright = A0;
const int IRleft = A1;
const int IRcenter = A2;

// defines variables
long duration;
int distance;

int value = 255;

int carSpeed = 200;



const float carRadius = 7.65;
const float wheelRadius = 3.15;

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
  //run forward
  analogWrite(rightForward,190);
  analogWrite(rightBackward,0);
  analogWrite(leftForward,170);
  analogWrite(leftBackward,0);
  delay(200);
  int rightSensor = analogRead(IRright);
  int leftSensor = analogRead(IRleft);
  int centerSensor = analogRead(IRcenter);
  while( rightSensor < 200 ||  leftSensor < 200)
  {
    /*if (centerSensor < 200 )
    {
      if ( leftSensor > 200)
        {
             analogWrite(rightForward,190);
             analogWrite(leftForward,50);
        }
      else
        {
             analogWrite(rightForward,50);
             analogWrite(leftForward,170);
        }
    }
    else 
    {
      if ( leftSensor > 200)
        {
             analogWrite(rightForward,170);
             analogWrite(leftForward,90);
        }
      else
        {
             analogWrite(rightForward,90);
             analogWrite(leftForward,150);
        }
    }*/
      if (leftSensor > 200)
        {
            analogWrite(rightForward,170);
            analogWrite(leftForward,100);
        }
        else if (rightSensor > 200)
        {
            analogWrite(rightForward,100);
            analogWrite(leftForward,170);
        }
        else
        {
            analogWrite(rightForward,170);
            analogWrite(leftForward,150);
        }
        
           rightSensor = analogRead(IRright);
           leftSensor = analogRead(IRleft);
           centerSensor = analogRead(IRcenter);
  
  }
  analogWrite(leftForward,0);
  analogWrite(leftBackward,0);
  analogWrite(rightForward,0);
  analogWrite(rightBackward,0);
}

void RunBackward()
{
  //run forward
  analogWrite(rightForward,0);
  analogWrite(rightBackward,190);
  analogWrite(leftForward,0);
  analogWrite(leftBackward,170);
  delay(500);
  int rightSensor = analogRead(IRright);
  int leftSensor = analogRead(IRleft);
  int centerSensor = analogRead(IRcenter);
  while( rightSensor < 200 ||  leftSensor < 200  )
  {
      //Serial.println("omar");
        if (leftSensor > 200)
        {
            analogWrite(rightBackward,190);
            analogWrite(leftBackward,0);
        }
        else if (rightSensor > 200)
        {
            analogWrite(rightBackward,0);
            analogWrite(leftBackward,170);
        }
        else
        {
            analogWrite(rightBackward,190);
            analogWrite(leftBackward,170);
        }
        
           rightSensor = analogRead(IRright);
           leftSensor = analogRead(IRleft);
           centerSensor = analogRead(IRcenter);
  
  }
  analogWrite(leftForward,0);
  analogWrite(leftBackward,0);
  analogWrite(rightForward,0);
  analogWrite(rightBackward,0);
}

void RunLeftward()
{
  //run forward
  analogWrite(rightForward,180);
  analogWrite(rightBackward,0);
  analogWrite(leftForward,0);
  analogWrite(leftBackward,130);
  delay(500);
  int rightSensor = analogRead(IRright);
  int leftSensor = analogRead(IRleft);
  int centerSensor = analogRead(IRcenter);
  bool forward = true;
  
  int degree = 85;
  angleLeft = 0;
  angleRight = 0;
  while( (rightSensor < 200 ||  leftSensor < 200 || centerSensor < 200) )
  {
      if (forward)
      {
        analogWrite(rightForward, 180);
        analogWrite(leftBackward, 130);
      }
      else
      {
        analogWrite(rightForward, 120);
        analogWrite(leftBackward, 170);
        delay(50);
      }
        
       rightSensor = analogRead(IRright);
       leftSensor = analogRead(IRleft);
       centerSensor = analogRead(IRcenter);

       //Angles for both motors
        //for Left Motor
         ReadLeftRotary();
        //for Right Motor
        ReadRightRotary();

       if (centerSensor <200)
          forward = false;
    
  
  }

  while (angleRight <= degree || angleLeft <= degree )
  {
    
    if (angleRight == angleLeft)
    {
       analogWrite(rightForward,180);
       analogWrite(leftBackward,150);  
    }
    else if (angleRight>angleLeft)
    { 
      analogWrite(rightForward,0);
       analogWrite(leftBackward,170);
    }
    else if (angleLeft>angleRight)
    {
       analogWrite(rightForward,180);
       analogWrite(leftBackward,0);  
    }
    
    analogWrite(leftForward,0);  
    analogWrite(rightBackward,0);


    //Angles for both motors
    //for Left Motor
     ReadLeftRotary();
    //for Right Motor
    ReadRightRotary();

  }
  analogWrite(leftForward,0);
  analogWrite(leftBackward,0);
  analogWrite(rightForward,0);
  analogWrite(rightBackward,0);
}


void RunRightward()
{
  //run forward
  analogWrite(rightForward,0);
  analogWrite(rightBackward,130);
  analogWrite(leftForward,170);
  analogWrite(leftBackward,0);
  delay(500);
  int rightSensor = analogRead(IRright);
  int leftSensor = analogRead(IRleft);
  int centerSensor = analogRead(IRcenter);
  bool forward = true;
  
  int degree = 75;
  angleLeft = 0;
  angleRight = 0;
  while( (rightSensor < 200 /*||  leftSensor < 200*/ || centerSensor < 200) )
  {
      if (forward)
      {
        analogWrite(leftForward, 170);
        analogWrite(rightBackward, 130);
      }
      else
      {
        analogWrite(leftForward, 110);
        analogWrite(rightBackward, 170);
        delay(50);
      }
        
       rightSensor = analogRead(IRright);
       leftSensor = analogRead(IRleft);
       centerSensor = analogRead(IRcenter);

       //Angles for both motors
        //for Left Motor
         ReadLeftRotary();
        //for Right Motor
        ReadRightRotary();

       if (centerSensor <200)
          forward = false;
    
  
  }

  while (angleRight <= degree && angleLeft <= degree )
  {
    
    if (angleRight == angleLeft)
    {
       analogWrite(leftForward,170);
       analogWrite(rightBackward,150);  
    }
    else if (angleRight>angleLeft)
    { 
       analogWrite(leftForward,150);
       analogWrite(rightBackward,0);  
    }
    else if (angleLeft>angleRight)
    {
       analogWrite(leftForward,0);
       analogWrite(rightBackward,150);
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
  analogWrite(leftBackward,0);
  analogWrite(rightForward,0);
  analogWrite(rightBackward,0);
}

void setup() {
  pinMode(leftForward, OUTPUT);
  pinMode(leftBackward, OUTPUT);
  pinMode(rightForward, OUTPUT);
  pinMode(rightBackward, OUTPUT);

  //Rotary encoder Pins
  pinMode(RotaryLeft,INPUT); //Set pins for left rotary encoder as an input to get angle
  pinMode(RotaryRight,INPUT); //Set pins for right rotary encoder as an input to get angle

  pinMode(IRright, INPUT);
  pinMode(IRleft, INPUT);
  pinMode(IRcenter, INPUT);

  Serial.begin(9600);
}

void loop() {  

  RunForward();
  delay(500);

//  RunForward();
//  delay(500);
//  
//  //RunRightward();
//  RunLeftward();
//  delay(500);
  
  /*int rightSensor = analogRead(IRright);
  int leftSensor = analogRead(IRleft);
  int centerSensor = analogRead(IRcenter);
     Serial.println(rightSensor);
  Serial.println(leftSensor);
  Serial.println(centerSensor);
  delay(1000);*/
  /*
  // run backward
  analogWrite(leftForward,0);
  analogWrite(leftBackward,255);
  analogWrite(rightForward,0);
  analogWrite(rightBackward,255);
  */
  

}
