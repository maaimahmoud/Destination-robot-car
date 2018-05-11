
#include <Servo.h>
#define SIZE 15
#define maped_grid_size 10
#define xpos 1
#define ypos 2
int grid[SIZE][SIZE];
int servo_angle=0;
Servo myservo;
// setting name of pins in order to use it easily in code//
const int trigPin =7;
const int echoPin = 8;
const int trigPin2 =3;
const int echoPin2 = 4;
const int servopin = 11;
void setup() {
// decleration of pins type //
pinMode(trigPin, OUTPUT); // Sets the trigPin as an Output
pinMode(echoPin, INPUT); // Sets the echoPin as an Input
pinMode(trigPin2, OUTPUT); // Sets the trigPin as an Output
pinMode(echoPin2, INPUT); // Sets the echoPin as an Input
myservo.attach(11);
// initillize grid as if there doesn't exist any obstacles 
Inilizegrid();
 Serial.begin(9600);
}
/*******************************************************************************************************************************/
void Updategrid(int x,int y)
{
  int mx=0;
  int my=0;
// mapped read distance to required valuce according to mapped_grid_size (each cell indicate ? distance on real)
  mx = (int(x / maped_grid_size))+xpos;
  my = (int(y / maped_grid_size))+ypos;
  if ((mx >=0) && (mx < SIZE) && (my >=0) && (my < SIZE))
    grid[mx][ my] = 1; // to indicate that there exists obstacle at this place
}
/*******************************************************************************************************************************/
void Inilizegrid()
{
  for (int i = 0; i < SIZE; i++)
    for (int j = 0; j < SIZE; j++)
      grid[i][j] = 0; // to indicate that there doesn't exist any obstacle

}
/*******************************************************************************************************************************/
bool Ultrasonicread(float& d,int mode)
{
  // defines variables
  long duration;
  if (mode==1){
    digitalWrite(trigPin, LOW); // Clears the trigPin
    delayMicroseconds(2);
    // Sets the trigPin on HIGH state for 10 micro seconds
    digitalWrite(trigPin, HIGH);
    delayMicroseconds(10);
    digitalWrite(trigPin, LOW);
    // Reads the echoPin, returns the sound wave travel time in microseconds
    duration = pulseIn(echoPin, HIGH);
  }
  else if (mode==2){
    digitalWrite(trigPin2, LOW); // Clears the trigPin
    delayMicroseconds(2);
    // Sets the trigPin on HIGH state for 10 micro seconds
    digitalWrite(trigPin2, HIGH);
    delayMicroseconds(10);
    digitalWrite(trigPin2, LOW);
    // Reads the echoPin, returns the sound wave travel time in microseconds
    duration = pulseIn(echoPin2, HIGH);
  }
  d = duration*0.034 / 2; // Calculating the distance
  if ((d>0)&&(d<3000)) // check that read distance is right 
    return true;
  return false;
}
/*******************************************************************************************************************************/
void Conversion(float d,int a,int& x,int& y)
{
 x = d*sin(a*PI/180);
 y = d*cos(a*PI / 180);
}
/*******************************************************************************************************************************/
void setservoangle()
{
  // send required angle to servo //
  myservo.write(servo_angle);
}
/*******************************************************************************************************************************/



void loop() {
   servo_angle = 0;
  // loop for 360 degree to get region's grid 
while (servo_angle<=180)
{
  // note all of this functions works on only one ultrasonic 
float distance = 0.0;
int x=0;
int y=0;
  // change angle of servo in order to get data from ultrasonic
// 
  setservoangle();
   Serial.println(servo_angle);
  // get reading of ultrasonic and check if there exist obstacle or not +
 bool obstacle =Ultrasonicread(distance,1);
 Serial.println(distance);
  // if there exit obstacle convert it to be mapped and updated in grid
  if (obstacle)  {
    Conversion(distance,servo_angle,x,y);
  Updategrid(x,y); 
 }
 obstacle =Ultrasonicread(distance,2);
  Serial.println(distance);
  // if there exit obstacle convert it to be mapped and updated in grid
  if (obstacle)
   {
    Conversion(distance,(servo_angle+180),x,y);
    Updategrid(x,y); 
  }
 // increment servo angle to check following places 
  servo_angle += 30;
  delay(100);
//  printing grid 
 for (int i = 0; i < SIZE; i++)
    for (int j = 0; j < SIZE; j++)
     Serial.println (grid[i][j]);
  delay(1000);

}
}
