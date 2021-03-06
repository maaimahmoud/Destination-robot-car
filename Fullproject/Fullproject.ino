#include <Servo.h>
#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>

#define ROW 15
#define COL 15

#define PAIRI 15
#define ShPAIRI 0
#define PAIRJ 240
#define ShPAIRJ 4

/* Description of the Grid-
16 bit for every cell
bit 16
1--> The cell is not blocked
0--> The cell is blocked    
bits 9 -15  g value
bits 5 -8 parent j 
bits 1 -4 parent i

h value will be calculated, f value is h + g
*/

#define UNBLOCKED     32768
#define ShUNBLOCKED   15
#define GVAl          32512
#define ShGVAl        8

/*******************************************************************************************************************************/
#define SIZE 15
#define maped_grid_size 20
uint16_t openList [SIZE*SIZE];
#define FVAl          65280
#define ShFVAl        8
uint32_t accessList [SIZE];
int servo_angle = 0;
// 4 means four direction (no cross), 8 means the eight deirections (include the cross)
#define AVILABLEMOVES 4
#define EFFORT 1
uint16_t grid[SIZE][SIZE];
Servo myservo;
// setting name of pins in order to use it easily in code//
const int trigPin = 2;
const int echoPin = 3;
const int trigPin2 = 2;
const int echoPin2 = 4;
const int servopin = 11;


const int leftForward = 9;
const int leftBackward = 10;
const int rightForward = 5;
const int rightBackward = 6;

const int IRright = A0;
const int IRleft = A1;
const int IRcenter = A2;



//Car position
const float carRadius = 7.65;
const float wheelRadius = 3.15;

uint8_t currentCell,lastCell;
uint8_t currentDegree = 1;
uint8_t dest = 0b00100010;

uint16_t findedPath [int(COL*ROW/2)];

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


void setup()
{
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
    

    
    // decleration of pins type //
    pinMode(trigPin, OUTPUT); // Sets the trigPin as an Output
    pinMode(echoPin, INPUT); // Sets the echoPin as an Input
    pinMode(trigPin2, OUTPUT); // Sets the trigPin as an Output
    pinMode(echoPin2, INPUT); // Sets the echoPin as an Input
    myservo.attach(11);
    // initillize grid as if there doesn't exist any obstacles 
    Inilizegrid();
    removeAllBlocks();
    Serial.begin(9600);

     //Read destination from app
     //Serial with bluetooth
     //uint8_t dest = 0b11101110; //14,14
    //Create Grid
    Serial.println("Call creategrid");
    CreateGrid();

    //find path with Astar

    //removeAllBlocks();
    uint8_t src = 0;    //0,0
   Serial.println("Call aStar");
    aStarSearch(src, dest);
        Serial.println("end aStar");

    lastCell = findedPath[0];
  Serial.println(lastCell);
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

/*******************************************************************************************************************************/
void Updategrid(int x,int y)
{
  int mx=0;
  int my=0;
// mapped read distance to required valuce according to mapped_grid_size (each cell indicate ? distance on real)
  mx = (int(x / maped_grid_size))+ ( (currentCell & PAIRI) >> ShPAIRI );
  my = (int(y / maped_grid_size))+ ( (currentCell & PAIRJ) >> ShPAIRJ );
  Serial.println("x = ");
  Serial.println(mx);
  Serial.println("y = ");
  Serial.println(my);
  if ((mx >=0) && (mx < SIZE) && (my >=0) && (my < SIZE))
    addBlock(mx,my);
}
/*******************************************************************************************************************************/
void Inilizegrid()
{
  for (int i = 0; i < SIZE; i++)
    for (int j = 0; j < SIZE; j++)
      grid[i][j] = UNBLOCKED | GVAl | PAIRI | PAIRJ; // to indicate that there doesn't exist any obstacle

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
    Serial.println(a);
    x = d*sin(radians(a)/**PI/180.0 /*+ currentDegree/2.0*/);
    y = d*cos(radians(a)/**PI/180.0 /*+ currentDegree/2.0*/);
}
/*******************************************************************************************************************************/
void setservoangle()
{
  // send required angle to servo //
  myservo.write(servo_angle);
}
/*******************************************************************************************************************************/



/*
A C Program to implement A* Search Algorithm 
This Program is Optimze the Size of Memory it uses
4058 bytes of program storage space (Code Size)
1058 bytes of dynamic memory (Global variables Size)
The Code is Written to be run on Microcontroller that's why I develop it using bits to get the optimal variables size
ALL THE INITIALIZED BITS HAS BEEN USED
OPTIMIZED FOR 15 * 15 GRID MAPS
*/ 


/*to get pair of numbers will use 8 bit int 
bits 1-4 parent i
bits 5-8 parent j
*/


/* access list is array of 32 bit 
every 2 bits contain information for 1 cell 
1 bit for is the cell in the closed list
2 bit for is the cell in the open list
32 bit (one entary of the array means 16 cell = col)*/

// A Utility Function to initilze Access list
void initAccessList ()
{
    for (uint8_t i =0 ;i<SIZE;i++)
    {
        accessList[i]=0;
    }
}
// A Utility Function to get the condition of the cell in closed list or not
bool isInClosedList (uint8_t i,uint8_t j)
{
    return accessList[i] & (1<<(j*2));
}
// A Utility Function to get the condition of the cell in open list or not
bool isInOpenList (uint8_t i,uint8_t j)
{
    return accessList[i] & (1<<(j*2+1));
}
// A Utility Function to add cell in closed list
void addToClosedList (uint8_t i,uint8_t j)
{
    accessList[i] |= (1<<(j*2));
}
// A Utility Function to add cell in open list or not
void addToOpenListAccess (uint8_t i,uint8_t j)
{
    accessList[i] |= (1<<(j*2+1));
}

//A Utility Function to compare integers in array to sort 
int compareFunction (const void * elem1, const void * elem2) 
{
    return *((uint16_t*)elem1) - *((uint16_t*)elem2);
}


/*open List of the algorithm
bits 8-15 f value
bits 1-7  pair of cell
*/

// A Utility Function to initialize open list
void initOpenList(uint8_t f,uint8_t i,uint8_t j)
{
    for (uint8_t i =0;i<SIZE*SIZE;i++)
    {
        openList[i] = FVAl | PAIRI | PAIRJ; //initialize with the max
    }
    openList[0] = f<<ShFVAl | i<<ShPAIRI | j<<ShPAIRJ;
    addToOpenListAccess(i,j);

}
// A Utility Function to insert on open list
void insertOpenList(uint8_t f,uint8_t i,uint8_t j,uint8_t at)
{
    openList[SIZE*SIZE-1-at] = f<<ShFVAl | i<<ShPAIRI | j<<ShPAIRJ;
    addToOpenListAccess(i,j);
    return;
}
// A Utility Function to delete and get from open list
// retrun false if the array is empty, return i,j if array has values
bool getOpenList(uint8_t &i,uint8_t &j)
{
    qsort (openList, sizeof(openList)/sizeof(*openList), sizeof(*openList), compareFunction); //sort
    uint16_t cell = openList[0]; //get the min
    if (cell ==  (FVAl | PAIRI | PAIRJ))
        return false;   //if the min equal the max ... the array is empty 
    
    openList[0] = FVAl | PAIRI | PAIRJ;
    i = (cell & PAIRI)>>ShPAIRI;
    j = (cell & PAIRJ)>>ShPAIRJ;
    return true;
}




// Check Map Utility Functions
// A Utility Function to check whether given cell (row, col)
// is a valid cell or not.
bool isValid(uint8_t point)
{
    // Returns true if row number and column number
    // is in range
    return (((point&PAIRI)>>ShPAIRI) >= 0) && (((point&PAIRI)>>ShPAIRI) < SIZE) &&
        (((point&PAIRJ)>>ShPAIRJ) >= 0) && (((point&PAIRJ)>>ShPAIRJ) < SIZE);
}

// A Utility Function to check whether the given cell is
// blocked or not
bool isUnBlocked(uint8_t point)
{
    // Returns true if the cell is not blocked else false
    if (grid[((point&PAIRI)>>ShPAIRI)][((point&PAIRJ)>>ShPAIRJ)] >= UNBLOCKED)
        return (true);
    else
        return (false);
}

// A Utility Function to check whether destination cell has
// been reached or not
bool isDestination(uint8_t point, uint8_t dest)
{
    if (((point&PAIRI)>>ShPAIRI) == ((dest&PAIRI)>>ShPAIRI) && ((point&PAIRJ)>>ShPAIRJ) == ((dest&PAIRJ)>>ShPAIRJ))
        return (true);
    else
        return (false);
}

// A Utility Function to calculate the 'h' L1 distance h=abs(x1-x2)+abs(y1-y2).
uint8_t calculateHValue(uint8_t point, uint8_t dest)
{
    // Return using the distance formula
    return abs(((point&PAIRI)>>ShPAIRI) - ((dest&PAIRI)>>ShPAIRI)) +abs(((point&PAIRJ)>>ShPAIRJ) - ((dest&PAIRJ)>>ShPAIRJ));
}

// A Utility Function to calculate the 'f' for point.
uint8_t getFValue(uint8_t i,uint8_t j,uint8_t dest)
{
    return ((grid[i][j] & GVAl) >> ShGVAl) + calculateHValue((i<<ShPAIRI)|(j<<ShPAIRJ) ,dest) ;
}



// A Utility Function to trace the path from the source
// to destination 
// I just print the path, Do Whatever you want after find the path
void pathFinded(uint8_t dest , uint8_t src)
{
    
    uint8_t i = 0;
    uint8_t j = 255;
    findedPath[0] = dest | (j<<ShGVAl);
    j--;
    i++;
    uint8_t cell = dest;
    while (cell != src)
    {
        // Serial.print("<- (%d,%d) ",(cell & PAIRI)>>ShPAIRI,(cell & PAIRJ)>>ShPAIRJ);
        cell = grid[((cell & PAIRI)>>ShPAIRI)][((cell & PAIRJ)>>ShPAIRJ)] ;
        findedPath[i] = cell | (j<<ShGVAl);
        i++;
        j--;
    }
    
    findedPath[i] = src | (j<<ShGVAl);
    i++;
    j--;
    
    for (;i<int(COL*ROW/2);i++)
    {
        findedPath[i]= UNBLOCKED | GVAl | PAIRI | PAIRJ;
    }

    qsort( findedPath, int(COL*ROW/2), sizeof(*findedPath),compareFunction);
    //finded path have the path from finded path[0] (source) to finded path[?] (destenation) and then 2**16
    //if you want x,y of x = (finded path[0] & PAIRI)>>ShPAIRI) , y = (finded path[0] & PAIRJ)>>ShPAIRJ)
    return;
}

uint8_t nextCell (uint8_t currentCell, uint8_t dest)
{
    uint8_t i =0;
    if (uint8_t(findedPath[i]) == uint8_t(UNBLOCKED | GVAl | PAIRI | PAIRJ))
        return uint8_t(UNBLOCKED | GVAl | PAIRI | PAIRJ);
        
    while (i< int(COL*ROW/2) && currentCell != uint8_t(findedPath[i]))
        i++;
    
    i++;
    uint8_t next = uint8_t(findedPath[i]);
    while (i< int(COL*ROW/2) && ((grid[((findedPath[i] & PAIRI)>>ShPAIRI)][((findedPath[i] & PAIRJ) >>ShPAIRJ)] & UNBLOCKED) >> ShUNBLOCKED) )
        i++;

    if (i== int(COL*ROW/2))
        return next;
    
    aStarSearch(currentCell,dest);
    if (uint8_t(findedPath[0]) == uint8_t(UNBLOCKED | GVAl | PAIRI | PAIRJ))
        return uint8_t(UNBLOCKED | GVAl | PAIRI | PAIRJ);

    return findedPath[1];
        
    
}

// A Function to find the shortest path between
// a given source cell to a destination cell according
// to A* Search Algorithm

void aStarSearch(uint8_t src, uint8_t dest)
{
    // Create a closed list and initialise it to false which means
    // that no cell has been included yet
    // This closed list is implemented as a boolean 2D array
    initAccessList();
 
    uint8_t i, j;
 
    for (i=0; i<SIZE; i++)
    {
        for (j=0; j<SIZE; j++)
        {
            grid[i][j] |= GVAl | PAIRI | PAIRJ; //initalinze with max values
        }
    }
 
    // Initialising the parameters of the starting node
    i = ((src&PAIRI)>>ShPAIRI), j = ((src&PAIRJ)>>ShPAIRJ);
    grid[i][j] &= (~GVAl);  //G distanse equal zero
    grid[i][j] |= (i<<ShPAIRI) | (j<<ShPAIRJ);  //set itself as parent 
 
    /*
    Create an open list having information as-
    f, i, j
    where f = g + h,
    and i, j are the row and column index of that cell
    Note that 0 <= i <= ROW-1 & 0 <= j <= COL-1*/
    // Put the starting cell on the open list and set its
    // 'f' as 0
    initOpenList(0,i,j);
     
    while (true)
    {
      
        // Remove this vertex from the open list
        if (getOpenList(i,j) == false)
            break;

        // Add this vertex to the closed list
        addToClosedList(i,j);

        // To store the 'g', 'h' and 'f' of the 8 successors
        uint8_t gNew, hNew, fNew;
        uint8_t iNew,jNew,point;
        
        /*  Generating all the 4 successor of this cell 

                      N   
                      |   
                      |  
                W----Cell----E
                      | 
                      |  
                      S

            Cell-->Popped Cell (i, j)
            N -->  North       (i-1, j)
            S -->  South       (i+1, j)
            E -->  East        (i, j+1)
            W -->  West        (i, j-1)*/
        int8_t directionsi[4] = {-1,1,0,0};
        int8_t directionsj[4] = {0,0,1,-1};
        for (uint8_t direc = 0; direc < AVILABLEMOVES; direc++)
        {   
            iNew = i + directionsi[direc];
            jNew = j + directionsj[direc];
            
            // Only process this cell if this is a valid one
            if (isValid( (iNew<<ShPAIRI) | (jNew<<ShPAIRJ) ) == true)
            {
                // If the destination cell is the same as the
                // current successor
                if (isDestination( (iNew<<ShPAIRI) | (jNew<<ShPAIRJ), dest) == true)
                {
                    // Set the Parent of the destination cell
                    grid[iNew][jNew] &= ~ ((PAIRI|PAIRJ));
                    grid[iNew][jNew] |= (i<<ShPAIRI) | (j<<ShPAIRJ) ;
                    //printf ("The destination cell is found\n");
                    Serial.println("T"); 
                    pathFinded (dest,src);
                    return;
                }
                // If the successor is already on the closed
                // list or if it is blocked, then ignore it.
                // Else do the following
                else if (isInClosedList(iNew,jNew) == false &&
                        isUnBlocked( (iNew<<ShPAIRI) | (jNew<<ShPAIRJ) ) == true)
                {
                    gNew = ((grid[i][j] & GVAl)>>ShGVAl) + EFFORT;
                    hNew = calculateHValue ((iNew<<ShPAIRI) | (jNew<<ShPAIRJ), dest);
                    fNew = gNew + hNew;
    
                    // If it isn’t on the open list, add it to
                    // the open list. Make the current square
                    // the parent of this square. Record the
                    // f, g, and h costs of the square cell
                    //                OR
                    // If it is on the open list already, check
                    // to see if this path to that square is better,
                    // using 'f' cost as the measure.
                    if ( getFValue(iNew,jNew,dest) > fNew)
                    {
                        insertOpenList(fNew,iNew,jNew,direc);

                        // Update the details of this cell
                        grid[iNew][jNew] &= ~ (PAIRI|PAIRJ|GVAl);
                        grid[iNew][jNew] |= (i<<ShPAIRI) | (j<<ShPAIRJ) | (gNew<<ShGVAl);
                    }
                }
            }
        }
    }
    // When the destination cell is not found and the open
    // list is empty, then we conclude that we failed to
    // reach the destiantion cell. This may happen when the
    // there is no way to destination cell (due to blockages)
    // printf("Failed to find the Destination Cell\n"); 
    for (i=0;i<int(COL*ROW/2) ; i++)
     findedPath [i]= UNBLOCKED | GVAl | PAIRI | PAIRJ;
    return;
}

 // A Utility Functions to add and remove blocks on the map
void addBlock (uint8_t i,uint8_t j)
{
    grid[i][j] &= ~(1<<ShUNBLOCKED);
}
void initGrad()
{
    for (uint8_t i =0;i<SIZE;i++)
        for (uint8_t j =0;j<SIZE;j++)
            grid[i][j] = UNBLOCKED | GVAl | PAIRI | PAIRJ;
}
void removeAllBlocks ()
{
    for (uint8_t i =0;i<SIZE;i++)
        for (uint8_t j =0;j<SIZE;j++)
            grid[i][j] |= (1<<ShUNBLOCKED);
}


void CreateGrid()
{
  Serial.println("Create grid begin");
    servo_angle = 0;
    // loop for 360 degree to get region's grid 
    while (servo_angle<=180)
    {
      // note all of this functions works on only one ultrasonic 
      float distance = 0.0;
      int x=0;
      int y=0;
      // change angle of servo in order to get data from ultrasonic
    
     setservoangle();
     //Serial.println(servo_angle);
      // get reading of ultrasonic and check if there exist obstacle or not +
     bool obstacle =Ultrasonicread(distance,1);
     //Serial.println(distance);
      // if there exit obstacle convert it to be mapped and updated in grid
      if (obstacle)  {
        Conversion(distance,servo_angle,x,y);
        Serial.println("distance = ");
        Serial.println(distance);
        Serial.println("angle = ");
        Serial.println(servo_angle);
        Updategrid(x, y);
     }
     obstacle =Ultrasonicread(distance,2);
     //Serial.println(distance);
      // if there exit obstacle convert it to be mapped and updated in grid
      if (obstacle)
       {
        Conversion(distance,(servo_angle+180),x,y);
        Serial.println("distance2 = ");
        Serial.println(distance);
        Serial.println("angle2 = ");
        Serial.println(servo_angle);
        Updategrid(x, y);

      }
     // increment servo angle to check following places 
      servo_angle += 30;
      delay(10000);
    //  printing grid 
//     for (int i = 0; i < SIZE; i++)
//        for (int j = 0; j < SIZE; j++)
//            Serial.println (grid[i][j]);
      //delay(1000);
  }

Serial.println("printing grid");
  for (int i = 0; i < SIZE; i++)
  {
    Serial.println("next line");
        for (int j = 0; j < SIZE; j++)
           Serial.println (grid[i][j]);
  }
    

  Serial.println("Create grid end");

  
    
}

void Move( int8_t deltaX, int8_t deltaY )
{
  Serial.println(deltaX);
  Serial.println(deltaY);
  int8_t degree;
  if (deltaX == 1)
    degree =0;
  else if (deltaX == -1)
     degree =2;

  if (deltaY == 1)
    degree =1;
  else if (deltaY == -1)
     degree =3;

     
  int8_t difference = degree - currentDegree;
  
  if (abs(difference) == 2 )
  {  
     //right
     //right
     /*RunRightward();
     RunRightward();*/
     Serial.println("run right right");
  }
  else if ( difference == 1 || difference == -3)
  {
      //left
      //RunLeftward();
      Serial.println("run left");
   }
  else if ( difference == -1 || difference == 3)
  {   
    //right
    //RunRightward();
    Serial.println("run right");
  }   
  
  //forward
  //RunForward();
  Serial.println("run forward");
  currentDegree = degree;    

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
            analogWrite(rightForward,150);
            analogWrite(leftForward,0);
        }
        else if (rightSensor > 200)
        {
            analogWrite(rightForward,0);
            analogWrite(leftForward,130);
        }
        else
        {
            analogWrite(rightForward,150);
            analogWrite(leftForward,130);
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
  
  int degree = 68;
  angleLeft = 0;
  angleRight = 0;
  while( (rightSensor < 200 /*||  leftSensor < 200*/ || centerSensor < 200) )
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

  while (angleRight <= degree && angleLeft <= degree )
  {
    
    if (angleRight == angleLeft)
    {
       analogWrite(rightForward,180);
       analogWrite(leftBackward,130);  
    }
    else if (angleRight>angleLeft)
    { 
       analogWrite(rightForward,180);
       analogWrite(leftBackward,0);  
    }
    else if (angleLeft>angleRight)
    {
       analogWrite(rightForward,0);
       analogWrite(leftBackward,130);
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
  
  int degree = 68;
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
       analogWrite(rightBackward,130);  
    }
    else if (angleRight>angleLeft)
    { 
       analogWrite(leftForward,170);
       analogWrite(rightBackward,0);  
    }
    else if (angleLeft>angleRight)
    {
       analogWrite(leftForward,0);
       analogWrite(rightBackward,130);
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


void loop()
{
    currentCell = nextCell(lastCell,dest);
    Serial.println(lastCell);
    Serial.println(currentCell);
    Serial.println("Call move");
    Move( ( (currentCell & PAIRI)>>ShPAIRI) - ((lastCell & PAIRI)>>ShPAIRI),( (currentCell & PAIRJ)>>ShPAIRJ ) - ((lastCell & PAIRJ)>>ShPAIRJ));  
    //CreateGrid();
    Serial.println("move ended");
  
    if ( currentCell == uint8_t(UNBLOCKED|GVAl|PAIRI|PAIRJ ) )
       {
          //Print on app "destination not found"
          Serial.println("not found");
       }
  
    if (currentCell == dest )
       {
          //Print on app "destination found"
          Serial.println("found");
       }

    lastCell = currentCell ;     
   
}
