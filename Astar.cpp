/*
A C Program to implement A* Search Algorithm 
This Program is Optimze the Size of Memory it uses
4058 bytes of program storage space (Code Size)
1058 bytes of dynamic memory (Global variables Size)
The Code is Written to be run on Microcontroller that's why I develop it using bits to get the optimal variables size
ALL THE INITIALIZED BITS HAS BEEN USED
OPTIMIZED FOR 15 * 15 GRID MAPS
*/ 
#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>

//Row = i , Col = j 
#define ROW 15
#define COL 15
// 4 means four direction (no cross), 8 means the eight deirections (include the cross)
#define AVILABLEMOVES 4
#define EFFORT 1
/*to get pair of numbers will use 8 bit int 
bits 1-4 parent i
bits 5-8 parent j
*/
#define PAIRI 15
#define ShPAIRI 0
#define PAIRJ 240
#define ShPAIRJ 4


void aStarSearch(uint8_t src, uint8_t dest);
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
uint16_t grid[ROW][COL];
#define UNBLOCKED     32768
#define ShUNBLOCKED   15
#define GVAl          32512
#define ShGVAl        8

// A Utility Functions to add and remove blocks on the map
void addBlock (uint8_t i,uint8_t j)
{
    grid[i][j] &= ~(1<<ShUNBLOCKED);
}
void removeAllBlocks ()
{
    for (uint8_t i =0;i<COL;i++)
        for (uint8_t j =0;j<COL;j++)
            grid[i][j] |= (1<<ShUNBLOCKED);
}

/* access list is array of 32 bit 
every 2 bits contain information for 1 cell 
1 bit for is the cell in the closed list
2 bit for is the cell in the open list
32 bit (one entary of the array means 16 cell = col)*/

// will be initialzed on Astar function 
//uint32_t accessList [ROW];

// A Utility Function to initilze Access list
void initAccessList (uint32_t* accessList)
{
    for (uint8_t i =0 ;i<ROW;i++)
    {
        accessList[i]=0;
    }
}
// A Utility Function to get the condition of the cell in closed list or not
bool isInClosedList (uint32_t* accessList,uint8_t i,uint8_t j)
{
    return accessList[i] & (1<<(j*2));
}
// A Utility Function to get the condition of the cell in open list or not
bool isInOpenList (uint32_t* accessList,uint8_t i,uint8_t j)
{
    return accessList[i] & (1<<(j*2+1));
}
// A Utility Function to add cell in closed list
void addToClosedList (uint32_t* accessList,uint8_t i,uint8_t j)
{
    accessList[i] |= (1<<(j*2));
}
// A Utility Function to add cell in open list or not
void addToOpenListAccess (uint32_t* accessList,uint8_t i,uint8_t j)
{
    accessList[i] |= (1<<(j*2+1));
}

//A Utility Function to compare integers in array to sort 
int compareFunction (const void * elem1, const void * elem2) 
{ 
  return ((*((uint16_t*)elem1)) > (*((uint16_t*)elem2))) ? 1:-1;
}


/*open List of the algorithm
bits 8-15 f value
bits 1-7  pair of cell
*/

// will be initialzed on Astar function 
// uint16_t openList [ROW*COL];
#define FVAl          65280
#define ShFVAl        8
// A Utility Function to initialize open list
void initOpenList(uint16_t * openList,uint32_t* accessList,uint8_t f,uint8_t i,uint8_t j)
{
    for (uint8_t i =0;i<ROW*COL;i++)
    {
        openList[i] = FVAl | PAIRI | PAIRJ; //initialize with the max
    }
    openList[0] = f<<ShFVAl | i<<ShPAIRI | j<<ShPAIRJ;
    addToOpenListAccess(accessList,i,j);

}
// A Utility Function to insert on open list
void insertOpenList(uint16_t * openList,uint32_t * accessList,uint8_t f,uint8_t i,uint8_t j,uint8_t at)
{
    openList[ROW*COL-1-at] = f<<ShFVAl | i<<ShPAIRI | j<<ShPAIRJ;
    addToOpenListAccess(accessList,i,j);
    return;
}
// A Utility Function to delete and get from open list
// retrun false if the array is empty, return i,j if array has values
bool getFromOpenList(uint16_t * openList,uint8_t &i,uint8_t &j)
{
    qsort (openList, ROW*COL , sizeof(*openList), compareFunction); //sort
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
    return (((point&PAIRI)>>ShPAIRI) >= 0) && (((point&PAIRI)>>ShPAIRI) < ROW) &&
        (((point&PAIRJ)>>ShPAIRJ) >= 0) && (((point&PAIRJ)>>ShPAIRJ) < COL);
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
uint16_t findedPath [int(COL*ROW/2)];
void pathFinded(uint8_t dest , uint8_t src)
{
    
    uint8_t i = 0;
    uint8_t j = 255;
    findedPath[0] = dest | (j<<ShGVAl);
    j--;
    i++;
    uint8_t cell =dest;
    while (cell != src)
    {
        printf("<- (%d,%d) ",(cell & PAIRI)>>ShPAIRI,(cell & PAIRJ)>>ShPAIRJ);
        cell = grid[((cell & PAIRI)>>ShPAIRI)][((cell & PAIRJ)>>ShPAIRJ)] ;
        findedPath[i] = cell | (j<<ShGVAl);
        i++;
        j--;
    }
    
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
    uint32_t accessList [ROW];
    initAccessList(accessList);
 
    uint8_t i, j;
 
    for (i=0; i<ROW; i++)
    {
        for (j=0; j<COL; j++)
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
    uint16_t openList [ROW*COL];
    initOpenList(openList,accessList,0,i,j);
     
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

    // To store the 'g', 'h' and 'f' of the successors
    uint8_t gNew, hNew, fNew;
    uint8_t iNew,jNew,point;

    while (true)
    {
        // Remove this vertex from the open list
        if (getFromOpenList(openList,i,j) == false)
            break;

        // Add this vertex to the closed list
        addToClosedList(accessList,i,j);

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
                    // Serial.print ("The destination cell is found\n");
                    pathFinded (dest,src);
                    return;
                }
                // If the successor is already on the closed
                // list or if it is blocked, then ignore it.
                // Else do the following
                else if (isInClosedList(accessList,iNew,jNew) == false &&
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
                        insertOpenList(openList,accessList,fNew,iNew,jNew,direc);

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
    // Serial.print("Failed to find the Destination Cell\n"); 
    for (i=0;i<int(COL*ROW/2) ; i++)
        findedPath [i]= UNBLOCKED | GVAl | PAIRI | PAIRJ;

    return;
}
 
 
// Driver program to test above function
int main()
{
    // Serial.begin(9600);
    removeAllBlocks();
    uint8_t src = 0;    //0,0
    uint8_t dest = 0b11101110; //14,14
    aStarSearch(src, dest);
    
    uint8_t test = nextCell(4,dest);
    printf("\n%d\n",uint8_t(test));
    addBlock(5,0);
    test = nextCell(4,dest);

    printf("\n%d\n",uint8_t(test));
    printf("\n%d\n",uint8_t(findedPath[0]));

    // Serial.print("\n");
    return(0);
}