
//char dataIn = 0;            //Variable for storing received data
//void setup()
//{
//    Serial.begin(9600);   //Sets the baud for serial data transmission                               
//}
//void loop()
//{
//   if(Serial.available() > 0)      // Send data only when you receive data:
//   {
//      dataIn = Serial.read();        //Read the incoming data & store into data
//      Serial.print(dataIn);          //Print Value inside data in Serial monitor      
//      if(dataIn == 'r') {             // Checks whether value of data is equal to 1
//         Serial.print ('d');}
//      else if(dataIn == '0')         //  Checks whether value of data is equal to 0
//         digitalWrite(13, LOW);    //If value is 0 then LED turns OFF
//   }
//}



char dataIn = 'r';            //Variable for storing received data
bool sended=true;
bool recieve_destination=true;
int destination=0;
int current_position=10;
int xdest=0;
int ydest=0;
int xcurr=0;
int ycurr=0;
void setup()
{
    Serial.begin(9600);   //Sets the baud for serial data transmission 
     // check that bluetooth connected to mobile app correctly 
while (dataIn!='c'){
  if(Serial.available() > 0)      // Send data only when you receive data:
  {
      dataIn = Serial.read();        //Read the incoming data & store into data
      Serial.println(dataIn);}
  }
  Serial.println("finsh");
  // cofirum that connection is right
  while(sended){
   if(Serial.available() > 0)      // Send data only when you receive data:
   {
       Serial.write('s');        //Read the incoming data & store into data
       Serial.println("s sent ");
       sended=false;
       }       
  }    

  while(recieve_destination)
  {
     if(Serial.available() > 0)      // Send data only when you receive data:
  {
      current_position=Serial.read();
      Serial.println(" position 1 recieved ");
      ycurr=(ycurr/15);
      xcurr=(current_position%15);
      recieve_destination =false;
    
  } }
  
  recieve_destination =true;
  while(recieve_destination)
  {
     if(Serial.available() > 0)      // Send data only when you receive data:
  {
      destination = Serial.read();        //Read the incoming data & store into data
      Serial.println(destination);
      ydest=(destination/15);
      xdest=(destination%15);
      Serial.println(xdest);
      Serial.println(ydest);
      recieve_destination=false;}
         
  }
}

void loop()
{
   Serial.println("all is done");
   
}


