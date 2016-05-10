#include <G15.h>    // include the library
#define LED_BOARD 13


/*Return Error definitions & Mask
=====Higher 8 bits of Word=========
packet length error :       0x0100
packet header error:        0x0200
ID mismatch error:          0x0400
packet checksum mismatch :  0x0800
====Lower 8 bits of word==========
Error status return by G15:
INST			0x0040		
OVERLOAD		0x0020
CHECKSUM		0x0010
RANGE			0x0008
OVERHEAT		0x0004
ANGLELIMIT 	        0x0002
VOLTAGE		        0x0001
*/

//declaration of variables & object
word ERROR=0;
byte DATA[10]; 
word STATUS;

//declare G15 Class Object
//servo1 array with ID of 0,1,2 and 3
G15 servo[4]={0,1,2,3};


void setup(){
  
//initialize the arduino main board's serial/UART and Control Pins
//CTRL pin for G15 =3 and AX12 =8
  G15ShieldInit(19200,3,8); 
  
//call the init function to init servo obj

  for(int i=0; i<4; i++)
  {
    servo[i].init();
  }    

//init LED indicator as output
  pinMode(LED_BOARD,OUTPUT);  
  digitalWrite(LED_BOARD, LOW); 
  
  delay(3000); 
  
}
void loop(){
   
  for(int i=0; i<4; i++)
  {
    servo[i].SetLED(ON, iWRITE_DATA); 
    servo[i].SetPos(ConvertAngle2Pos(0),iWRITE_DATA);
    delay(1000); 
    servo[i].SetPos(ConvertAngle2Pos(90),iWRITE_DATA);
    delay(1000); 
    servo[i].SetLED(OFF, iWRITE_DATA); 
    
  }
  
      
      
}
