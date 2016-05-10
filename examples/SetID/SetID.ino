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
#define DesiredID 0x00

//declare G15 Class Object
//servo1 ID=0xFE
G15 servo_broadcast(0xFE); 


void setup(){
  
//initialize the arduino main board's serial/UART and Control Pins
//CTRL pin for G15 =3 and AX12 =8
  G15ShieldInit(19200,3,8); 
  
//call the init function to init servo obj
  servo_broadcast.init();           

//init LED indicator as output
  pinMode(LED_BOARD,OUTPUT);  
  digitalWrite(LED_BOARD, LOW); 
  
  delay(1000); 
  
}
void loop(){
   
  //connect one servo only at a time 
  
  //LED will blink at speed of 1 Hz if sucessfully change the ID
  //if fail LED will blink fast.
  
  //broadcast to set the ID, change the DesiredID to ID you wish to set the servo
  servo_broadcast.SetID(DesiredID); 
  
  ERROR=servo_broadcast.Ping(DATA); 

  if(ERROR==0 || ERROR==0x0400)    //ignore ID mistmatch since broadcast ID us used to Ping the servo
   {        
     if(DATA[0]==DesiredID){
      while(1){     //blink LED if failed   
        digitalWrite(LED_BOARD, LOW); 
        delay(1000); 
        digitalWrite(LED_BOARD, HIGH); 
        delay(1000); 
      }
     }
     else{       
        while(1){     //blink LED if failed   
        digitalWrite(LED_BOARD, LOW); 
        delay(200); 
        digitalWrite(LED_BOARD, HIGH); 
        delay(200); 
      }
     }
     
   }  
  else{
    while(1){
      digitalWrite(LED_BOARD, LOW); 
      delay(100); 
      digitalWrite(LED_BOARD, HIGH); 
      delay(100); 
    }
  }  
 

}
