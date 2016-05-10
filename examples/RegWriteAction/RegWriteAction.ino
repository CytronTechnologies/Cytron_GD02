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
//servo1 ID=1
G15 servo1(1); 
//servo2 ID=2
G15 servo2(2); 


void setup(){
  
//initialize the arduino main board's serial/UART and Control Pins
  G15ShieldInit(19200,3,8); 
  
//call the init function to init servo obj
  servo1.init();           
  servo2.init(); 

//init LED indicator as output
  pinMode(LED_BOARD,OUTPUT);  
  digitalWrite(LED_BOARD, LOW); 
  
  delay(500); 
  digitalWrite(LED_BOARD, HIGH);
  
}
void loop(){
    

    servo1.SetPos(ConvertAngle2Pos(0),iREG_WRITE);    //goto 0 degree pos 
    servo2.SetPos(ConvertAngle2Pos(0),iREG_WRITE);    //goto 0 degree pos 
    
    servo1.SetLED(ON, iWRITE_DATA); 
    servo2.SetLED(ON, iWRITE_DATA); 
    
    delay(1000); 
    
    G15::SetAction();
    
    servo1.SetLED(OFF, iWRITE_DATA); 
    servo2.SetLED(OFF, iWRITE_DATA);  
    
    servo1.SetPos(ConvertAngle2Pos(90),iREG_WRITE);    //goto 90 degree pos 
    servo2.SetPos(ConvertAngle2Pos(90),iREG_WRITE);    //goto 90 degree pos 
    
    servo1.SetLED(ON, iWRITE_DATA); 
    servo2.SetLED(ON, iWRITE_DATA); 
    
    delay(1000);
    
    G15::SetAction(); 
    servo1.SetLED(OFF, iWRITE_DATA); 
    servo2.SetLED(OFF, iWRITE_DATA); 
    
   
     
    
}
