/*
//Cytron Technologies

*/

#include "Arduino.h"
#include "HardwareSerial.h"
#include "G15.h"
//variables
 char G15CTRL; 
 char AX12CTRL;

// This Code is using Serial library, hence
// baudrate 300, 1200, 2400, 4800, 9600, 14400, 19200, 28800, 38400, 57600, or 115200
// tested to run baudrate at 500k, no problem
//EN1 Ctrl pin can be D8 or D2 for AX12 port
//EN2 ctrl pin can be D9 or D3 for G15 port

void G15ShieldInit(long baud, char G15_CTRL, char AX12_CTRL){
	Serial.begin (baud) ; 
	Serial.setTimeout(SerialTimeOut); 
	G15CTRL=G15_CTRL;
	AX12CTRL=AX12_CTRL;
	
	pinMode(G15CTRL, OUTPUT);		//control pin setup	
	pinMode(AX12CTRL, OUTPUT);		//control pin setup	
	
	digitalWrite(G15CTRL,TxMode);
	digitalWrite(AX12CTRL,TxMode);
	

}
// void waitTXC(void){  // use to wait until all data has left Arduino before changing buffer direction

      // bitSet(UCSR0A, TXC0);
      // while (bit_is_clear(UCSR0A, TXC0));
// }


G15::G15(byte ID)	//, char ctrl)
{   
  ServoID=ID; 
  init();
}

AX12::AX12(byte ID):G15(ID) //,ctrl)			//inherit G15 constructor
{
	// Add more initializations here if exist

}

void G15::init(void)
{
	TxRx=G15CTRL; 	
   
}
void AX12::init(void)
{
	TxRx=AX12CTRL;
	pinMode(TxRx, OUTPUT);		//control pin setup	
	setTX(); 
}	

void G15::setTX(void){  // set the dynamixel bus buffer direction to out

	digitalWrite(TxRx,TxMode); 
	
}
void G15::setRX(void){ // set the dynamixel bus buffer direction to in
     digitalWrite(TxRx,RxMode); 
}


//*=================send packet==========================================================
// caution: at least 2 bytes of data array need to be passed into the function 

word G15::send_packet(byte ID, byte inst, byte* data, byte param_len)	
{
	int i; 
	byte packet_len = 0;
    byte TxBuff[16];	
    char Status[16];
	char checksum=0; 		//Check Sum = ~ (ID + Length + Instruction + Parameter1 + ... Parameter N)
	word error=0; 
	
	setTX(); 					//set for transmit mode
	
	checksum=0;					//clear checksum value
    TxBuff[0] = 0xFF;			//0xFF not included in checksum
    TxBuff[1] = 0xFF;
    TxBuff[2] = ID; 			checksum += TxBuff[2];	//0-254, 0xFE = broadcast id
    TxBuff[3] = param_len + 2;	checksum += TxBuff[3];	//INSTRUCTION + PARAMETERS( START ADDR + VALUES ) + CHECKSUM                                                                                                        //0xFF and ID not included
    TxBuff[4] = inst;			checksum += TxBuff[4];	

    for(i = 0; i < param_len; i++)		//data
    {
		TxBuff[i+5] = data[i];
		checksum += TxBuff[i+5];
    }
    TxBuff[i+5] = ~checksum; 				//Checksum with Bit Inversion

    packet_len = TxBuff[3] + 4;			//# of bytes for the whole packet
    
	for(i=0; i<packet_len;i++){
		Serial.write(TxBuff[i]);
	}
	Serial.flush();
	//waitTXC();		//arduino version 1.01 only
       
    //Status[4]=0x00;		//clear status byte
	
    // we'll only get a reply if it was not broadcast
    if((ID != 0xFE) || (inst == iPING))
	{
        if(inst == iREAD_DATA)			//if a read instruction
		{
			param_len = data[1]; 
            packet_len = data[1] + 6;  	// data[1] = length of the data to be read
		}
        else
		{
            packet_len = 6;
		}

		
		setRX(); 						//set to receive mode and start receiving from G15		
		byte readcount= Serial.readBytes(Status, packet_len); 
		
		setTX();  						//set back to tx mode to prevent noise into buffer
		
		// Serial.write(0xAA); 
		// for(i=0; i<packet_len; i++)
			// Serial.write(Status[i]); 
			
		//Checking received bytes
		error=0; 		//clear error 
		if(readcount!=packet_len){		
				
			error|=0x0100; 
			//return (error);			//packet lost or receive time out 
		}
		if ((Status[0] !=char(0xFF)) || (Status[1] != char(0xFF)))	
		{
			error|=0x0200; 
			//return (error);			//1000 00001	//wrong header
		}
		if (Status[2] != char(ID))
		{
			error|=0x0400;
			//return (error);			//ID mismatch
		}
		if(Status[4] != char(0))		
		{
			error|=word(Status[4]); 
			//return(error); 
		}
		// calculate checksum
		checksum = 0;					//clear checksum value
        for(i = 2; i < packet_len; i++)	//whole package including checksum but excluding header
        {
            checksum += Status[i];		//correct end result must be 0xFF
        }
        if(checksum != char(0xFF))
        {
            error |= 0x0800;       		//return packet checksum mismatch error
            //return (error);
        }
		if(Status[4]==char(0x00) && (error&0x0100)==0x00)	//copy data only if there is no packet error
		{       
			if(inst == iPING)
			{
				// ID is passed to the data[0]
				data[0] = Status[2];
			} 
			else if(inst == iREAD_DATA)
			{
				for(i = 0; i < param_len; i++)  //Requested Parameters
					data[i] = byte (Status[i+5]);
			}
		}
		
    }

    return(error); // return error code	 
	
}

word G15::SetWheelMode(void)	//10 bits speed (0-1024)
{		
	word Error=0; 
	Error=SetAngleLimit(0,0);	//enable wheel mode
	if(Error!=0) return (Error);
	
	Error = SetTorqueOnOff(1, iWRITE_DATA);	//enable torque		
	
	return(Error);    
}

word G15::ExitWheelMode(void)
{
	return(SetAngleLimit(0,1087));  //reset to default angle limit
}

word G15::SetWheelSpeed(word Speed, byte CW_CCW)
{
	Speed=Speed&0x03FF; 	//0000 0011 1111 1111 eliminate bits which are non speed
	if(CW_CCW){		//if CW
		Speed=Speed|0x0400; 
	}
	return(SetSpeed(Speed,iWRITE_DATA));	
}

//******************************************************************
//*	SET GOAL POSITION
//* 	eg:	dat[0] = 0xC8;				// Position lower byte
//*			dat[1] = 0x00;				// Position upper byte
//*			SetPos(dat,iWRITE_DATA);	// Send to servo 2 & action!
//******************************************************************
word G15::SetPos(word Position, byte Write_Reg)
{
	byte TxBuff[3];	
	
    TxBuff[0] = GOAL_POSITION_L;	//Control Starting Address
	TxBuff[1] = byte (Position&0x00FF);  			//goal pos bottom 8 bits
    TxBuff[2] = byte (Position>>8); 			//goal pos top 8 bits
	
	// write the packet, return the error code
	return(send_packet(ServoID, Write_Reg, TxBuff, 3));
}

/* byte G15::SetPosAngle(word Angle, byte Write_Reg){

	return(SetPos(ConvertAngle(Angle),Write_Reg));
	//return(SetPos(word(float(Angle)*1088.0/360.0),Write_Reg));

} */

word G15::RotateCW (word Position, byte Write_Reg){

	Position = Position|0xC000;  //directional positioning mode CW
		
	return(SetPos(Position, Write_Reg));

}
word G15::RotateCCW (word Position, byte Write_Reg){

	Position = Position|0x8000;  //directional positioning mode 
	Position = Position&0xBFFF;  //CCW	1011 1111 1111 1111
		
	return(SetPos(Position, Write_Reg));

}
//******************************************************************
//*	SET TORQUE ON OFF
//* 	eg:	SetTorqueOnOff(1,iREG_WRITE);	// Turn on torque of servo 2
//******************************************************************
word G15::SetTorqueOnOff(byte on_off, byte Write_Reg)
{
	byte TxBuff[2];

    TxBuff[0] = TORQUE_ENABLE;		//Control Starting Address
	TxBuff[1] = on_off; 			//ON = 1, OFF = 0

    // write the packet, return the error code
    return(send_packet(ServoID, Write_Reg, TxBuff, 2));
}
//******************************************************************
//*	SET SPEED
//* 	eg:	dat[0] = 0x0A;				// Speed lower byte
//*			dat[1] = 0x02;				// Speed upper byte
//*			SetSpeed(dat,iREG_WRITE);	// Save data in servo 2 register &
//*										// wait for action command
//******************************************************************
word G15::SetSpeed(word Speed, byte Write_Reg)
{
	byte TxBuff[3];

    TxBuff[0] = MOVING_SPEED_L;		//Control Starting Address
	TxBuff[1] = byte(Speed&0x00FF);			//speed bottom 8 bits
    TxBuff[2] = byte(Speed>>8); 			//speed top 8 bits
	
    // write the packet, return the error code
    return(send_packet(ServoID, Write_Reg, TxBuff, 3));
} 
//********************************************************************
//* 
//*
word G15::SetTimetoGoal(word Time,byte Write_Reg)
{
	Time = Time&0x0FFF; 	//			0000 1111 1111 1111
	Time = Time|0x8000; 	//bit 15 represents the time to goal pos mode
	
	return(SetSpeed(Time, Write_Reg)); 
}


//******************************************************************
//*	SET ANGLE LIMIT
//* byte SetAngleLimit(word CW_angle, word CCW_angle)	
//*	CW_angle & CCW_angle are not in degree value
//*	Use ConverAngle to convert angle values if needed		
//*	
//*		
//******************************************************************
word G15::SetAngleLimit(word CW_angle, word CCW_angle)
{
	byte TxBuff[5];
	word error; 

    TxBuff[0] = CW_ANGLE_LIMIT_L;			//Starting Address
	TxBuff[1] = byte(CW_angle&0x00FF);  	//CW limit bottom 8 bits
    TxBuff[2] = byte(CW_angle>>8); 			//CW limit top 8 bits
	TxBuff[3] = byte(CCW_angle&0x00FF); 	//CCW limit bottom 8 bits
	TxBuff[4] = byte(CCW_angle>>8); 		//CCW limit top 8 bits

	
	error = send_packet(ServoID, iWRITE_DATA, TxBuff, 5); 
	delay(10); 		//delay for eeprom write
    // write the packet, return the error code
    return(error);
}

word G15::SetTorqueLimit(word TorqueLimit)
{
	byte TxBuff[3];

    TxBuff[0] = TORQUE_LIMIT_L;				//Starting Address
	TxBuff[1] = byte(TorqueLimit&0x00FF);  	//Torque limit bottom 8 bits
    TxBuff[2] = byte(TorqueLimit>>8); 		//Torque limit top 8 bits

    // write the packet, return the error code
    return(send_packet(ServoID, iWRITE_DATA, TxBuff, 3));

}

word G15::SetTemperatureLimit(byte Temperature)
{
	byte TxBuff[2];
	word error;

    TxBuff[0] = LIMIT_TEMPERATURE;			//Starting Address
	TxBuff[1] = Temperature;  				//temperature

	error=send_packet(ServoID, iWRITE_DATA, TxBuff, 2);
	delay(10); 			//delay for eeprom write
    // write the packet, return the error code
    return(error);
}

word G15::SetVoltageLimit(byte VoltageLow, byte VoltageHigh)
{
	byte TxBuff[3];
	word error; 

    TxBuff[0] = DOWN_LIMIT_VOLTAGE;				//Starting Address
	TxBuff[1] = VoltageLow;  	//lower voltage limit 
    TxBuff[2] = VoltageHigh; 		//Higher voltage limit 

	error=send_packet(ServoID, iWRITE_DATA, TxBuff, 3);
	delay(10); 		//delay for eeprom write
    // write the packet, return the error code
    return(error);
}

//******************************************************************
//*	SET ID
//* 	eg:	SetID(MAIN,0xFE,3);	// Change the ID of any number to 3
//******************************************************************
word G15::SetID(byte NewID)
{
    byte TxBuff[2];
	word error; 
	
    TxBuff[0] = ID;			//Control Starting Address
	TxBuff[1] = NewID;	
	
	error=send_packet(ServoID, iWRITE_DATA, TxBuff, 2);
	ServoID=NewID; 
	delay(10); 			//delay for eeprom write
    return(error);
	
}	

//******************************************************************
//*	SET LED
//* 	eg:	SetLED(1,iWRITE_DATA);	// Turn on LED of servo 2
//******************************************************************
word G15::SetLED(byte on_off, byte Write_Reg)
{
	byte TxBuff[2];

    TxBuff[0] = LED;				//Control Starting Address
	TxBuff[1] = on_off; 			//ON = 1, OFF = 0

    // write the packet, return the error code
    return(send_packet(ServoID, Write_Reg, TxBuff, 2));
	
}	

word G15::SetAlarmLED(byte AlarmLED){

	byte alarmval=0x00; 	
	byte TxBuff[2];
	word error; 
	
	alarmval=alarmval|AlarmLED; 

    TxBuff[0] = ALARM_LED;		//Control Starting Address
	TxBuff[1] = alarmval;			//alarm val
   
	error=send_packet(ServoID, iWRITE_DATA, TxBuff, 2); 
	delay(10); 
	
    // write the packet, return the error code
    return(error);

}

word G15::SetAlarmShutDown(byte Alarm){

	byte alarmval=0x00; 	
	byte TxBuff[2];
	word error; 
	
	alarmval=alarmval|Alarm; 

    TxBuff[0] = ALARM_SHUTDOWN;		//Control Starting Address
	TxBuff[1] = alarmval;			//alarm
   
   error=send_packet(ServoID, iWRITE_DATA, TxBuff, 2); 
   delay(10); 	//delay for eeprom write
	
    // write the packet, return the error code
    return(error);

}

word G15::SetMarginSlopePunch(byte CWMargin, byte CCWMargin, byte CWSlope, byte CCWSlope, word Punch)
{
	byte TxBuff[5];	
	word error=0; 

    TxBuff[0] = CW_COMPLIANCE_MARGIN;		//Control Starting Address
	TxBuff[1] = CWMargin;	
	TxBuff[2] = CCWMargin; 
	TxBuff[3] = CWSlope; 
	TxBuff[4] = CCWSlope; 
	
    // write the packet, return the error code
    error=send_packet(ServoID, iWRITE_DATA, TxBuff, 5);

	if(error!=0)
		return (error); 
	
	TxBuff[0] = PUNCH_L;				//Control Starting Address
	TxBuff[1] = byte(Punch&0x00FF);		//punch Lower 8 bits
	TxBuff[2] = byte(Punch>>8); 		//punch Higher 8 bits
	
	// write the packet, return the error code
    error=send_packet(ServoID, iWRITE_DATA, TxBuff, 3);
	
	return(error); 
}

//******************************************************************
//*	SET BAUDRATE
//* 	eg:	SetBaud(1,2,1);	// Turn on torque of servo 2
//******************************************************************
word G15::SetBaudRate(long bps)
{
	byte TxBuff[2];
	word error; 
	

    TxBuff[0] = BAUD_RATE;			//Control Starting Address
	TxBuff[1] = (2000000/bps)-1;	//Calculate baudrate
									//Speed (BPS) = 32M / (16*(n + 1))=2000000/(n+1)
									
	error=send_packet(ServoID, iWRITE_DATA, TxBuff, 2); 
	delay(10); 

    // write the packet, return the error code
    return(error);
}
word AX12::SetBaudRate(long bps)
{
	byte TxBuff[2];
	word error;

    TxBuff[0] = BAUD_RATE;			//Control Starting Address
	TxBuff[1] = (2000000/bps)-1;	//Calculate baudrate
									//Speed (BPS) = 2000000 / (Address4 + 1)
	error=send_packet(ServoID, iWRITE_DATA, TxBuff, 2); 
	delay(10); 
    // write the packet, return the error code
    return(error);
}
//******************************************************************
//*	RESET TO FACTORY SETTINGS
//* 	eg:	FactoryReset(1,1);// Reset servo 1
//******************************************************************
word G15::FactoryReset(void)
{
	byte TxBuff[1];	//dummy byte
	word error; 
	
	error=send_packet(ServoID, iRESET, TxBuff, 0); 
	delay(100); 		//delay for eeprom write

    // write the packet, return the error code
    return(error);
}
//******************************************************************
//*	PING
//* 	eg:	Ping(MAIN,1,dat);	// Ping servo 1, dat is array's pointer
//******************************************************************
word G15::Ping(byte* data)
{
	// write the packet, return the error code
    return(send_packet(ServoID, iPING, data, 0));
}

//******************************************************************
//*	GET CURRENT POSITION
//******************************************************************
word G15::GetPos(byte* data)		
{
    data[0] = PRESENT_POSITION_L;	// Starting Addr where data to be read
    data[1] = 0x02;			// # of bytes to be read
    
    return (send_packet(ServoID, iREAD_DATA, data, 2));
}

word G15::GetSpeed(byte* data)
{	
    data[0] = PRESENT_SPEED_L;	// Starting Addr where data to be read
    data[1] = 0x02;			// # of bytes to be read
    
    return (send_packet(ServoID, iREAD_DATA, data, 2));
}

word G15::GetLoad(byte* data)
{
	
    data[0] = PRESENT_LOAD_L;	// Starting Addr where data to be read
    data[1] = 0x02;			// # of bytes to be read
    
    return (send_packet(ServoID, iREAD_DATA, data, 2));
}

word G15::GetVoltage(byte* data)
{
	data[0] = PRESENT_VOLTAGE;	// Starting Addr where data to be read
    data[1] = 0x01;			// # of bytes to be read
    
    return (send_packet(ServoID, iREAD_DATA, data, 2));
}

word G15::GetTemperature(byte* data)
{
	data[0] = PRESENT_TEMPERATURE;	// Starting Addr where data to be read
    data[1] = 0x01;			// # of bytes to be read
    
    return (send_packet(ServoID, iREAD_DATA, data, 2));
}

//******************************************************************
//*	GET TORQUE (ON/OFF?)
//******************************************************************
word G15::GetTorqueOnOff(byte* data)
{
    data[0] = TORQUE_ENABLE;		// Starting Addr where data to be read
    data[1] = 0x01;					// # of bytes to be read

    return (send_packet(ServoID, iREAD_DATA, data, 2));
}

//******************************************************************
//*	IS MOTOR MOVING?
//******************************************************************
word G15::IsMoving(byte* data)
{
    data[0] = MOVING;				// Starting Addr where data to be read
    data[1] = 0x01;					// # of bytes to be read
    
	return (send_packet(ServoID, iREAD_DATA, data, 2));
}

//******************************************************************
//*	SET ACTION
//* 	eg:	SetAction(1);	// All servo action!
//******************************************************************
void G15::SetAction(void)
{	
	 // byte TxBuff[1];	//dummy byte
	 // send_packet(0xFE, iACTION, TxBuff, 0);	
	set_act(G15CTRL); 

}

void AX12::SetAction(void){

	// byte TxBuff[1];	//dummy byte
	// send_packet(0xFE, iACTION, TxBuff, 0);
	set_act(AX12CTRL); 
}

void set_act(char ctrl){

	int i; 
	byte packet_len = 0;
    byte TxBuff[16];
    byte chksum = 0;	//Check Sum = ~ (ID + Length + Instruction + Parameter1 + ... Parameter N)
 
	digitalWrite(ctrl,TxMode); 

    TxBuff[0] = 0xFF;			//0xFF not included in checksum
    TxBuff[1] = 0xFF;
    TxBuff[2] = 0xFE; 			chksum += TxBuff[2];	//0-254, 0xFE = broadcast id
    TxBuff[3] =  2;	chksum += TxBuff[3];	//INSTRUCTION + PARAMETERS( START ADDR + VALUES ) + CHECKSUM
                                                                                                            //0xFF and ID not included
    TxBuff[4] = iACTION;			chksum += TxBuff[4];	//

    TxBuff[5] = ~chksum; 				//Checksum with Bit Inversion

    packet_len = TxBuff[3] + 4;			//# of bytes for the whole packet
    
	for(i=0; i<packet_len;i++){
		Serial.write(TxBuff[i]);
	}
	Serial.flush();
	//waitTXC();		//arduino version 1.01 only


}