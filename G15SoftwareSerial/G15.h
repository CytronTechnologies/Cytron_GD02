#ifndef G15_h
#define G15_h

#include "Arduino.h"
#include "SoftwareSerial.h" 


//Arduino Leonardo
#if defined (__AVR_ATmega32U4__)
	#define Serial Serial1
#else 
	#define Serial Serial
#endif

//definitions
//******************************************************************
//*	INSTRUCTIONS
//******************************************************************
#define iPING 		0x01 //obtain a status packet
#define iREAD_DATA	0x02 //read Control Table values
#define iWRITE_DATA	0x03 //write Control Table values
#define iREG_WRITE 	0x04 //write and wait for ACTION instruction
#define iACTION 	0x05 //triggers REG_WRITE instruction
#define iRESET 		0x06 //set factory defaults
#define iSYNC_WRITE     0x83 //simultaneously control multiple actuators

#define SerialTimeOut 100L 
#define TxMode LOW		//Master Transmit to G15
#define RxMode HIGH		//Master Receive from G15
#define ConvertAngle2Pos(Angle) word(word(Angle)*1088UL/360UL)
#define ConvertPos2Angle(Pos) float(Pos)*360.0/1088.0
#define ConvertTime(Time) word(Time*10UL)
#define CW 1
#define CCW 0
#define ON 1
#define OFF 0
//Alarm Mask	1111 1111
#define ALARM_INST			0x40		
#define ALARM_OVERLOAD		0x20
#define ALARM_CHECKSUM		0x10
#define ALARM_RANGE			0x08
#define ALARM_OVERHEAT		0x04
#define ALARM_ANGLELIMIT 	0x02
#define ALARM_VOLTAGE		0x01

enum{
			  MODEL_NUMBER_L, 		// 0x00
			  MODEL_NUMBER_H, 		// 0x01
			  VERSION, 			// 0x02
			  ID, 				// 0x03
			  BAUD_RATE, 			// 0x04
			  RETURN_DELAY_TIME, 		// 0x05
			  CW_ANGLE_LIMIT_L, 		// 0x06
			  CW_ANGLE_LIMIT_H, 		// 0x07
			  CCW_ANGLE_LIMIT_L, 		// 0x08
			  CCW_ANGLE_LIMIT_H, 		// 0x09
			  RESERVED1, 			// 0x0A
			  LIMIT_TEMPERATURE, 		// 0x0B
			  DOWN_LIMIT_VOLTAGE, 		// 0x0C
			  UP_LIMIT_VOLTAGE, 		// 0x0D
			  MAX_TORQUE_L, 		// 0x0E
			  MAX_TORQUE_H, 		// 0x0F
			  STATUS_RETURN_LEVEL, 		// 0x10	
			  ALARM_LED, 			// 0x11
			  ALARM_SHUTDOWN, 		// 0x12
			  RESERVED2, 			// 0x13
			  DOWN_CALIBRATION_L, 		// 0x14
			  DOWN_CALIBRATION_H, 		// 0x15
			  UP_CALIBRATION_L, 		// 0x16
			  UP_CALIBRATION_H, 		// 0x17
			  TORQUE_ENABLE, 		// 0x18
			  LED, 				// 0x19
			  CW_COMPLIANCE_MARGIN, 	// 0x1A
			  CCW_COMPLIANCE_MARGIN, 	// 0x1B
			  CW_COMPLIANCE_SLOPE, 		// 0x1C
			  CCW_COMPLIANCE_SLOPE, 	// 0x1D
			  GOAL_POSITION_L, 		// 0x1E 
			  GOAL_POSITION_H, 		// 0x1F 
			  MOVING_SPEED_L, 		// 0x20 
			  MOVING_SPEED_H,		// 0x21 
			  TORQUE_LIMIT_L, 		// 0x22 
			  TORQUE_LIMIT_H, 		// 0x23 
			  PRESENT_POSITION_L, 		// 0x24 
			  PRESENT_POSITION_H, 		// 0x25 
			  PRESENT_SPEED_L, 		// 0x26 
			  PRESENT_SPEED_H, 		// 0x27 
			  PRESENT_LOAD_L, 		// 0x28 
			  PRESENT_LOAD_H, 		// 0x29 
			  PRESENT_VOLTAGE, 		// 0x2A 
			  PRESENT_TEMPERATURE, 		// 0x2B 
			  REGISTERED_INSTRUCTION, 	// 0x2C 
			  RESERVE3, 			// 0x2D 
			  MOVING, 			// 0x2E 
			  LOCK, 			// 0x2F 
			  PUNCH_L, 			// 0x30
			  PUNCH_H			// 0x31
			};	
	

//function protos	
void G15DriverInit(long baudrate, byte rx, byte tx, char G15_CTRL); 
void set_act(char ctrl);  

//class, Object and Variable 


class G15
{
	public:		
		byte ServoID;
		
		G15(byte ID) ; //, char ctrl);
		void init(void);
		
		//*=========Wheel Mode=====================================================================================
		//360 degree continous rotation. change CW and CCW Angle Limits to same value
		word SetWheelMode(void);
		word ExitWheelMode(void); 
		word SetWheelSpeed(word Speed, byte CW_CCW);

		
		//*=========Normal Positioning Mode========================================================================
		//(Rotation limited by Angle Limit and Direction of Rotation determined by operation section of Angle Limit)
		word SetPos(word Position, byte Write_Reg);
		word SetPosSpeed(word Position, word Speed, byte Write_Reg);
		//byte SetPosAngle(word Angle, byte Write_Reg); 	//replaced with ConvertAngle()
		
		//*========Direction Positioning Mode======================================================================
		//(Rotation direction and angle is NOT limited by Angle Limit Control Register value)
		word RotateCW (word Position, byte Write_Reg); 
		word RotateCCW (word Position, byte Write_Reg); 
		
		//*=======Torque Enable and Speed Control==================================================================
		word SetTorqueOnOff(byte on_off, byte Write_Reg);
		word SetSpeed(word Speed, byte Write_Reg);
		word SetTimetoGoal(word Time,byte Write_Reg); 
		
		//*=======Set Maximum Limits===============================================================================
		word SetAngleLimit(word CW_angle, word CCW_angle);
		word SetTorqueLimit(word TorqueLimit); //in RAM area
		word SetTemperatureLimit(byte Temperature);
		word SetVoltageLimit(byte VoltageLow, byte VoltageHigh); 
		
		word SetID(byte NewID); 	
		
		word SetLED(byte on_off, byte Write_Reg);
		word SetAlarmLED(byte AlarmLED); 
		word SetAlarmShutDown(byte Alarm); 
		
		
		//*========Servo Positioning Control Parameters============================================================
		word SetMarginSlopePunch(byte CWMargin, byte CCWMargin, byte CWSlope, byte CCWSlope, word Punch);
			
		
		
		word SetBaudRate(long bps);
		
		word FactoryReset(void); 
		
		word Ping(byte* data); 
		
		word GetPos(byte* data);
		word GetSpeed(byte* data);
		word GetLoad(byte* data); 
		word GetVoltage(byte* data); 
		word GetTemperature(byte* data);
		word GetTorqueOnOff(byte* data);		
		word IsMoving(byte* data);
		
		static void SetAction(void);
		
		
		
		
	protected:
		
		char TxRx;				
		void setRX(void);
		void setTX(void);
		word send_packet(byte ID, byte inst, byte* data, byte param_len);	
		//byte read_data(byte id, byte* data); 
		

};

#endif
