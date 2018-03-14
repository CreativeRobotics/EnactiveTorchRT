//ET-RT Device Class
#define FIRMWARE_VERSION    "0.1.1"

#ifndef _ETRTSystem_H_
#define _ETRTSystem_H_
#endif
#include "ETRT-MPU9250.h"
#include <inttypes.h>
#include <EEPROM.h>
#include <TimerOne.h>
/*
Timer1 used for fast PWM on the LRA pins
Timer1.initialize(40); // 40 us = 25 kHz

*/


//#define DEBUG //comment in for debud data over bluetooth - uses 3%
//#define IMUInitLoop //repeatedly try and init the imu if not working

//Bits for the interrupt on change port (if used)
#define PUSHBUTTON_BIT        1
#define DIGITAL_SENSOR2_BIT   2
#define AUXD2_BIT             3
#define AUXD10_BIT            4
#define AUXD9_BIT             5

//Serial port settings
#define BLUETOOTH_SPEED         38400
#define USB_SPEED         			115200

#define Bluetooth         			Serial1
#define USB               			Serial
#define Debug										Bluetooth //Define which serial port to use for debug messages
#define USB_MIRROR
//Bluetooth Commands;
//buffer locations for command processing
#define MAIN_COMMAND 0
#define SUB_COMMAND  1
//MAIN COMMANDS
#define CMD_GET_DEVICE_INFO							'?'
#define CMD_PRINT_CALIBRATION_DATA			'c'
#define CMD_DATA_STREAM_ON							'D'
#define CMD_DATA_STREAM_OFF							'd'
#define CMD_SET_DATA_STREAM_RAW 				'r'
#define CMD_SET_DATA_STREAM_YPR					'y'
#define CMD_SET_DATA_STREAM_QUATERNION	'q'
//Pin asignments
const uint8_t userLED1_pin			= 17;
const uint8_t userLED2_pin			= 30;
const uint8_t userButton_pin		= 15;
const uint8_t btStatus_pin			= 13;

const uint8_t selectSwitch_pin		= 4;
const uint8_t LRA1PWM_pin			= 10;
const uint8_t LRA2PWM_pin			= 11;
const uint8_t LRA1EN_pin			= 22;
const uint8_t LRA2EN_pin			= 23;
const uint8_t ImuInt_pin			= 7;
const uint8_t audioOut_pin			= 12; //piezo resonator

//Pins for external IO - these are usually left alone for their own driver to handle

const uint8_t digitalSensor0_pin	= 5; //
const uint8_t digitalSensor1_pin	= 16; //PWM
const uint8_t analogSensor0_ch		= 0; //analog chanel
const uint8_t analogSensor1_ch		= 1; //analog chanel
const uint8_t AuxD0_pin				= 6;
const uint8_t AuxD1_pin				= 14;
const uint8_t AuxD2_pin				= 9;
const uint8_t AuxD3_pin				= 8;
const uint8_t AuxA0_ch				= 2; //analog chanel
//Analog Pins
#define BUFFER_LENGTH 128

//Quaternion filter values
// These are the free parameters in the Mahony filter and fusion scheme, Kp
// for proportional feedback, Ki for integral
#define Kp 2.0f * 5.0f
#define Ki 0.0f
//------------------------------------------
enum dataStreamType{
	RawData = 0,
	YPRData,
	QuaternionData
};

enum lraState{
	LraOff = 0,
	LraBraking,
	LraOverdriving,
	LraOn
};
const uint8_t LRA_RESPONSE = 30; //minimum time for LRA response
const uint8_t LRA_OVERDRIVE = 128; //ammount of PWM to overdrive the LRA when starting
const uint16_t LRA_MAX = 1023; //maxumum PWM value
const uint16_t LRA_MIN = 544; //minimum PWM needed to start LRA
static volatile bool imuNewDataflag = 0;  
static volatile uint16_t imuIntTotal = 0;

void IMU_ISR();
//------------------------------------------

class ETRTSystem{
	public:

	ETRTSystem(void)	{setupPins();}
	void initialise();
	void initialiseImu(int32_t gbx, int32_t gby, int32_t gbz, int32_t abx, int32_t aby, int32_t abz);
	void update(); //update the whole system
	void setLed1(bool state);
	void setLed2(bool State);
	
	void Led1On()					{setLed1(1);}
	void Led1Off()				{setLed1(0);}
	void Led2On()					{setLed2(1);}
	void LED2Off()				{setLed2(0);}
	
	bool isButtonPressed()						{return !digitalRead(userButton_pin);}
	bool getSwitchState()						{return digitalRead(selectSwitch_pin);}
	bool getBluetoothState()				{return !digitalRead(btStatus_pin);}
	void beep(int pitch, int duration)	{tone(audioOut_pin, pitch, duration);}
	void enableLra1()								{digitalWrite(LRA1EN_pin, 1);}
	void disableLra1()							{digitalWrite(LRA1EN_pin, 0);}
	void setLra1(uint16_t intensity);
	void enableLra2()								{digitalWrite(LRA2EN_pin, 1);}
	void disableLra2()							{digitalWrite(LRA2EN_pin, 0);}
	void setLra2(uint16_t intensity);
	
	void pulseLra1(int intensity, int milliTime);
	void pulseLra2(int intensity, int milliTime);
	void printIMURawData();
	void printIMUYPRData();
	void printIMUQuaternionData();
	void printCalibrationValues();

	void printDeviceInfo();
	float getQ(uint8_t index);
	
	

	void waitFor(long int delayMillis);//delay for the specified time - keeps updating the system while the delay runs
	MPU9250 imu;
	
	uint8_t dataStreamType = QuaternionData;
	//int printDelay 							= 100;
	bool dataStreamEnable 			= 0;
	int declination 						= 0.0; //-1.28 For creative Robotics	
	uint8_t floatPrecision 			= 3;
	
	bool newData = 0;//flag indicating new imu data
	
	private: //=-----------------------------------------------------------------------------------------------	
	void updateHaptics();
	void setupPins();
	void updateBluetooth(); //update the serial port and process any pending commands
	void printCmdErr();
	void printCmdAck();
	void initialiseIMU();
	//void imuSelfTest();
	bool updateImu();
	void updateQuaternions();
	void updateYPR();
															
	void MahonyQuaternionUpdate(float ax, float ay, float az, float gx, float gy,
                            float gz, float mx, float my, float mz,
                            float deltat);
	//void initialiseI2C();
	bool buttonState 						= 0; //the state of the user button
	bool switchState 						= 0; //the position of the user switch
	bool BTStatus 							= 0;
	bool imuInitialised 				= 0;
	bool magInitialised 				= 0;
	
	//LRA control parameters - used to update haptics in the background so functions return immediatly
	uint16_t lra1Duty = 0, lra2Duty = 0;
	long int lra1Timer = 0, lra2Timer = 0;
	uint8_t lra1State = LraOff, lra2State = LraOff;
	long int nextLra1Update = 0, nextLra2Update = 0;
	long int pulseLength1 = 0, pulseLength2 = 0;
	bool pulse1Active = 0, pulse2Active = 0;
	//Quaternion Filters

	float GyroMeasError 				= PI * (40.0f / 180.0f);
	// gyroscope measurement drift in rad/s/s (start at 0.0 deg/s/s)
	float GyroMeasDrift 				= PI * (0.0f  / 180.0f);

	float beta 									= sqrt(3.0f / 4.0f) * GyroMeasError;   // Compute beta
	// Compute zeta, the other free parameter in the Madgwick scheme usually
	// set to a small or zero value
	float zeta 									= sqrt(3.0f / 4.0f) * GyroMeasDrift;

	// Vector to hold integral error for Mahony method
	float eInt[3] 							= {0.0f, 0.0f, 0.0f};
	// Vector to hold quaternion
	float q[4] 									= {1.0f, 0.0f, 0.0f, 0.0f};
};