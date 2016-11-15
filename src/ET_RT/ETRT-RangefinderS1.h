//ETRT-RangefinderS1
//Driver for the ET-RT Sharp Distance sensor head
//Creative Robotics Ltd 2016

#include <inttypes.h>
#if ARDUINO >= 100
 #include "Arduino.h"
#else
 #include "WProgram.h"
#endif


const uint8_t rangefinder_ch 					= 0; //Analog Chanel
const uint8_t lightSensor_ch  				= 1; //Analog Chanel
const uint8_t LEDTorch_pin  					= 5;
const uint8_t rangefinderEnable_pin  	= 16;

static const int RANGEFINDER_MAX = 710;
#define ENABLE_RANGEFINDER		1
#define DISABLE_RANGEFINDER		0

class ETRTRangefinderS1{
	public:
	ETRTRangefinderS1(void)			{initialise(0);}
	void initialise(bool enableState);
	void update(void);
	
	int getRange() 							{return rangerRaw;}
	int getRangeFiltered() 			{return rangerFilt;}
	
	int getLight() 							{return lightRaw;}
	int getLightFiltered()			{return lightFilt;}
	
	void setLed(int ledValue);
	void LedOn()								{setLed(255);}
	void ledOff()								{setLed(0);}
	void enable()								{setEnable(1);}
	void disable()							{setEnable(0);}
	void setEnable(bool state);
	bool getEnableState() 			{return rangefinderEnableState;}
	
	void setRangeFilterLength(int length) {	rangeFilterLength = constrain(length, 1, 10);}
	void setLightFilterLength(int length) {	lightFilterLength = constrain(length, 1, 10);}
	
	private:
	
	int rangerRaw;
	int rangerFilt;
	int lightRaw;
	int lightFilt;
	int ledIntensity;
	bool rangefinderEnableState;
	
	int rangeFilterLength;
	int lightFilterLength;
	
	
};