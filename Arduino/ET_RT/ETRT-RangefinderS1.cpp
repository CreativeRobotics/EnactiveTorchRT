//ETRT-RangefinderS1
//Driver for the ET-RT Sharp Distance sensor head

#include <ETRT-RangefinderS1.h>
//#include "Arduino.h"
void ETRTRangefinderS1::initialise(bool enableState){
	rangerRaw 			= 0;
	rangerFilt 			= 0;
	lightRaw 			= 0;
	lightFilt 			= 0;
	ledIntensity 		= 0;
	rangeFilterLength 	= 0;
	lightFilterLength 	= 0;

	pinMode(LEDTorch_pin, OUTPUT);
	pinMode(rangefinderEnable_pin, OUTPUT);
	setEnable(enableState);
	setLed(ledIntensity);
}



void ETRTRangefinderS1::update(void){
	rangerRaw = analogRead(rangefinder_ch);
	lightRaw = analogRead(lightSensor_ch);
	//Very simple running average filter
	rangerFilt = ( ( rangerFilt * rangeFilterLength ) + rangerRaw ) / ( rangeFilterLength + 1 );
	lightFilt = ( ( lightFilt * lightFilterLength ) + lightRaw ) / ( lightFilterLength + 1 );
}


void ETRTRangefinderS1::setEnable(bool state){
	rangefinderEnableState = state;
	digitalWrite(rangefinderEnable_pin, rangefinderEnableState);
}
void ETRTRangefinderS1::setLed(int ledValue){
	ledIntensity = constrain(ledValue, 0, 255);
	analogWrite(LEDTorch_pin, ledIntensity);
}
