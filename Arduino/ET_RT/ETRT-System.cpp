
#include <ETRT-System.h>

void IMU_ISR(){
  imuNewDataflag = 1;
  imuIntTotal++;
}


//#include "Arduino.h"

void ETRTSystem::initialise(){
	//init pins
	setupPins();
	Wire.begin();
	Wire.setClock(400000L);
	Serial1.begin(BLUETOOTH_SPEED);
	Serial.begin(USB_SPEED);
	Timer1.initialize(40); // 40 us = 25 kHz
	Timer1.pwm(LRA1PWM_pin, 0);
	Timer1.pwm(LRA2PWM_pin, 0);
	delay(100);
	printDeviceInfo();
}

void  ETRTSystem::initialiseImu(int32_t gbx, int32_t gby, int32_t gbz, int32_t abx, int32_t aby, int32_t abz){
	imu.gyro_bias[0] = gbx;
	imu.gyro_bias[1] = gby;
	imu.gyro_bias[2] = gbz;
	imu.accel_bias[0] = abx;
	imu.accel_bias[1] = aby;
	imu.accel_bias[2] = abz;
	
	initialiseIMU();
	#ifdef IMUInitLoop
	while(!imuInitialised || !magInitialised){
		delay(500);
		initialiseIMU();
	}
	#endif
	attachInterrupt(digitalPinToInterrupt(ImuInt_pin), IMU_ISR, RISING);
}



void ETRTSystem::setupPins(){
	pinMode(selectSwitch_pin, INPUT_PULLUP);
	pinMode(userButton_pin, INPUT_PULLUP);
	pinMode(AuxD0_pin, INPUT_PULLUP);
	pinMode(AuxD1_pin, INPUT_PULLUP);
	pinMode(AuxD2_pin, INPUT_PULLUP);
	pinMode(AuxD3_pin, INPUT_PULLUP);
	pinMode(btStatus_pin, INPUT);
	pinMode(audioOut_pin, OUTPUT);

	pinMode(LRA1EN_pin, OUTPUT); 
	pinMode(LRA2EN_pin, OUTPUT); 
	//pinMode(LRA1PWM_pin, OUTPUT); 
	//pinMode(LRA2PWM_pin, OUTPUT);
}
//-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_
void ETRTSystem::update(){
	updateBluetooth();
	updateHaptics();
	if(updateImu()){
		updateQuaternions();
		updateYPR();
		newData = 1; //this can be checked and cleared by the user sketch
	}
}
//-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_
void ETRTSystem::setLed1(bool state){
	if(state == 1) RXLED0;//digitalWrite(USERLED_1, 0);
	else RXLED1;//digitalWrite(USERLED_1, 1);
}
//-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_

void ETRTSystem::setLed2(bool state){
	if(state == 1) TXLED0;//digitalWrite(USERLED_2, 0);
	else TXLED1;//digitalWrite(USERLED_2, 1);
}
//-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-
void  ETRTSystem::setLra1(uint16_t intensity){
	if(millis() < nextLra1Update) return; //prevent updates happening too fast
	nextLra1Update = millis()+LRA_RESPONSE; //set the next minimum time between updates
	//Map intensity (0-1023) to viable WM (512-1023)
	lra1Duty = map(intensity, 0, 1023, LRA_MIN, LRA_MAX);//(intensity/2)+LRA_MIN;
	/*if(lra1Duty == LRA_MIN){
		Timer1.setPwmDuty(LRA1PWM_pin, 0);
		disableLra1();
	}
	else{
		enableLra1();
		Timer1.setPwmDuty(LRA1PWM_pin, constrain(lra1Duty, 0, LRA_MAX));
	}
	return;*/
	
	//activate brake if intensity is zero, or exit of lra is already off
	if(lra1State == LraOff && lra1Duty == LRA_MIN) return; //do nothing if everything is off
	else if(lra1Duty == LRA_MIN){
		Timer1.setPwmDuty(LRA1PWM_pin, 0); //zero PWM
		lra1State = LraBraking; ////set state to braking
	}
	else{
		switch(lra1State){
			case LraOff: //enable the device then set the duty cycle with overdrive and change the state
				enableLra1();
				Timer1.setPwmDuty(LRA1PWM_pin, constrain(lra1Duty+LRA_OVERDRIVE, 0, LRA_MAX));
				lra1State = LraOverdriving;
				break;
			case LraBraking: //switch back to overdrive - this should never happen because updates should always happen less frequently than braking period
				Timer1.setPwmDuty(LRA1PWM_pin, constrain(lra1Duty+LRA_OVERDRIVE, 0, LRA_MAX));
				lra1State = LraOverdriving;
				break;
			case LraOverdriving: //change the duty but keep overdriving unless zero - this shoud never happen because the update frequency should always be less than the time spent overdriving
				Timer1.setPwmDuty(LRA1PWM_pin, constrain(lra1Duty+LRA_OVERDRIVE, 0, LRA_MAX));
				break;
			case LraOn: //just change the duty cycle
				Timer1.setPwmDuty(LRA1PWM_pin, constrain(lra1Duty, 0, LRA_MAX));
				break;
		}
	}
	//set the timeout variable
	lra1Timer = millis()+LRA_RESPONSE;
}
//-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_
void  ETRTSystem::setLra2(uint16_t intensity){
	if(millis() < nextLra2Update) return; //prevent updates happening too fast
	nextLra2Update = millis()+LRA_RESPONSE; //set the next minimum time between updates
	//activate brake if intensity is zero, or exit of lra is already off
	lra2Duty = intensity;
	if(lra2State == LraOff && lra2Duty == 0) return; //do nothing if everything is off
	else if(lra2Duty == 0){
		Timer1.setPwmDuty(LRA2PWM_pin, 0); //zero PWM
		lra2State = LraBraking; ////set state to braking
	}
	else{
		switch(lra2State){
			case LraOff: //do nothing if new value is zero
				//enable the device then set the duty cycle with overdrive and change the state
				enableLra2();
				Timer1.setPwmDuty(LRA2PWM_pin, constrain(lra2Duty+LRA_OVERDRIVE, 0, LRA_MAX));
				lra2State = LraOverdriving;
				break;
			case LraBraking: //switch back to overdrive - this should never happen because updates should always happen less frequently than braking period
				Timer1.setPwmDuty(LRA2PWM_pin, constrain(lra2Duty+LRA_OVERDRIVE, 0, LRA_MAX));
				lra2State = LraOverdriving;
				break;
			case LraOverdriving: //change the duty but keep overdriving unless zero - this shoud never happen because the update frequency should always be less than the time spent overdriving
				Timer1.setPwmDuty(LRA2PWM_pin, constrain(lra2Duty+LRA_OVERDRIVE, 0, LRA_MAX));
				break;
			case LraOn: //just change the duty cycle
				Timer1.setPwmDuty(LRA2PWM_pin, constrain(lra2Duty, 0, LRA_MAX));
				break;
		}
	}
	//set the timeout variable
	lra2Timer = millis()+LRA_RESPONSE;
}


void ETRTSystem::pulseLra1(int intensity, int milliTime){
	pulseLength1 = millis()+milliTime;
	pulse1Active = 1;
	setLra1(intensity);
}
void ETRTSystem::pulseLra2(int intensity, int milliTime){
	pulseLength2 = millis()+milliTime;
	pulse2Active = 1;
	setLra2(intensity);
}
//Update the haptic devices so they can be set and run in the background
void   ETRTSystem::updateHaptics(){
	//If the haptic is set to zero then the device must have a braking period of 30ms
	//If it is set to non zero then it has an overdrive period of 30ms
	//Check to see if a pulse command was sent - disable if pulse period has ended
	if(pulse1Active && millis() > pulseLength1){
		//end the pulse1Active
		setLra1(0);
		pulse1Active = 0;
	}
	if(pulse2Active && millis() > pulseLength2){
		//end the pulse1Active
		setLra2(0);
		pulse2Active = 0;
	}
	//if the current time has gone past the time when we need to update, then update
	if(lra1Timer < millis()){
		switch(lra1State){
			lra1Timer = millis()+LRA_RESPONSE; //reset the timer
			case LraBraking: //end braking period, disable the LRA
				//Bluetooth.println("BrakeEnd");
				Timer1.setPwmDuty(LRA1PWM_pin, 0);
				disableLra1();
				lra1State = LraOff;
				break;
			case LraOverdriving: //end overdrive period, set to the desired duty
				//Bluetooth.println("ODEnd");
				Timer1.setPwmDuty(LRA1PWM_pin, lra1Duty);
				lra1State = LraOn;
				break;
		}
	}
	if(lra2Timer < millis()){
		lra2Timer = millis()+LRA_RESPONSE;//reset the timer
		switch(lra2State){
			case LraBraking: //end braking period, disable the LRA
				Timer1.setPwmDuty(LRA2PWM_pin, 0);
				disableLra2();
				lra2State = LraOff;
				break;
			case LraOverdriving: //end overdrive period, set to the desired duty
				Timer1.setPwmDuty(LRA2PWM_pin, lra2Duty);
				lra2State = LraOn;
				break;
		}
	}
}

//-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_
//Print all the raw data from the IMU as comma seperated values
//Terminte line adds a line feed at the end
void ETRTSystem::printIMURawData(){
	//float ax, ay, az, gx, gy, gz, mx, my, mz;
	/*Bluetooth.print(imu.ax- -0.142531914893617, floatPrecision); Bluetooth.write(',');
	Bluetooth.print(imu.ay- -0.136396276595745, floatPrecision); Bluetooth.write(',');
	Bluetooth.print(imu.az- -0.158966480446927, floatPrecision); Bluetooth.write(',');
	
	Bluetooth.print(imu.gx- -0.047470744680851, floatPrecision); Bluetooth.write(',');
	Bluetooth.print(imu.gy- 0.175768617021277, floatPrecision); Bluetooth.write(',');
	Bluetooth.print(imu.gz- 0.163659574468085, floatPrecision); Bluetooth.write(',');
	*/
	Bluetooth.print(imu.ax, floatPrecision); Bluetooth.write(',');
	Bluetooth.print(imu.ay, floatPrecision); Bluetooth.write(',');
	Bluetooth.print(imu.az, floatPrecision); Bluetooth.write(',');
	
	Bluetooth.print(imu.gx, floatPrecision); Bluetooth.write(',');
	Bluetooth.print(imu.gy, floatPrecision); Bluetooth.write(',');
	Bluetooth.print(imu.gz, floatPrecision); Bluetooth.write(',');
	
	Bluetooth.print(imu.mx, floatPrecision); Bluetooth.write(',');
	Bluetooth.print(imu.my, floatPrecision); Bluetooth.write(',');
	Bluetooth.print(imu.mz, floatPrecision);
}

//-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_
//Print all the ypr data from the IMU as comma seperated values
//Terminte line adds a line feed at the end
void ETRTSystem::printIMUYPRData(){
	Bluetooth.print(imu.yaw, floatPrecision); 		Bluetooth.write(',');
	Bluetooth.print(imu.pitch, floatPrecision); 	Bluetooth.write(',');
	Bluetooth.print(imu.roll, floatPrecision);
}

//-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_
//Print the four quaternion values from the IMU as comma seperated values
//Terminte line adds a line feed at the end
void ETRTSystem::printIMUQuaternionData(){
	Bluetooth.print(q[0], floatPrecision); 	Bluetooth.write(',');
	Bluetooth.print(q[1], floatPrecision); 	Bluetooth.write(',');
	Bluetooth.print(q[2], floatPrecision); 	Bluetooth.write(',');
	Bluetooth.print(q[3], floatPrecision);
}
//-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_


void ETRTSystem::printDeviceInfo(){
	Bluetooth.print(F("Enactive Torch RT Version "));
	Bluetooth.println(F(FIRMWARE_VERSION));
	if(imuInitialised) Bluetooth.println(F("IMU Online"));
	else							 Bluetooth.println(F("IMU Err"));
	
	if(magInitialised) Bluetooth.println(F("MAG Online"));
	else							 Bluetooth.println(F("MAG Err"));
	
	if(dataStreamEnable) Bluetooth.println(F("Datastream ON"));
	else							 Bluetooth.println(F("Datastream OFF"));
	
	if(isButtonPressed()) Bluetooth.println(F("Button ON"));
	else							 Bluetooth.println(F("Button OFF"));
	
	if(getSwitchState()) Bluetooth.println(F("Switch ON"));
	else							 Bluetooth.println(F("Switch OFF"));
}

//-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_

void  ETRTSystem::printCalibrationValues(){
	Bluetooth.print("Gyro XYZ=");
	for(int n = 0; n < 3; n++){
		Bluetooth.print(imu.gyro_bias[n]);
		Bluetooth.write(',');
	}
	Bluetooth.print(" Acc XYZ=");
	for(int n = 0; n < 3; n++){
		Bluetooth.print(imu.accel_bias[n]);
		Bluetooth.write(',');
	}
}
//-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_

void ETRTSystem::waitFor(long int delayMillis){
	long int end = millis()+delayMillis;
	while(end > millis()){
		//update the system
		update();
		delay(1);
	}
}
//-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_
//check the serial buffer for new chars and see if they match any commands
void ETRTSystem::updateBluetooth(){
	char inByte;
	while(Bluetooth.available()){
		inByte = Bluetooth.read();
		#ifdef USB_MIRROR
		if(USB) USB.write(inByte);
		#endif
		switch(inByte){
			case CMD_GET_DEVICE_INFO:
				printDeviceInfo();
				break;
			case CMD_PRINT_CALIBRATION_DATA:
				printCalibrationValues();
				break;
			case CMD_DATA_STREAM_ON:
				dataStreamEnable = 1;
				printCmdAck();
				break;
			case CMD_DATA_STREAM_OFF:
				dataStreamEnable = 0;
				printCmdAck();
				break;
			case CMD_SET_DATA_STREAM_RAW:
				dataStreamType = RawData;
				printCmdAck();
				break;
			case CMD_SET_DATA_STREAM_YPR:
				dataStreamType = YPRData;
				printCmdAck();
				break;
			case CMD_SET_DATA_STREAM_QUATERNION:
				dataStreamType = QuaternionData;
				printCmdAck();
				break;
			default:
				break;
		}
	}
}

//-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_
void ETRTSystem::printCmdErr(){
		Bluetooth.write('\n');
	Bluetooth.println(F("ERR"));
}
//-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_
void ETRTSystem::printCmdAck(){
	Bluetooth.write('\n');
	Bluetooth.println(F("AOK"));
}

//-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_
void ETRTSystem::initialiseIMU(){
	
	
  // Read the WHO_AM_I register, this is a good test of communication
	byte c = imu.readByte(MPU9250_ADDRESS, WHO_AM_I_MPU9250);

	if (c != 0x71){ // WHO_AM_I should always be 0x71
		imuInitialised = 0;
		#ifdef DEBUG 
			Debug.print(F("Could not connect to MPU9250: 0x"));
			Debug.println(c, HEX);
		#endif
	}
	else if(c == 0x71) {
		imuInitialised = 1;
		#ifdef DEBUG 
			Debug.println("MPU9250 is online...");
		#endif

		// Start by performing self test and reporting values
		#ifdef DEBUG 
			//this adds around 7% to code size
			imu.MPU9250SelfTest(imu.SelfTest);
		#endif
		//Assume that calibration has already happened so apply offsets to the imu.
		//imu.applyOffsets();
		// Calibrate gyro and accelerometers, load biases in bias registers
		//imu.calibrateMPU9250(imu.gyroBias, imu.accelBias);

		imu.initMPU9250();
		// Initialize device for active mode read of acclerometer, gyroscope, and
		// temperature
		#ifdef DEBUG 
			Debug.println("MPU9250 initialized for active data mode....");
		#endif
	}

	// Read the WHO_AM_I register of the magnetometer, this is a good test of
	// communication
	byte d = imu.readByte(AK8963_ADDRESS, WHO_AM_I_AK8963);
	if(d != 0x48){
		magInitialised = 0;
		#ifdef DEBUG 
			Debug.print(F("Could not connect to Magnetometer: "));
			Debug.println(d, HEX);
		#endif
	}
	else if(d == 0x48){
		magInitialised = 1;
		// Get magnetometer calibration from AK8963 ROM
		imu.initAK8963(imu.magCalibration);
		// Initialize device for active mode read of magnetometer
		#ifdef DEBUG
			Debug.println("Magnetometer is online...");
			Debug.println("AK8963 initialized for active data mode....");
			Debug.println("Calibration values: ");
			Debug.print("X-Axis sensitivity adjustment value ");
			Debug.println(imu.magCalibration[0], 2);
			Debug.print("Y-Axis sensitivity adjustment value ");
			Debug.println(imu.magCalibration[1], 2);
			Debug.print("Z-Axis sensitivity adjustment value ");
			Debug.println(imu.magCalibration[2], 2);
		#endif
	}
}


//-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_

bool ETRTSystem::updateImu(){
	//if (imu.readByte(MPU9250_ADDRESS, INT_STATUS) & 0x01 && imuNewDataflag)
	if (imu.readByte(MPU9250_ADDRESS, INT_STATUS) & 0x01)
  {  
    imu.readAccelData(imu.accelCount);  // Read the x/y/z adc values
		//imu.accelCount[0]-=imu.accel_bias[0];
		//imu.accelCount[1]-=imu.accel_bias[1];
		//imu.accelCount[2]-=imu.accel_bias[2];
    imu.getAres();

    // Now we'll calculate the accleration value into actual g's
    // This depends on scale being set
    imu.ax = (float)(imu.accelCount[0]-imu.accel_bias[0])*imu.aRes; // - accelBias[0];
    imu.ay = (float)(imu.accelCount[1]-imu.accel_bias[1])*imu.aRes; // - accelBias[1];
    imu.az = (float)(imu.accelCount[2]-imu.accel_bias[2])*imu.aRes; // - accelBias[2];

    imu.readGyroData(imu.gyroCount);  // Read the x/y/z adc values
		//imu.gyroCount[0]-=imu.gyro_bias[0];
		//imu.gyroCount[1]-=imu.gyro_bias[1];
		//imu.gyroCount[2]-=imu.gyro_bias[2];
    imu.getGres();

    // Calculate the gyro value into actual degrees per second
    // This depends on scale being set
    imu.gx = (float)(imu.gyroCount[0]-imu.gyro_bias[0])*imu.gRes;
    imu.gy = (float)(imu.gyroCount[1]-imu.gyro_bias[1])*imu.gRes;
    imu.gz = (float)(imu.gyroCount[2]-imu.gyro_bias[2])*imu.gRes;

    imu.readMagData(imu.magCount);  // Read the x/y/z adc values
    imu.getMres();
    // User environmental x-axis correction in milliGauss, should be
    // automatically calculated
    imu.magbias[0] = +470.;
    // User environmental x-axis correction in milliGauss TODO axis??
    imu.magbias[1] = +120.;
    // User environmental x-axis correction in milliGauss
    imu.magbias[2] = +125.;

    // Calculate the magnetometer values in milliGauss
    // Include factory calibration per data sheet and user environmental
    // corrections
    // Get actual magnetometer value, this depends on scale being set
    imu.mx = (float)imu.magCount[0]*imu.mRes*imu.magCalibration[0] -
               imu.magbias[0];
    imu.my = (float)imu.magCount[1]*imu.mRes*imu.magCalibration[1] -
               imu.magbias[1];
    imu.mz = (float)imu.magCount[2]*imu.mRes*imu.magCalibration[2] -
               imu.magbias[2];
		imuNewDataflag = 0;
		return 1;
	}
	return 0;
}
//-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_

void ETRTSystem::updateQuaternions(){
	imu.updateTime();
	MahonyQuaternionUpdate(imu.ax, imu.ay, imu.az, imu.gx*DEG_TO_RAD,
																imu.gy*DEG_TO_RAD, imu.gz*DEG_TO_RAD, imu.my,
																imu.mx, imu.mz, imu.deltat);

}
//-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_

void  ETRTSystem::updateYPR(){
	//No need to use getQ and Q is now member data
		imu.yaw  = atan2(2.0f * 	(q[1] * q[2] + q[0] * q[3]),
															 q[0] * q[0] + q[1] * q[1] - q[2] * q[2] - getQ(3) * getQ(3));
												
		imu.pitch = -asin(2.0f * 	(q[1] * q[3] - q[0] * q[2]));
												
		imu.roll  = atan2(2.0f * 	(q[0] * q[1] + q[2] * q[3]),
															 q[0] * q[0] - q[1] * q[1] - q[2] * q[2] + q[3] * q[3]);
		imu.pitch *= RAD_TO_DEG;
		imu.yaw   *= RAD_TO_DEG;

		// - http://www.ngdc.noaa.gov/geomag-web/#declination
		//Ceative Robotics lat 52.199473 long -1.737919 Decination = -1.28
		imu.yaw   += declination; //1.28;
		imu.roll  *= RAD_TO_DEG;
}
//-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_

void ETRTSystem::MahonyQuaternionUpdate(float ax, float ay, float az, float gx, float gy, float gz, float mx, float my, float mz, float deltat)
{
  // short name local variable for readability
	//This uses 12% of flash memory on the 32u4
  float q1 = q[0], q2 = q[1], q3 = q[2], q4 = q[3];
  float norm;
  float hx, hy, bx, bz;
  float vx, vy, vz, wx, wy, wz;
  float ex, ey, ez;
  float pa, pb, pc;

  // Auxiliary variables to avoid repeated arithmetic
  float q1q1 = q1 * q1;
  float q1q2 = q1 * q2;
  float q1q3 = q1 * q3;
  float q1q4 = q1 * q4;
  float q2q2 = q2 * q2;
  float q2q3 = q2 * q3;
  float q2q4 = q2 * q4;
  float q3q3 = q3 * q3;
  float q3q4 = q3 * q4;
  float q4q4 = q4 * q4;

  // Normalise accelerometer measurement
  norm = sqrt(ax * ax + ay * ay + az * az);
  if (norm == 0.0f) return; // Handle NaN
  norm = 1.0f / norm;       // Use reciprocal for division
  ax *= norm;
  ay *= norm;
  az *= norm;

  // Normalise magnetometer measurement
  norm = sqrt(mx * mx + my * my + mz * mz);
  if (norm == 0.0f) return; // Handle NaN
  norm = 1.0f / norm;       // Use reciprocal for division
  mx *= norm;
  my *= norm;
  mz *= norm;

  // Reference direction of Earth's magnetic field
  hx = 2.0f * mx * (0.5f - q3q3 - q4q4) + 2.0f * my * (q2q3 - q1q4) + 2.0f * mz * (q2q4 + q1q3);
  hy = 2.0f * mx * (q2q3 + q1q4) + 2.0f * my * (0.5f - q2q2 - q4q4) + 2.0f * mz * (q3q4 - q1q2);
  bx = sqrt((hx * hx) + (hy * hy));
  bz = 2.0f * mx * (q2q4 - q1q3) + 2.0f * my * (q3q4 + q1q2) + 2.0f * mz * (0.5f - q2q2 - q3q3);

  // Estimated direction of gravity and magnetic field
  vx = 2.0f * (q2q4 - q1q3);
  vy = 2.0f * (q1q2 + q3q4);
  vz = q1q1 - q2q2 - q3q3 + q4q4;
  wx = 2.0f * bx * (0.5f - q3q3 - q4q4) + 2.0f * bz * (q2q4 - q1q3);
  wy = 2.0f * bx * (q2q3 - q1q4) + 2.0f * bz * (q1q2 + q3q4);
  wz = 2.0f * bx * (q1q3 + q2q4) + 2.0f * bz * (0.5f - q2q2 - q3q3);

  // Error is cross product between estimated direction and measured direction of gravity
  ex = (ay * vz - az * vy) + (my * wz - mz * wy);
  ey = (az * vx - ax * vz) + (mz * wx - mx * wz);
  ez = (ax * vy - ay * vx) + (mx * wy - my * wx);
  if (Ki > 0.0f)
  {
    eInt[0] += ex;      // accumulate integral error
    eInt[1] += ey;
    eInt[2] += ez;
  }
  else
  {
    eInt[0] = 0.0f;     // prevent integral wind up
    eInt[1] = 0.0f;
    eInt[2] = 0.0f;
  }

  // Apply feedback terms
  gx = gx + Kp * ex + Ki * eInt[0];
  gy = gy + Kp * ey + Ki * eInt[1];
  gz = gz + Kp * ez + Ki * eInt[2];
 
  // Integrate rate of change of quaternion
  pa = q2;
  pb = q3;
  pc = q4;
  q1 = q1 + (-q2 * gx - q3 * gy - q4 * gz) * (0.5f * deltat);
  q2 = pa + (q1 * gx + pb * gz - pc * gy) * (0.5f * deltat);
  q3 = pb + (q1 * gy - pa * gz + pc * gx) * (0.5f * deltat);
  q4 = pc + (q1 * gz + pa * gy - pb * gx) * (0.5f * deltat);

  // Normalise quaternion
  norm = sqrt(q1 * q1 + q2 * q2 + q3 * q3 + q4 * q4);
  norm = 1.0f / norm;
  q[0] = q1 * norm;
  q[1] = q2 * norm;
  q[2] = q3 * norm;
  q[3] = q4 * norm;
}
//-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_


float ETRTSystem::getQ(uint8_t index){
	if(index > 4) return 0;
	return q[index];
}
//-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_
