#include <ETRT-MPU9250.h>
#include <ETRT-System.h>
#include <ETRT-RangefinderS1.h>

#define Bluetooth Serial1

ETRTRangefinderS1 mySensor;
ETRTSystem et;

//Gyro and accelerometer calibration values - these can be obtained using the calibration sketch
//et01

#define GYRO_X_BIAS -94
#define GYRO_Y_BIAS 28
#define GYRO_Z_BIAS 306
#define ACC_X_BIAS  -94
#define ACC_Y_BIAS  131
#define ACC_Z_BIAS  -529

//et02
/*
#define GYRO_X_BIAS 95
#define GYRO_Y_BIAS -97
#define GYRO_Z_BIAS -22
#define ACC_X_BIAS  95
#define ACC_Y_BIAS  99
#define ACC_Z_BIAS  482
*/

/*/et03
#define GYRO_X_BIAS 646
#define GYRO_Y_BIAS 409
#define GYRO_Z_BIAS -786
#define ACC_X_BIAS  646
#define ACC_Y_BIAS  -164
#define ACC_Z_BIAS  2555
*/


#define GRAVITY 16384

//---------------------------------------

const int delayBetweenPulses = 50;
bool pulseState = 0;
long int nextChange = 0;
const int printDelay = 50; //50ms between data packets
long int nextPrint = 0;
const int dataPrecision = 3; //print floating point data with three deciml places


uint16_t vibIntensity = 0;
uint16_t vibStep = 8;
long int nextPulse = 0;
void setup() {
  // put your setup code here, to run once:
  //initialise the et then the IMU with the gyro and accelerometer calibration values for this device
  et.initialise();
  et.initialiseImu(GYRO_X_BIAS, GYRO_Y_BIAS, GYRO_Z_BIAS, ACC_X_BIAS, ACC_Y_BIAS, ACC_Z_BIAS);
  et.declination = -1.28; //use to get accurate orientation with respect to world coordinates (using the magnetometer).
                          //Use http://www.ngdc.noaa.gov/geomag-web/#declination to get one for wherever you are or ignore to use default zero
  et.beep(2000, 500);
  et.printDeviceInfo();
  //et.setLra1(255);
  delay(500);
  
  //et.setLra1(0);
  mySensor.initialise(1); //initialise with rangefinder enabled
  et.dataStreamType = RawData; //options are RawData, YPRData or QuaternionData
  et.floatPrecision = dataPrecision;
  nextChange = millis();
  nextPrint = millis();
  nextPulse = millis();
}

void loop() {
  et.update();
  mySensor.update();
  if(et.isButtonPressed())  setHaptics();
  else{
    et.setLra1(0);
    et.setLra2(0);
  }
  if(et.dataStreamEnable == true) printData();
  et.setLed2(et.getBluetoothState());
}

void setHaptics(){
  //map the sensor to the haptic actuator
  //If switch is in state 1 use pulses
  //Otherwise use continuous
  if(et.getSwitchState()) updatePulseIntensityMode(); //pulse them on and off at 0.5hz with intensity set by sensor
  else {
    et.setLra1(map(mySensor.getRange(), 0, RANGEFINDER_MAX, 0, LRA_MAX)); //map the sensor (0-RANGEFINDER_MAX) to the haptics (0-LRA_MAX_DUTY)
    et.setLra2(map(mySensor.getRange(), 0, RANGEFINDER_MAX, 0, LRA_MAX)); //map the sensor (0-RANGEFINDER_MAX) to the haptics (0-LRA_MAX_DUTY)
  }
}

//Pulse the haptic actuator at a fixed interval but with an intensity dependent on the range
void updatePulseIntensityMode(){
  if(millis() < nextChange) return; //not enough time has elapsed
  nextChange = millis()+delayBetweenPulses; //set a new elapsed time
  if(pulseState){//pulse is active, so switch off
    et.setLra1(0);
    pulseState = 0;
  }
  else{
    et.setLra1(map(mySensor.getRange(), 0, RANGEFINDER_MAX, 0, LRA_MAX)); //map the sensor (0-RANGEFINDER_MAX) to the haptics (0-LRA_MAX_DUTY)
    pulseState = 1;
  }
}

//Pulse the haptic actuator at a fixed intensity but with a frequency dependent on the range
void updatePulseFrequencyMode(){
   if(millis() < nextPulse) return; //not enough time has elapsed
   int adjustedRange;
   adjustedRange = (RANGEFINDER_MAX-mySensor.getRange())+50; //longer range means longer delay - minimum delay is 50ms
   nextPulse = millis()+adjustedRange;
   et.pulseLra1(LRA_MAX, 50); //this returns straight away and updates in the background
}

void printData(){
   if(millis() < nextPrint) return; //not enough time has passed
   nextPrint = millis()+printDelay; //set the new elapsed time for printing
  //print the IMU data first - these are as comma seperated values, each begins with a character marking the type of data
  if(et.dataStreamType == RawData){
    Bluetooth.write('r');
    et.printIMURawData();
  }else if(et.dataStreamType == YPRData){
    Bluetooth.write('y');
    et.printIMUYPRData();
  }else{
    Bluetooth.write('q');
    et.printIMUQuaternionData();
  }
  Bluetooth.write(','); //comma
  Bluetooth.println( (float)mySensor.getRange(), 1); //cast as float with 1DP and send a line feed/character return for end of packet
}
  
