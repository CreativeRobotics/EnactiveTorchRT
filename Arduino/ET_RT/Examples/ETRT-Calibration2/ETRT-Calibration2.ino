#include <ETRT-MPU9250.h>
#include <ETRT-System.h>
#include <ETRT-RangefinderS1.h>

#define Bluetooth Serial1
//ETRTRangefinderS1 mySensor;
ETRTSystem et;

//Gyro and accelerometer calibration values - these can be obtained using the calibration sketch
/*
 * //et02
#define GYRO_X_BIAS 1
#define GYRO_Y_BIAS 32
#define GYRO_Z_BIAS -4
#define ACC_X_BIAS  1
#define ACC_Y_BIAS  112
#define ACC_Z_BIAS  474
*/

/*
 * et03
#define GYRO_X_BIAS 646
#define GYRO_Y_BIAS 409
#define GYRO_Z_BIAS -786
#define ACC_X_BIAS  646
#define ACC_Y_BIAS  -164
#define ACC_Z_BIAS  2555
 */

#define GYRO_X_BIAS 0
#define GYRO_Y_BIAS 0
#define GYRO_Z_BIAS 0
#define ACC_X_BIAS 0
#define ACC_Y_BIAS 0
#define ACC_Z_BIAS 0
#define GRAVITY 16384
int counter = 600;
long int lastTime = 0;

static int noOfSamples = 2000;
int32_t gxSum = 0, gySum = 0, gzSum = 0, axSum = 0, aySum = 0, azSum = 0;
//---------------------------------------


int loopCount = 0;
void setup() {
  // put your setup code here, to run once:
  //initialise the et with the gyro and accelerometer calibration values for this device
  attachInterrupt(digitalPinToInterrupt(ImuInt_pin), IMU_ISR, RISING);
  //et.initialise(GYRO_X_BIAS, GYRO_Y_BIAS, GYRO_Z_BIAS, ACC_X_BIAS, ACC_Y_BIAS, ACC_Z_BIAS);
  et.initialise();
  et.beep(2000, 500);
  //et.setLra1(255);
  delay(500);
  
  //et.setLra1(0);
  //mySensor.initialise(1); //initialise with rangefinder enabled
  et.dataStreamType = RawData;
}

void loop() {
  //loop 2000 times and print data to the terminal
  //use this to compute calibration values
  Bluetooth.println("Send a char to calibrate");
  waitForSerial();
  Bluetooth.println("Starting Calibration . . .");
  et.waitFor(500);

  et.imu.calibrateMPU9250();

  Bluetooth.println("Calibration Values are: ");
  Bluetooth.print("#define GYRO_X_BIAS "); Bluetooth.println(et.imu.gyro_bias[0]);
  Bluetooth.print("#define GYRO_Y_BIAS "); Bluetooth.println(et.imu.gyro_bias[1]);
  Bluetooth.print("#define GYRO_Z_BIAS "); Bluetooth.println(et.imu.gyro_bias[2]);
  Bluetooth.print("#define ACC_X_BIAS  "); Bluetooth.println(et.imu.accelBias[0]);
  Bluetooth.print("#define ACC_Y_BIAS  "); Bluetooth.println(et.imu.accelBias[1]);
  Bluetooth.print("#define ACC_Z_BIAS  "); Bluetooth.println(et.imu.accelBias[2]);
  reset();
}

void reset(){
  gxSum =0;
  axSum =0;
  gySum =0;
  aySum =0;
  gzSum =0;
  azSum =0;
  loopCount = 0;
}

void printRaw(){
  Bluetooth.print(et.imu.gyroCount[0]);Bluetooth.write(',');
  Bluetooth.print(et.imu.gyroCount[1]);Bluetooth.write(',');
  Bluetooth.print(et.imu.gyroCount[2]);Bluetooth.write(',');

  Bluetooth.print(et.imu.accelCount[0]);Bluetooth.write(',');
  Bluetooth.print(et.imu.accelCount[1]);Bluetooth.write(',');
  Bluetooth.print(et.imu.accelCount[2]-16384);Bluetooth.write(','); //cancel out gravity

  Bluetooth.print(et.imu.magCount[0]);Bluetooth.write(',');
  Bluetooth.print(et.imu.magCount[1]);Bluetooth.write(',');
  Bluetooth.print(et.imu.magCount[2]);
}

void waitForSerial(){
  flushSerial();
  while(!Bluetooth.available()){et.update();}
  flushSerial();
}
void flushSerial(){
  
  while(Bluetooth.available()){
    Bluetooth.read();
    et.update();
  }
}

