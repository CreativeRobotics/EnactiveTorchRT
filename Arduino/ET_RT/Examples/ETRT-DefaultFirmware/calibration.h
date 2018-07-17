//Gyro and accelerometer calibration values - these can be obtained using the calibration sketch
//uncomment the correct define for your device here
#define ET_RT_06
//#define ET_RT_02
//#define ET_RT_03
//#define ET_RT_04
//#define ET_RT_05
//#define ET_RT_06

//#define NOCAL //use this for zero calibration values

#define GRAVITY 16384

#ifdef NOCAL
  #define GYRO_X_BIAS 0
  #define GYRO_Y_BIAS 0
  #define GYRO_Z_BIAS 0
  #define ACC_X_BIAS  0
  #define ACC_Y_BIAS  0
  #define ACC_Z_BIAS  0
#endif
//et01
#ifdef ET_RT_01
  #define GYRO_X_BIAS 84
  #define GYRO_Y_BIAS 44
  #define GYRO_Z_BIAS -18
  #define ACC_X_BIAS  84
  #define ACC_Y_BIAS  306
  #define ACC_Z_BIAS  -178
#endif
//et02
#ifdef ET_RT_02
  #define GYRO_X_BIAS 95
  #define GYRO_Y_BIAS -97
  #define GYRO_Z_BIAS -22
  #define ACC_X_BIAS  95
  #define ACC_Y_BIAS  99
  #define ACC_Z_BIAS  482
#endif
//et03
#ifdef ET_RT_03
  #define GYRO_X_BIAS 646
  #define GYRO_Y_BIAS 409
  #define GYRO_Z_BIAS -786
  #define ACC_X_BIAS  646
  #define ACC_Y_BIAS  -164
  #define ACC_Z_BIAS  2555
#endif
//et04
#ifdef ET_RT_04
  #define GYRO_X_BIAS 0
  #define GYRO_Y_BIAS 0
  #define GYRO_Z_BIAS 0
  #define ACC_X_BIAS  0
  #define ACC_Y_BIAS  0
  #define ACC_Z_BIAS  0
#endif
//ET-05
#ifdef ET_RT_05
  #define GYRO_X_BIAS 84
  #define GYRO_Y_BIAS 44
  #define GYRO_Z_BIAS -18
  #define ACC_X_BIAS  84
  #define ACC_Y_BIAS  306
  #define ACC_Z_BIAS  -178
#endif
//et06
#ifdef ET_RT_06
  #define GYRO_X_BIAS -55
  #define GYRO_Y_BIAS 34
  #define GYRO_Z_BIAS 107
  #define ACC_X_BIAS  -55
  #define ACC_Y_BIAS  177
  #define ACC_Z_BIAS  2048
#endif


