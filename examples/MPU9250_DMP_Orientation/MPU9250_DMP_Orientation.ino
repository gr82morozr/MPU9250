/************************************************************
MPU9250_Orientation
 Orientation example for MPU-9250 DMP Arduino Library 
Jim Lindblom @ SparkFun Electronics
original creation date: November 23, 2016
https://github.com/sparkfun/SparkFun_MPU9250_Arduino_Library

Uses the MPU-9250's digital motion processing engine to
determine orientation of the board.

Development environment specifics:
Arduino IDE 1.6.12
SparkFun 9DoF Razor IMU M0

Supported Platforms:
- ATSAMD21 (Arduino Zero, SparkFun SAMD21 Breakouts)
*************************************************************/
#include <MPU9250.h>



MPU9250 imu;

unsigned long stepCount = 0;
unsigned long stepTime = 0;
unsigned long lastStepCount = 0;

const signed char orientationMatrix[9] = {
  1, 0, 0,
  0, 1, 0,
  0, 0, 1
};
unsigned char lastOrient = 0;

void setup() 
{
  Serial.begin(115200);

  // Call imu.begin() to verify communication and initialize
  if (imu.begin() != INT_SUCCESS)
  {
    while (1)
    {
      Serial.println("Unable to communicate with MPU-9250");
      Serial.println("Check connections, and try again.");
      Serial.println();
      delay(5000);
    }
  }
  
  imu.begin_dmp(DMP_FEATURE_ANDROID_ORIENT);
  imu.set_dmp_orientation(orientationMatrix);
}

void loop() 
{
  if ( imu.get_fifo_available() )
  {
    imu.update_dmp_fifo();
    unsigned char orient = imu.get_dmp_orientation();
    if (orient != lastOrient)
    {
      switch (orient)
      {
      case ORIENT_PORTRAIT:
        Serial.println("Portrait");
        break;
      case ORIENT_LANDSCAPE:
        Serial.println("Landscape");
        break;
      case ORIENT_REVERSE_PORTRAIT:
        Serial.println("Portrait (Reverse)");
        break;
      case ORIENT_REVERSE_LANDSCAPE:
        Serial.println("Landscape (Reverse)");
        break;
      }
      lastOrient = orient;
    }
  }
}

