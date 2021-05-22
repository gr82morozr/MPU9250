/************************************************************
MPU9250_DMP_Pedometer
 Pedometer example for MPU-9250 DMP Arduino Library 
Jim Lindblom @ SparkFun Electronics
original creation date: November 23, 2016
https://github.com/sparkfun/SparkFun_MPU9250_DMP_Arduino_Library

The MPU-9250's digital motion processor (DMP) can estimate
steps taken -- effecting a pedometer.

After uploading the code, try shaking the 9DoF up and
down at a "stepping speed."

Development environment specifics:
Arduino IDE 1.6.12
SparkFun 9DoF Razor IMU M0

Supported Platforms:
- ATSAMD21 (Arduino Zero, SparkFun SAMD21 Breakouts)
*************************************************************/
#include <MPU9250_DMP.h>



MPU9250_DMP imu;

unsigned long stepCount = 0;
unsigned long stepTime = 0;
unsigned long lastStepCount = 0;

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
  
  imu.begin_dmp(DMP_FEATURE_PEDOMETER);
  imu.set_dmp_pedometer_steps(stepCount);
  imu.set_dmp_pedometer_time(stepTime);
}

void loop() 
{
  stepCount = imu.get_dmp_pedometer_steps();
  stepTime = imu.get_dmp_pedometer_time();
  
  if (stepCount != lastStepCount)
  {
    lastStepCount = stepCount;
    Serial.print("Walked " + String(stepCount) + 
                     " steps");
    Serial.println(" (" + 
              String((float)stepTime / 1000.0) + " s)");
  }
}

