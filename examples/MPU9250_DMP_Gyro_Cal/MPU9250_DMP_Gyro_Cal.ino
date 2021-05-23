/************************************************************
MPU9250_Gyro_Cal
 Gyro calibration example for MPU-9250 DMP Arduino Library 
Jim Lindblom @ SparkFun Electronics
original creation date: November 23, 2016
https://github.com/sparkfun/SparkFun_MPU9250_Arduino_Library

This example sketch demonstrates how to use the MPU-9250's
digital motion processor (DMP) to calibrate the gyroscope.
After eight seconds of no motion, the DMP will compute
gyro biases and subtract them.

Development environment specifics:
Arduino IDE 1.6.12
SparkFun 9DoF Razor IMU M0

Supported Platforms:
- ATSAMD21 (Arduino Zero, SparkFun SAMD21 Breakouts)
*************************************************************/
#include <MPU9250.h>



MPU9250 imu;

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

  imu.set_sensors(INV_XYZ_GYRO); // Enable gyroscope only
  imu.set_gyro_scale(2000); // Set gyro to 2000 dps

  imu.begin_dmp(DMP_FEATURE_GYRO_CAL |   // Enable gyro cal
              DMP_FEATURE_SEND_CAL_GYRO,// Send cal'd gyro values
              10);                   // Set DMP rate to 10 Hz
}

void loop() 
{
  // Check for new data in the FIFO
  if ( imu.get_fifo_available() )
  {
    // Use update_dmp_fifo to update the ax, gx, mx, etc. values
    if ( imu.update_dmp_fifo() == INT_SUCCESS)
    {
      printIMUData();
    }
  }
}

void printIMUData(void)
{  
  // After calling update_dmp_fifo() the ax, gx, mx, etc. values
  // are all updated.
  float gyroX = imu.calc_gyro(imu.gx);
  float gyroY = imu.calc_gyro(imu.gy);
  float gyroZ = imu.calc_gyro(imu.gz);
  
  Serial.println("Gyro: " + String(gyroX) + ", " +
              String(gyroY) + ", " + String(gyroZ) + " dps");
  Serial.println("Time: " + String(imu.time) + " ms");
  Serial.println();
}

