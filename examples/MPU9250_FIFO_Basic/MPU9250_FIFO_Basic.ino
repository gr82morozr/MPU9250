/************************************************************
MPU9250_FIFO_Basic
 Basic example sketch for MPU-9250 DMP Arduino Library 
Jim Lindblom @ SparkFun Electronics
original creation date: November 23, 2016
https://github.com/sparkfun/SparkFun_MPU9250_DMP_Arduino_Library

This example sketch demonstrates how to use the MPU-9250's
512 byte first-in, first-out (FIFO) buffer. The FIFO can be
set to store either accelerometer and/or gyroscope (not the
magnetometer, though :( ).

Development environment specifics:
Arduino IDE 1.6.12
SparkFun 9DoF Razor IMU M0

Supported Platforms:
- ATSAMD21 (Arduino Zero, SparkFun SAMD21 Breakouts)
*************************************************************/
#include <MPU9250_DMP.h>



MPU9250_DMP imu;

void setup() 
{
  Serial.begin(115200);

  // Call imu.begin() to verify communication with and
  // initialize the MPU-9250 to it's default values.
  // Most functions return an error code - INT_SUCCESS (0)
  // indicates the IMU was present and successfully set up
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

  // The sample rate of the accel/gyro can be set using
  // set_sample_rate. Acceptable values range from 4Hz to 1kHz
  imu.set_sample_rate(100); // Set sample rate to 100Hz

  // Use config_fifo to set which sensors should be stored
  // in the buffer.  
  // Parameter to this function can be: INV_XYZ_GYRO, 
  // INV_XYZ_ACCEL, INV_X_GYRO, INV_Y_GYRO, or INV_Z_GYRO
  imu.config_fifo(INV_XYZ_GYRO |INV_XYZ_ACCEL);
}

void loop() 
{
  // get_fifo_available returns the number of bytes in the FIFO
  // The FIFO is 512 bytes max. We'll read when it reaches
  // half of that.
  if ( imu.get_fifo_available() >= 256)
  {
    // Then read while there is data in the FIFO
    while ( imu.get_fifo_available() > 0)
    {
    // Call update_fifo to update ax, ay, az, gx, gy, and/or gz
      if ( imu.update_fifo() == INT_SUCCESS)
      {
        printIMUData();
      }
    }
  }
}

void printIMUData(void)
{  
  // After calling update() the ax, ay, az, gx, gy, gz, mx,
  // my, mz, time, and/or temerature class variables are all
  // updated. Access them by placing the object. in front:

  // Use the calc_accel, calc_gyro, and calc_mag functions to
  // convert the raw sensor readings (signed 16-bit values)
  // to their respective units.
  float accelX = imu.calc_accel(imu.ax);
  float accelY = imu.calc_accel(imu.ay);
  float accelZ = imu.calc_accel(imu.az);
  float gyroX = imu.calc_gyro(imu.gx);
  float gyroY = imu.calc_gyro(imu.gy);
  float gyroZ = imu.calc_gyro(imu.gz);
  
  Serial.println("Accel: " + String(accelX) + ", " +
              String(accelY) + ", " + String(accelZ) + " g");
  Serial.println("Gyro: " + String(gyroX) + ", " +
              String(gyroY) + ", " + String(gyroZ) + " dps");
  Serial.println("Time: " + String(imu.time) + " ms");
  Serial.println();
}
