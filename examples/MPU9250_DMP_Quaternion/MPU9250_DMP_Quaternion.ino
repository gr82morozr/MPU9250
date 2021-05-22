/************************************************************
MPU9250_DMP_Quaternion
 Quaternion example for MPU-9250 DMP Arduino Library 
Jim Lindblom @ SparkFun Electronics
original creation date: November 23, 2016
https://github.com/sparkfun/SparkFun_MPU9250_DMP_Arduino_Library

The MPU-9250's digital motion processor (DMP) can calculate
four unit quaternions, which can be used to represent the
rotation of an object.

This exmaple demonstrates how to configure the DMP to 
calculate quaternions, and prints them out to the serial
monitor. It also calculates pitch, roll, and yaw from those
values.

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
  
  imu.begin_dmp(DMP_FEATURE_6X_LP_QUAT | // Enable 6-axis quat
               DMP_FEATURE_GYRO_CAL, // Use gyro calibration
              10); // Set DMP FIFO rate to 10 Hz
  // DMP_FEATURE_LP_QUAT can also be used. It uses the 
  // accelerometer in low-power mode to estimate quat's.
  // DMP_FEATURE_LP_QUAT and 6X_LP_QUAT are mutually exclusive
}

void loop() 
{
  // Check for new data in the FIFO
  if ( imu.get_fifo_available() )
  {
    // Use update_dmp_fifo to update the ax, gx, mx, etc. values
    if ( imu.update_dmp_fifo() == INT_SUCCESS)
    {
      // calc_euler_angles can be used -- after updating the
      // quaternion values -- to estimate roll, pitch, and yaw
      imu.calc_euler_angles();
      printIMUData();
    }
  }
}

void printIMUData(void)
{  
  // After calling update_dmp_fifo() the ax, gx, mx, etc. values
  // are all updated.
  // Quaternion values are, by default, stored in Q30 long
  // format. calc_quat turns them into a float between -1 and 1
  float q0 = imu.calc_quat(imu.qw);
  float q1 = imu.calc_quat(imu.qx);
  float q2 = imu.calc_quat(imu.qy);
  float q3 = imu.calc_quat(imu.qz);

  Serial.println("Q: " + String(q0, 4) + ", " +
                    String(q1, 4) + ", " + String(q2, 4) + 
                    ", " + String(q3, 4));
  Serial.println("R/P/Y: " + String(imu.roll) + ", "
            + String(imu.pitch) + ", " + String(imu.yaw));
  Serial.println("Time: " + String(imu.time) + " ms");
  Serial.println();
}

