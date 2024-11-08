/************************************************************
MPU9250_Basic
 Basic example sketch for MPU-9250 DMP Arduino Library 
Jim Lindblom @ SparkFun Electronics
original creation date: November 23, 2016
https://github.com/sparkfun/SparkFun_MPU9250_Arduino_Library

This example sketch demonstrates how to initialize the 
MPU-9250, and stream its sensor outputs to a serial monitor.

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

  // Call imu.begin() to verify communication with and
  // initialize the MPU-9250 to it's default values.
  // Most functions return an error code - INT_SUCCESS (0)
  // indicates the IMU was present and successfully set up
  if (imu.begin() != INT_SUCCESS) {
    while (1)  {
      Serial.println("Unable to communicate with MPU-9250");
      Serial.println("Check connections, and try again.");
      Serial.println();
      delay(5000);
    }
  }

  // Use set_sensors to turn on or off MPU-9250 sensors.
  // Any of the following defines can be combined:
  // INV_XYZ_GYRO, INV_XYZ_ACCEL, INV_XYZ_COMPASS,
  // INV_X_GYRO, INV_Y_GYRO, or INV_Z_GYRO
  // Enable all sensors:
  imu.set_sensors(INV_XYZ_GYRO | INV_XYZ_ACCEL | INV_XYZ_COMPASS);

  // Use set_gyro_scale() and set_accel_scale() to configure the
  // gyroscope and accelerometer full scale ranges.
  // Gyro options are +/- 250, 500, 1000, or 2000 dps
  imu.set_gyro_scale(2000); // Set gyro to 2000 dps
  // Accel options are +/- 2, 4, 8, or 16 g
  imu.set_accel_scale(2); // Set accel to +/-2g
  // Note: the MPU-9250's magnetometer FSR is set at 
  // +/- 4912 uT (micro-tesla's)

  // setLPF() can be used to set the digital low-pass filter
  // of the accelerometer and gyroscope.
  // Can be any of the following: 188, 98, 42, 20, 10, 5
  // (values are in Hz).
  imu.setLPF(42); // Set LPF corner frequency to 5Hz

  // The sample rate of the accel/gyro can be set using
  // set_sample_rate. Acceptable values range from 4Hz to 1kHz
  imu.set_sample_rate(10); // Set sample rate to 10Hz

  // Likewise, the compass (magnetometer) sample rate can be
  // set using the set_mag_sample_rate() function.
  // This value can range between: 1-100Hz
  imu.set_mag_sample_rate(10); // Set mag rate to 10Hz
}

void loop() {
  // is_data_ready() checks to see if new accel/gyro data
  // is available. It will return a boolean true or false
  // (New magnetometer data cannot be checked, as the library
  //  runs that sensor in single-conversion mode.)
  if ( imu.is_data_ready() )
  {
    // Call update() to update the imu objects sensor data.
    // You can specify which sensors to update by combining
    // UPDATE_ACCEL, UPDATE_GYRO, UPDATE_COMPASS, and/or
    // UPDATE_TEMPERATURE.
    // (The update function defaults to accel, gyro, compass,
    //  so you don't have to specify these values.)
    imu.update(UPDATE_ACCEL | UPDATE_GYRO | UPDATE_COMPASS);
    printIMUData();
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
  float magX = imu.calc_mag(imu.mx);
  float magY = imu.calc_mag(imu.my);
  float magZ = imu.calc_mag(imu.mz);
  
  Serial.println("Accel: " + String(accelX) + ", " +
              String(accelY) + ", " + String(accelZ) + " g");
  Serial.println("Gyro: " + String(gyroX) + ", " +
              String(gyroY) + ", " + String(gyroZ) + " dps");
  Serial.println("Mag: " + String(magX) + ", " +
              String(magY) + ", " + String(magZ) + " uT");
  Serial.println("Time: " + String(imu.time) + " ms");
  Serial.println();
}

