/************************************************************
MPU9250_Basic_Interrupt
 Basic interrupt sketch for MPU-9250 DMP Arduino Library 
Jim Lindblom @ SparkFun Electronics
original creation date: November 23, 2016
https://github.com/sparkfun/SparkFun_MPU9250_Arduino_Library

This example sketch demonstrates how to initialize the 
MPU-9250, and stream its sensor outputs to a serial monitor.
It uses the MPU-9250's interrupt output to indicate when
new data is ready.

Development environment specifics:
Arduino IDE 1.6.12
SparkFun 9DoF Razor IMU M0 (interrupt on pin 4)

Supported Platforms:
- ATSAMD21 (Arduino Zero, SparkFun SAMD21 Breakouts)
*************************************************************/
#include <MPU9250.h>


#define INTERRUPT_PIN 4

MPU9250 imu;

void setup() 
{
  pinMode(INTERRUPT_PIN, INPUT_PULLUP);
  Serial.begin(115200);
  
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

  // Enable all sensors, and set sample rates to 4Hz.
  // (Slow so we can see the interrupt work.)
  imu.set_sensors(INV_XYZ_GYRO | INV_XYZ_ACCEL | INV_XYZ_COMPASS);
  imu.set_sample_rate(4); // Set accel/gyro sample rate to 4Hz
  imu.set_mag_sample_rate(4); // Set mag rate to 4Hz

  // Use enable_interrupt() to configure the MPU-9250's 
  // interrupt output as a "data ready" indicator.
  imu.enable_interrupt();

  // The interrupt level can either be active-high or low.
  // Configure as active-low, since we'll be using the pin's
  // internal pull-up resistor.
  // Options are INT_ACTIVE_LOW or INT_ACTIVE_HIGH
  imu.set_interrupt_level(INT_ACTIVE_LOW);

  // The interrupt can be set to latch until data has
  // been read, or to work as a 50us pulse.
  // Use latching method -- we'll read from the sensor
  // as soon as we see the pin go LOW.
  // Options are INT_LATCHED or INT_50US_PULSE
  imu.set_int_latched(INT_LATCHED);
}

void loop() {
  // The interrupt pin is pulled up using an internal pullup
  // resistor, and the MPU-9250 is configured to trigger
  // the input LOW.
  if ( digitalRead(INTERRUPT_PIN) == LOW )  {
    // Call update() to update the imu objects sensor data.
    imu.update(UPDATE_ACCEL | UPDATE_GYRO | UPDATE_COMPASS);
    printIMUData();
  }
}

void printIMUData(void) {  
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

