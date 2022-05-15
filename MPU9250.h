//******************************************************************************
//    MPU9250.h - MPU-9250 Digital Motion Processor Arduino Library 
//      - This is wrapper package of SparkFun_MPU9250_Arduino_Library
//      - Modified for ESP32 
//
//******************************************************************************

#pragma once 
#ifndef _GT_MPU9250_H_
#define _GT_MPU9250_H_


#include <Arduino.h>
#include <Wire.h>
#include <MPU9250_RegisterMap.h>
#include <MPU9250_Common.h>


// Include the Invensense MPU9250 driver and DMP keys:
extern "C" {
  #include "util/inv_mpu.h"
  #include "util/inv_mpu_dmp_motion_driver.h"
  #include "util/inv_mpu.h"
}


class MPU9250 {
  public:
    int   ax, ay, az;       // raw data read from register
    int   gx, gy, gz;       // raw data read from register
    int   mx, my, mz;       // raw data read from register
    long  qw, qx, qy, qz;   // quaternion data
    long  temperature;      // temperature data

    unsigned long time;
    float pitch, roll, yaw;
    float heading;
    
    float aX, aY, aZ;       // real data calculated with scale factor from raw data
    float gX, gY, gZ;       // real data calculated with scale factor from raw data
    float mX, mY, mZ;       // real data calculated with scale factor from raw data

    float aX_offset, aY_offset, aZ_offset ;     // accelerometer offset values
    float gX_offset, gY_offset, gZ_offset ;     // gyroscope offset values
    float mX_offset, mY_offset, mZ_offset ;     // magnetometer offset values
    
    
    // constyruction
    MPU9250();
    

    //**************************************************************************************
    //*                                                                                    *
    //*                         Main config functions of MPU9250                           *
    //*                                                                                    *
    //**************************************************************************************


    // begin(void) -- Verifies communication with the MPU-9250 and the AK8963,
    // and initializes them to the default state:
    // All sensors enabled
    // Gyro FSR:    +/- 2000 dps
    // Accel FSR:   +/- 2g
    // LPF:         42 Hz
    // FIFO:        50 Hz, disabled
    // Output:      INT_SUCCESS (0) on success, otherwise error
    int_return_t begin(int sda_pin, int scl_pin);
    int_return_t begin();     // with default sda=21,scl=22 pin of ESP32
    
    // set_sensors(unsigned char) -- Turn on or off MPU-9250 sensors. Any of the 
    // following defines can be combined: INV_XYZ_GYRO, INV_XYZ_ACCEL, 
    // INV_XYZ_COMPASS, INV_X_GYRO, INV_Y_GYRO, or INV_Z_GYRO
    // Input: Combination of enabled sensors. 
    // Unless specified a sensor will be disabled.
    // Output: INT_SUCCESS (0) on success, otherwise error
    int_return_t set_sensors(unsigned char sensors);
    
    // set_gyro_scale(unsigned short) -- Sets the full-scale range of the gyroscope
    // Input: Gyro DPS - 250, 500, 1000, or 2000
    // Output: INT_SUCCESS (0) on success, otherwise error
    int_return_t set_gyro_scale(unsigned short fsr);
    
    
    // get_gyro_scale -- Returns the current gyroscope FSR
    // Output: Current Gyro DPS - 250, 500, 1000, or 2000
    unsigned short get_gyro_scale(void);
    
    
    // get_gyro_sensitivity -- Returns current gyroscope sensitivity. The FSR divided by
    // the resolution of the sensor (signed 16-bit).
    // Output: Currently set gyroscope sensitivity (e.g. 131, 65.5, 32.8, 16.4)
    float get_gyro_sensitivity(void);
    
    // set_accel_scale(unsigned short) -- Sets the FSR of the accelerometer
    // 
    // Input: Accel g range - 2, 4, 8, or 16
    // Output: INT_SUCCESS (0) on success, otherwise error
    int_return_t set_accel_scale(unsigned char fsr);
    
    // get_accel_scale -- Returns the current accelerometer FSR
    // Output: Current Accel g - 2, 4, 8, or 16
    unsigned char get_accel_scale(void);
    
    // get_accel_sensitivity -- Returns current accelerometer sensitivity. The FSR 
    // divided by the resolution of the sensor (signed 16-bit).
    // Output: Currently set accel sensitivity (e.g. 16384, 8192, 4096, 2048)
    unsigned short get_accel_sensitivity(void);
    
    // get_mag_scale -- Returns the current magnetometer FSR
    // Output: Current mag uT range - +/-1450 uT
    unsigned short get_mag_scale(void);
    
    // get_mag_sensitivity -- Returns current magnetometer sensitivity. The FSR 
    // divided by the resolution of the sensor (signed 16-bit).
    // Output: Currently set mag sensitivity (e.g. 0.15)
    float get_mag_sensitivity(void);
    
    // setLPF -- Sets the digital low-pass filter of the accel and gyro.
    // Can be any of the following: 188, 98, 42, 20, 10, 5 (value in Hz)
    // Input: 188, 98, 42, 20, 10, or 5 (defaults to 5 if incorrectly set)
    // Output: INT_SUCCESS (0) on success, otherwise error
    int_return_t setLPF(unsigned short lpf);
    
    // getLPF -- Returns the set value of the LPF.
    // Output: 5, 10, 20, 42, 98, or 188 if set. 0 if the LPF is disabled.
    unsigned short getLPF(void);
    
    // set_sample_rate -- Set the gyroscope and accelerometer sample rate to a 
    // value between 4Hz and 1000Hz (1kHz).
    // The library will make an attempt to get as close as possible to the
    // requested sample rate.
    // Input: Value between 4 and 1000, indicating the desired sample rate
    // Output: INT_SUCCESS (0) on success, otherwise error
    int_return_t set_sample_rate(unsigned short rate);
        
    
    // get_sample_rate -- Get the currently set sample rate.
    // May differ slightly from what was set in set_sample_rate.
    // Output: set sample rate of the accel/gyro. A value between 4-1000.
    unsigned short get_sample_rate(void);
    

    // set_mag_sample_rate -- Set the magnetometer sample rate to a value
    // between 1Hz and 100 Hz.
    // The library will make an attempt to get as close as possible to the
    // requested sample rate.
    // Input: Value between 1 and 100, indicating the desired sample rate
    // Output: INT_SUCCESS (0) on success, otherwise error
    int_return_t set_mag_sample_rate(unsigned short rate);


    // get_mag_sample_rate -- Get the currently set magnetometer sample rate.
    // May differ slightly from what was set in set_mag_sample_rate.
    //
    // Output: set sample rate of the magnetometer. A value between 1-100
    unsigned short get_mag_sample_rate(void);
    

    // is_data_ready -- checks to see if new accel/gyro data is available.
    // (New magnetometer data cannot be checked, as the library runs that sensor 
    //  in single-conversion mode.)
    // Output: true if new accel/gyro data is available
    bool is_data_ready();
    

    // update -- Reads latest data from the MPU-9250's data registers.
    // Sensors to be updated can be set using the [sensors] parameter.
    // [sensors] can be any combination of UPDATE_ACCEL, UPDATE_GYRO,
    // UPDATE_COMPASS, and UPDATE_TEMP.
    // Output: INT_SUCCESS (0) on success, otherwise error
    // Note: after a successful update the public sensor variables 
    // (e.g. ax, ay, az, gx, gy, gz) will be updated with new data 
    int_return_t update(unsigned char sensors = UPDATE_ACCEL | UPDATE_GYRO | UPDATE_COMPASS);
    
    // update_accel, update_gyro, update_mag, and update_temp are 
    // called by the update() public method. They read from their respective
    // sensor and update the class variable (e.g. ax, ay, az)
    // Output: INT_SUCCESS (0) on success, otherwise error
    int_return_t update_accel(void);
    int_return_t update_gyro(void);
    int_return_t update_mag(void);
    int_return_t update_temp(void);
    

    // config_fifo(unsigned char) -- Initialize the FIFO, set it to read from
    // a select set of sensors.
    // Any of the following defines can be combined for the [sensors] parameter:
    // INV_XYZ_GYRO, INV_XYZ_ACCEL, INV_X_GYRO, INV_Y_GYRO, or INV_Z_GYRO
    // Input: Combination of sensors to be read into FIFO
    // Output: INT_SUCCESS (0) on success, otherwise error
    int_return_t config_fifo(unsigned char sensors);



    // get_fifo_config -- Returns the sensors configured to be read into the FIFO
    // Output: combination of INV_XYZ_GYRO, INV_XYZ_ACCEL, INV_Y_GYRO,
    //         INV_X_GYRO, or INV_Z_GYRO
    unsigned char get_fifo_config(void);


    // get_fifo_available -- Returns the number of bytes currently filled in the FIFO
    // Outputs: Number of bytes filled in the FIFO (up to 512)
    unsigned short get_fifo_available(void);


    // update_fifo -- Reads from the top of the FIFO, and stores the new data
    // in ax, ay, az, gx, gy, or gz (depending on how the FIFO is configured).
    // Output: INT_SUCCESS (0) on success, otherwise error
    int_return_t update_fifo(void);
    
    
    // reset_fifo -- Resets the FIFO's read/write pointers
    // Output: INT_SUCCESS (0) on success, otherwise error
    int_return_t reset_fifo(void);
    
    // enable_interrupt -- Configure the MPU-9250's interrupt output to indicate
    // when new data is ready.
    // Input: 0 to disable, >=1 to enable
    // Output: INT_SUCCESS (0) on success, otherwise error
    int_return_t enable_interrupt(unsigned char enable = 1);
    
    
    // set_interrupt_level -- Configure the MPU-9250's interrupt to be either active-
    // high or active-low.
    // Input: 0 for active-high, 1 for active-low
    // Output: INT_SUCCESS (0) on success, otherwise error
    int_return_t set_interrupt_level(unsigned char active_low);
    
    
    
    // set_int_latched -- Configure the MPU-9250's interrupt to latch or operate
    // as a 50us pulse.
    // Input: 0 for 
    // Output: INT_SUCCESS (0) on success, otherwise error
    int_return_t set_int_latched(unsigned char enable);
    
    
    // get_int_status -- Reads the MPU-9250's INT_STATUS register, which can
    // indicate what (if anything) caused an interrupt (e.g. FIFO overflow or
    // or data read).
    // Output: contents of the INT_STATUS register
    short get_int_status(void);
    
    // begin_dmp -- Initialize the DMP, enable one or more features, and set the FIFO's sample rate
    // features can be any one of 
    //    DMP_FEATURE_TAP             -- Tap detection
    //    DMP_FEATURE_ANDROID_ORIENT  -- Orientation (portrait/landscape) detection
    //    DMP_FEATURE_LP_QUAT         -- Accelerometer, low-power quaternion calculation
    //    DMP_FEATURE_PEDOMETER       -- Pedometer (always enabled)
    //    DMP_FEATURE_6X_LP_QUAT      -- 6-axis (accel/gyro) quaternion calculation
    //    DMP_FEATURE_GYRO_CAL        -- Gyroscope calibration (0's out after 8 seconds of no motion)
    //    DMP_FEATURE_SEND_RAW_ACCEL  -- Send raw accelerometer values to FIFO
    //    DMP_FEATURE_SEND_RAW_GYRO   -- Send raw gyroscope values to FIFO
    //    DMP_FEATURE_SEND_CAL_GYRO   -- Send calibrated gyroscop values to FIFO
    // fifoRate can be anywhere between 4 and 200Hz.
    // Input: OR'd list of features and requested FIFO sampling rate
    // Output: INT_SUCCESS (0) on success, otherwise error
    int_return_t begin_dmp(unsigned short features = 0, unsigned short fifoRate = MAX_DMP_SAMPLE_RATE);
    
 
    
    // get_dmp_fifo_rate -- Returns the sample rate of the FIFO
    // Output: Set sample rate, in Hz, of the FIFO
    unsigned short get_dmp_fifo_rate(void);
    
    
    // set_dmp_fifo_rate -- Sets the rate of the FIFO.
    // Input: Requested sample rate in Hz (range: 4-200)
    // Output: INT_SUCCESS (0) on success, otherwise error
    int_return_t set_dmp_fifo_rate(unsigned short rate);
    
    // update_dmp_fifo -- Reads from the top of the FIFO and fills accelerometer, gyroscope,
    // quaternion, and time public variables (depending on how the DMP is configured).
    // Should be called whenever an MPU interrupt is detected
    // Once read sucess, ax, ay, az
    // Output: INT_SUCCESS (0) on success, otherwise error
    // 
    int_return_t update_dmp_fifo(void); 
    
    // enable_dmp_features -- Enable one, or multiple DMP features.
    // Input: An OR'd list of features (see begin_dmp)
    // Output: INT_SUCCESS (0) on success, otherwise error
    int_return_t enable_dmp_features(unsigned short mask);
   
   
    // get_dmp_features -- Returns the OR'd list of enabled DMP features
    //
    // Output: OR'd list of DMP feature's (see begin_dmp for list)
    unsigned short get_dmp_features(void);
    
    // set_dmp_tap -- Enable tap detection and configure threshold, tap time, and minimum tap count.
    // Inputs: x/y/zThresh - accelerometer threshold on each axis. Range: 0 to 1600. 0 disables tap
    //                       detection on that axis. Units are mg/ms.
    //         taps - minimum number of taps to create a tap event (Range: 1-4)
    //         tapTime - Minimum number of milliseconds between separate taps
    //         tapMulti - Maximum number of milliseconds combined taps
    // Output: INT_SUCCESS (0) on success, otherwise error
    int_return_t set_dmp_tap(unsigned short xThresh = 250, 
                            unsigned short yThresh = 250, 
                            unsigned short zThresh = 250,
                            unsigned char taps = 1, 
                            unsigned short tapTime = 100,
                            unsigned short tapMulti = 500);
    // is_dmp_tap_available -- Returns true if a new tap is available
    // Output: True if new tap data is available. Cleared on get_dmp_tap_dir or get_dmp_tap_count.
    bool is_dmp_tap_available(void);
    
    
    // get_dmp_tap_dir -- Returns the tap direction.
    // Output: One of the following: TAP_X_UP, TAP_X_DOWN, TAP_Y_UP, TAP_Y_DOWN, TAP_Z_UP,
    //         or TAP_Z_DOWN
    unsigned char get_dmp_tap_dir(void);
    
    // get_dmp_tap_count -- Returns the number of taps in the sensed direction
    // Output: Value between 1-8 indicating successive number of taps sensed.
    unsigned char get_dmp_tap_count(void);


    // set_dmp_orientation -- Set orientation matrix, used for orientation sensing.
    // Use default_orientation matrix as an example input.
    // Input: Gyro and accel orientation in body frame (9-byte array)
    // Output: INT_SUCCESS (0) on success, otherwise error
    int_return_t set_dmp_orientation(const signed char * orientationMatrix = default_orientation);
    
    
    // get_dmp_orientation -- Get the orientation, if any.
    // Output: If an orientation is detected, 
    // one of ORIENT_LANDSCAPE
    //        ORIENT_PORTRAIT,
    //        ORIENT_REVERSE_LANDSCAPE
    //        ORIENT_REVERSE_PORTRAIT
    unsigned char get_dmp_orientation(void);


    // enable_dmp_3quat -- Enable 3-axis quaternion calculation
    // Output: INT_SUCCESS (0) on success, otherwise error
    int_return_t enable_dmp_3quat(void);


    // enable_dmp_6quat -- Enable 6-axis quaternion calculation
    // Output: INT_SUCCESS (0) on success, otherwise error
    int_return_t enable_dmp_6quat(void);
    
    // get_dmp_pedometer_steps -- Get number of steps in pedometer register
    // Output: Number of steps sensed
    unsigned long get_dmp_pedometer_steps(void);
        
    // set_dmp_pedometer_steps -- Set number of steps to a value
    // Input: Desired number of steps to begin incrementing from
    // Output: INT_SUCCESS (0) on success, otherwise error
    int_return_t set_dmp_pedometer_steps(unsigned long steps);
        
    // get_dmp_pedometer_time -- Get number of milliseconds ellapsed over stepping
    // Output: Number of milliseconds where steps were detected
    unsigned long get_dmp_pedometer_time(void);
    
    
    // set_dmp_pedometer_time -- Set number time to begin incrementing step time counter from
    // Input: Desired number of milliseconds
    // Output: INT_SUCCESS (0) on success, otherwise error
    int_return_t set_dmp_pedometer_time(unsigned long time);
    
    // set_dmp_int_mode --
    // Output: INT_SUCCESS (0) on success, otherwise error
    int_return_t set_dmp_int_mode(unsigned char mode);
    
    
    // set_gyro_bias --
    // Output: INT_SUCCESS (0) on success, otherwise error
    int_return_t set_gyro_bias(long * bias);
    
    
    // set_accel_bias -- 
    // Output: INT_SUCCESS (0) on success, otherwise error
    int_return_t set_accel_bias(long * bias);
    
    // lowPowerAccel --
    // Output: INT_SUCCESS (0) on success, otherwise error
    // Accelerometer Low-Power Mode. Rate options:
    // 1.25 (1), 2.5 (2), 5, 10, 20, 40, 
    // 80, 160, 320, or 640 Hz
    // Disables compass and gyro
    int_return_t lowPowerAccel(unsigned short rate);

    // calc_accel -- Convert 16-bit signed acceleration value to g's
    float calc_accel(int axis);
    
    // calc_gyro -- Convert 16-bit signed gyroscope value to degree's per second
    float calc_gyro(int axis);
    
    // calc_mag -- Convert 16-bit signed magnetometer value to microtesla (uT)
    float calc_mag(int axis);
    
    // calc_quat -- Convert Q30-format quaternion to a vector between +/- 1
    float calc_quat(long axis);
    
    // calc_euler_angles -- Compute euler angles based on most recently read qw, qx, qy, and qz
    // Input: boolean indicating whether angle results are presented in degrees or radians
    // Output: class variables roll, pitch, and yaw will be updated on exit.  
    void calc_euler_angles(bool degrees = true);
    
    // calc_mag_heading -- Compute heading based on most recently read mx, my, and mz values
    // Output: class variable heading will be updated on exit
    float calc_mag_heading(void);
    
    // run_self_test -- Run gyro and accel self-test.
    // Output: Returns bit mask, 1 indicates success. A 0x7 is success on all sensors.
    //         Bit pos 0: gyro
    //         Bit pos 1: accel
    //         Bit pos 2: mag
    int run_self_test(unsigned char debug = 0);

    // calibration of the sensors
    int_return_t cal_sensors();
    int_return_t cal_mag();

    // config function
    int_return_t config(int config_id);
    int_return_t refresh_reading(void);

  private:
    unsigned short _accel_sensitivity;
    float _gyro_sensitivity, _mag_sensitivity;
    
    // Convert a QN-format number to a float
    float qn_fmt_2_float(long number, unsigned char q);
    unsigned short orientation_row_2_scale(const signed char *row);


    // load_dmp_image -- Loads the DMP with 3062-byte image memory. Must be called to begin DMP.
    // This function is called by the begin_dmp function.
    // Output: INT_SUCCESS (0) on success, otherwise error
    int_return_t load_dmp_image(void);    
    

};

#endif // _GT_MPU9250_H_