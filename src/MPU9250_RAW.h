#ifndef _GT_MPU9250_RAW_H_
#define _GT_MPU9250_RAW_H_


#include <Arduino.h>
#include <Wire.h>
#include <MPU9250_RegisterMap.h>
#include <MPU9250_Common.h>
#include <math.h>

#define MPU9250_I2C_PASSED        113
#define MPU9250_I2C_SPEED         400000

#define MPU9250_ADDRESS_AD0_LOW   0x68
#define MPU9250_ADDRESS_AD0_HIGH  0x69

#define ACC_FULL_SCALE_2_G        0x00
#define ACC_FULL_SCALE_4_G        0x08
#define ACC_FULL_SCALE_8_G        0x10
#define ACC_FULL_SCALE_16_G       0x18

#define GYRO_FULL_SCALE_250_DPS   0x00
#define GYRO_FULL_SCALE_500_DPS   0x08
#define GYRO_FULL_SCALE_1000_DPS  0x10
#define GYRO_FULL_SCALE_2000_DPS  0x18

#define MAG_MODE_POWERDOWN        0x0
#define MAG_MODE_SINGLE           0x1
#define MAG_MODE_CONTINUOUS_8HZ   0x2
#define MAG_MODE_EXTERNAL         0x4
#define MAG_MODE_CONTINUOUS_100HZ 0x6
#define MAG_MODE_run_self_test         0x8
#define MAG_MODE_FUSEROM          0xF

#define MPU9250_BUFF_LEN          14
#define MPU9250_BUFF_LEN_ACCEL    6
#define MPU9250_BUFF_LEN_GYRO     6
#define MPU9250_BUFF_LEN_MAG      7


#define AK8963_ADDRESS            0x0C
#define AK8963_RA_HXL             0x03
#define AK8963_RA_CNTL1           0x0A
#define AK8963_RA_ASAX            0x10

#define MPU9250_ADDR_ACCELCONFIG  0x1C
#define MPU9250_ADDR_INT_PIN_CFG  0x37
#define MPU9250_ADDR_ACCEL_XOUT_H 0x3B
#define MPU9250_ADDR_GYRO_XOUT_H  0x43
#define MPU9250_ADDR_PWR_MGMT_1   0x6B



class MPU9250_RAW {
  public:
    const uint8_t address;
    int16_t magX_offset, magY_offset, magZ_offset;
    
    uint8_t accelBuff[MPU9250_BUFF_LEN_ACCEL];
    uint8_t gyroBuff[MPU9250_BUFF_LEN_GYRO];
    uint8_t magBuff[MPU9250_BUFF_LEN_MAG];

    MPU9250_RAW(uint8_t sda_pin, uint8_t scl_pin, uint8_t address = MPU9250_ADDRESS_AD0_LOW);

    void init();
    void init(uint8_t sda_pin, uint8_t scl_pin);


    bool check_i2c_mpu_id(); //used to confirm if I2C working.

    void init_accel(uint8_t mode = ACC_FULL_SCALE_4_G);
    uint8_t update_accel();
    float accelX();
    float accelY();
    float accelZ();
    float accel_scaler();

    void init_gyro(uint8_t mode = GYRO_FULL_SCALE_500_DPS);
    uint8_t update_gyro();
    float gyroX();
    float gyroY();
    float gyroZ();

    void init_mag(uint8_t mode = MAG_MODE_CONTINUOUS_100HZ);
    void set_mag_mode(uint8_t mode);
    uint8_t update_mag();
    float magX();
    float magY();
    float magZ();
    float get_horiz_direction();

  private:
    uint8_t sda_pin, scl_pin;

    TwoWire* i2c_wire;
    float accel_range,accel_scale_factor;
    float gyro_range,gyro_scale_factor;

    uint8_t magX_adjust, magY_adjust, magZ_adjust;
    void init_i2c_wire();
    float get_accel(uint8_t highIndex, uint8_t lowIndex);
    float get_gyro(uint8_t highIndex, uint8_t lowIndex);
    int16_t get_mag(uint8_t highIndex, uint8_t lowIndex);
    void enable_mag_slave_mode();
    void read_mag_adjust_values();
    void wake_mpu();
    uint8_t read_i2c(uint8_t Address, uint8_t Register, uint8_t Nbytes, uint8_t* Data);
    uint8_t write_i2c(uint8_t Address, uint8_t Register, uint8_t Data);
};

#endif //_GT_MPU9250_RAW_H_