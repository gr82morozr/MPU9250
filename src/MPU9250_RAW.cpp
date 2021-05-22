#include <MPU9250_RAW.h>


MPU9250_RAW::MPU9250_RAW(uint8_t sda_pin, uint8_t scl_pin, uint8_t address) : address(address) {
  this->init(sda_pin, scl_pin);
}

void MPU9250_RAW::init (uint8_t sda_pin, uint8_t scl_pin) {
  this->sda_pin = sda_pin;
  this->scl_pin = scl_pin;
  this->init();
}

void MPU9250_RAW::init() {
  this->accel_range = 0;
  this->gyro_range  = 0;
  this->magX_offset = 0;
  this->magY_offset = 0;
  this->magZ_offset = 0;
  this->accel_scale_factor = 0.0;
  this->gyro_scale_factor = 0.0;
  this->i2c_wire = NULL;
  this->init_i2c_wire();
}

void MPU9250_RAW::init_i2c_wire() {
  if (i2c_wire == NULL) {
    Wire.begin(this->sda_pin, this->scl_pin, MPU9250_I2C_SPEED);
    i2c_wire = &Wire;
  }
}

uint8_t MPU9250_RAW::read_i2c(uint8_t Address, uint8_t Register, uint8_t Nbytes, uint8_t* Data) {
  i2c_wire->beginTransmission(Address);
  i2c_wire->write(Register);
  uint8_t result = i2c_wire->endTransmission();
  if (result != 0) {
    return result;
  }

  i2c_wire->requestFrom(Address, Nbytes);
  uint8_t index = 0;
  while (i2c_wire->available()) {
    uint8_t d = i2c_wire->read();
    if (index < Nbytes) {
      Data[index++] = d;
    }
  }
  return 0;
}

uint8_t MPU9250_RAW::write_i2c(uint8_t Address, uint8_t Register, uint8_t Data) {
  i2c_wire->beginTransmission(Address);
  i2c_wire->write(Register);
  i2c_wire->write(Data);
  return i2c_wire->endTransmission();
}


bool MPU9250_RAW::check_i2c_mpu_id() {
  uint8_t id;
  init_i2c_wire();
  read_i2c(address, MPU9250_WHO_AM_I_RESULT, 1, &id);
  return (id == MPU9250_I2C_PASSED);
}



void MPU9250_RAW::init_accel(uint8_t mode) {
  init_i2c_wire();
  switch(mode) {
    case ACC_FULL_SCALE_2_G:
      accel_range = 2.0;
      break;
    case ACC_FULL_SCALE_4_G:
      accel_range = 4.0;
      break;
    case ACC_FULL_SCALE_8_G:
      accel_range = 8.0;
      break;
    case ACC_FULL_SCALE_16_G:
      accel_range = 16.0;
      break;
    default:
      accel_range = 8.0;
      break;
  };
  
  accel_scale_factor = accel_range / (float) 0x8000;
  write_i2c(address, MPU9250_ADDR_ACCELCONFIG, mode);
  delay(10);
}

void MPU9250_RAW::init_gyro(uint8_t mode) {
  init_i2c_wire();
  switch (mode) {
  case GYRO_FULL_SCALE_250_DPS:
    this->gyro_range = 250.0;
    break;
  case GYRO_FULL_SCALE_500_DPS:
    this->gyro_range = 500.0;
    break;
  case GYRO_FULL_SCALE_1000_DPS:
    this->gyro_range = 1000.0;
    break;
  case GYRO_FULL_SCALE_2000_DPS:
    this->gyro_range = 2000.0;
    break;
  default:
    this->gyro_range = 2000.0;
    break;
  }
  this->gyro_scale_factor = this->gyro_range / (float) 0x8000;
  write_i2c(address, 27, mode);
  delay(10);
}

void MPU9250_RAW::init_mag(uint8_t mode) {
  init_i2c_wire();
  wake_mpu();
  enable_mag_slave_mode();

  read_mag_adjust_values();
  set_mag_mode(MAG_MODE_POWERDOWN);
  set_mag_mode(mode);
  delay(10);
}

void MPU9250_RAW::read_mag_adjust_values() {
  set_mag_mode(MAG_MODE_POWERDOWN);
  set_mag_mode(MAG_MODE_FUSEROM);
  uint8_t buff[3];
  read_i2c(AK8963_ADDRESS, AK8963_RA_ASAX, 3, buff);
  magX_adjust = buff[0];
  magY_adjust = buff[1];
  magZ_adjust = buff[2];
}

void MPU9250_RAW::set_mag_mode(uint8_t mode) {
  write_i2c(AK8963_ADDRESS, AK8963_RA_CNTL1, mode);
  delay(10);
}

void MPU9250_RAW::wake_mpu() {
  unsigned char bits;
  read_i2c(address, MPU9250_ADDR_PWR_MGMT_1, 1, &bits);
  bits &= ~B01110000; // Turn off SLEEP, STANDBY, CYCLE
  write_i2c(address, MPU9250_ADDR_PWR_MGMT_1, bits);
  delay(10);
}

void MPU9250_RAW::enable_mag_slave_mode() {
  unsigned char bits;
  read_i2c(address, MPU9250_ADDR_INT_PIN_CFG, 1, &bits);
  bits |= B00000010; // Activate BYPASS_EN
  write_i2c(address, MPU9250_ADDR_INT_PIN_CFG, bits);
  delay(10);
}



float MPU9250_RAW::get_horiz_direction() {
  return atan2(magX(), magY()) * 180 / PI;
}

uint8_t MPU9250_RAW::update_mag() {
  return read_i2c(AK8963_ADDRESS, AK8963_RA_HXL, 7, magBuff);
}

int16_t MPU9250_RAW::get_mag(uint8_t highIndex, uint8_t lowIndex) {
  return (((int16_t) magBuff[highIndex]) << 8) | magBuff[lowIndex];
}

float adjust_mag_value(int16_t value, uint8_t adjust) {
  return ((float) value * (((((float) adjust - 128) * 0.5) / 128) + 1));
}

float MPU9250_RAW::magX() {
  return adjust_mag_value(get_mag(1, 0), magX_adjust) + magX_offset;
}

float MPU9250_RAW::magY() {
  return adjust_mag_value(get_mag(3, 2), magY_adjust) + magY_offset;
}

float MPU9250_RAW::magZ() {
  return adjust_mag_value(get_mag(5, 4), magZ_adjust) + magZ_offset;
}

uint8_t MPU9250_RAW::update_accel() {
  return read_i2c(address, MPU9250_ADDR_ACCEL_XOUT_H, 6, accelBuff);
}

float MPU9250_RAW::get_accel(uint8_t highIndex, uint8_t lowIndex) {
  int16_t v = ((int16_t) accelBuff[highIndex]) << 8 | accelBuff[lowIndex];
  return ((float) -v) * accel_range / (float) 0x8000; // (float) 0x8000 == 32768.0
}

float MPU9250_RAW::accelX() {
  return get_accel(0, 1);
}

float MPU9250_RAW::accelY() {
  return get_accel(2, 3);
}

float MPU9250_RAW::accelZ() {
  return get_accel(4, 5);
}

float MPU9250_RAW::accel_scaler() {
  return sqrt(pow(accelX(), 2) + pow(accelY(), 2) + pow(accelZ(), 2));
}



uint8_t MPU9250_RAW::update_gyro() {
  return read_i2c(address, MPU9250_ADDR_GYRO_XOUT_H, 6, gyroBuff);
}

float MPU9250_RAW::get_gyro(uint8_t highIndex, uint8_t lowIndex) {
  int16_t v = ((int16_t) gyroBuff[highIndex]) << 8 | gyroBuff[lowIndex];
  return ((float) -v) * gyro_range / (float) 0x8000;
}

float MPU9250_RAW::gyroX() {
  return get_gyro(0, 1);
}

float MPU9250_RAW::gyroY() {
  return get_gyro(2, 3);
}

float MPU9250_RAW::gyroZ() {
  return get_gyro(4, 5);
}