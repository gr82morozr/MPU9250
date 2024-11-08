/******************************************************************************
MPU9250.cpp - MPU-9250 Digital Motion Processor Arduino Library 

******************************************************************************/


#include <MPU9250.h>


static unsigned char mpu9250_orientation;
static unsigned char tap_count;
static unsigned char tap_direction;
static bool _tap_available;
static void orient_cb(unsigned char orient);
static void tap_cb(unsigned char direction, unsigned char count);

MPU9250::MPU9250() {
  _mag_sensitivity    = 6.665f; // Constant - 4915 / 32760
  _accel_sensitivity  = 0.0f;   // Updated after accel FSR is set
  _gyro_sensitivity   = 0.0f;   // Updated after gyro FSR is set

  aX_offset = 0.0; aY_offset=0.0; aZ_offset=0.0;
  gX_offset = 0.0; gY_offset=0.0; gZ_offset=0.0;
}


int_return_t MPU9250::begin() {
  return this->begin(DEFAULT_SCA_PIN,DEFAULT_SCL_PIN, MPU9250_I2C_SPEED);
}


int_return_t MPU9250::begin(int sda_pin, int scl_pin, uint32_t i2c_speed) {
  int_return_t result;
  struct int_param_s int_param;
  Wire.begin( (int) sda_pin, (int) scl_pin, (uint32_t) i2c_speed);
  result = mpu_init(&int_param);
  
  mpu_set_bypass(1); // Place all slaves (including compass) on primary bus
  set_sensors(INV_XYZ_GYRO | INV_XYZ_ACCEL | INV_XYZ_COMPASS);
  
  _gyro_sensitivity   = get_gyro_sensitivity();
  _accel_sensitivity  = get_accel_sensitivity();
  
  while (result != INT_SUCCESS)  {
    if (Serial.availableForWrite() == 0) {
      Serial.begin(115200);
    }
    Serial.println("Unable to communicate with MPU-9250, check connection ...");
    Serial.println();    
    delay(5000);
    result = mpu_init(&int_param);
  };

  return 0; 
}

int_return_t MPU9250::enable_interrupt(unsigned char enable) {
  return set_int_enable(enable);
}

int_return_t MPU9250::set_interrupt_level(unsigned char active_low) {
  return mpu_set_int_level(active_low);
}

int_return_t MPU9250::set_int_latched(unsigned char enable) {
  return mpu_set_int_latched(enable);
}

short MPU9250::get_int_status(void) {
  short status;
  if (mpu_get_int_status(&status) == INT_SUCCESS)  {
    return status;
  }
  return 0;
}

// Accelerometer Low-Power Mode. Rate options:
// 1.25 (1), 2.5 (2), 5, 10, 20, 40, 
// 80, 160, 320, or 640 Hz
// Disables compass and gyro
int_return_t MPU9250::lowPowerAccel(unsigned short rate) {
  return mpu_lp_accel_mode(rate);
}

int_return_t MPU9250::set_gyro_scale(unsigned short fsr) {
  int_return_t err;
  err = mpu_set_gyro_fsr(fsr);
  if (err == INT_SUCCESS) {
    _gyro_sensitivity = get_gyro_sensitivity();
  }
  return err;
}

int_return_t MPU9250::set_accel_scale(unsigned char fsr) {
  int_return_t err;
  err = mpu_set_accel_fsr(fsr);
  if (err == INT_SUCCESS)  {
    _accel_sensitivity = get_accel_sensitivity();
  }
  return err;
}

unsigned short MPU9250::get_gyro_scale(void) {
  unsigned short tmp;
  if (mpu_get_gyro_fsr(&tmp) == INT_SUCCESS)   {
    return tmp;
  }
  return 0;
}

unsigned char MPU9250::get_accel_scale(void) {
  unsigned char tmp;
  if (mpu_get_accel_fsr(&tmp) == INT_SUCCESS)   {
    return tmp;
  }
  return 0; 
}

unsigned short MPU9250::get_mag_scale(void) {
  unsigned short tmp;
  if (mpu_get_compass_fsr(&tmp) == INT_SUCCESS)   {
    return tmp;
  }
  return 0;
}

int_return_t MPU9250::setLPF(unsigned short lpf) {
  return mpu_set_lpf(lpf);
}

unsigned short MPU9250::getLPF(void) {
  unsigned short tmp;
  if (mpu_get_lpf(&tmp) == INT_SUCCESS)   {
    return tmp;
  }
  return 0;
}

int_return_t MPU9250::set_sample_rate(unsigned short rate) {
    return mpu_set_sample_rate(rate);
}

unsigned short MPU9250::get_sample_rate(void) {
  unsigned short tmp;
  if (mpu_get_sample_rate(&tmp) == INT_SUCCESS)   {
    return tmp;
  }
  return 0;
}

int_return_t MPU9250::set_mag_sample_rate(unsigned short rate) {
  return mpu_set_compass_sample_rate(rate);
}

unsigned short MPU9250::get_mag_sample_rate(void) {
  unsigned short tmp;
  if (mpu_get_compass_sample_rate(&tmp) == INT_SUCCESS)   {
    return tmp;
  }
  
  return 0;
}

float MPU9250::get_gyro_sensitivity(void) {
  float sens;
  if (mpu_get_gyro_sens(&sens) == INT_SUCCESS)   {
    return sens; 
  }
  return 0;
}
  
unsigned short MPU9250::get_accel_sensitivity(void) {
  unsigned short sens;
  if (mpu_get_accel_sens(&sens) == INT_SUCCESS)   {
    return sens;
  }
  return 0;
}

float MPU9250::get_mag_sensitivity(void) {
  return 0.15; // Static, 4915/32760
}

unsigned char MPU9250::get_fifo_config(void) {
  unsigned char sensors;
  if (mpu_get_fifo_config(&sensors) == INT_SUCCESS)   {
    return sensors;
  }
  return 0;
}

int_return_t MPU9250::config_fifo(unsigned char sensors) {
  return mpu_configure_fifo(sensors);
}

int_return_t MPU9250::reset_fifo(void) {
  return mpu_reset_fifo();
}

unsigned short MPU9250::get_fifo_available(void) {
  unsigned char fifoH, fifoL;
  
  if (mpu_read_reg(MPU9250_FIFO_COUNTH, &fifoH) != INT_SUCCESS)
    return 0;
  if (mpu_read_reg(MPU9250_FIFO_COUNTL, &fifoL) != INT_SUCCESS)
    return 0;
  
  return (fifoH << 8 ) | fifoL;
}

int_return_t MPU9250::update_fifo(void) {
  short gyro[3], accel[3];
  unsigned long timestamp;
  unsigned char sensors, more;
  
  if (mpu_read_fifo(gyro, accel, &timestamp, &sensors, &more) != INT_SUCCESS)
    return INT_ERROR;
  
  if (sensors & INV_XYZ_ACCEL)   {
    ax = accel[X_AXIS];
    ay = accel[Y_AXIS];
    az = accel[Z_AXIS];
  }
  if (sensors & INV_X_GYRO) 
    gx = gyro[X_AXIS];
  if (sensors & INV_Y_GYRO)
    gy = gyro[Y_AXIS];
  if (sensors & INV_Z_GYRO)
    gz = gyro[Z_AXIS];
  
  time = timestamp;
  
  return INT_SUCCESS;
}

int_return_t MPU9250::set_sensors(unsigned char sensors) {
  return mpu_set_sensors(sensors);
}

bool MPU9250::is_data_ready() {
  unsigned char intStatusReg;
  if (mpu_read_reg(MPU9250_INT_STATUS, &intStatusReg) == INT_SUCCESS)   {
    return (intStatusReg & (1<<INT_STATUS_RAW_DATA_RDY_INT));
  }
  return false;
}

int_return_t MPU9250::update(unsigned char sensors) {
  int_return_t aErr = INT_SUCCESS;
  int_return_t gErr = INT_SUCCESS;
  int_return_t mErr = INT_SUCCESS;
  int_return_t tErr = INT_SUCCESS;
  
  if (sensors & UPDATE_ACCEL)
    aErr = update_accel();
  if (sensors & UPDATE_GYRO)
    gErr = update_gyro();
  if (sensors & UPDATE_COMPASS)
    mErr = update_mag();
  if (sensors & UPDATE_TEMP)
    tErr = update_temp();
  
  return aErr | gErr | mErr | tErr;
}

int MPU9250::update_accel(void) {
  short data[3];
  
  if (mpu_get_accel_reg(data, &time))   {
    return INT_ERROR;   
  }
  ax = data[X_AXIS];
  ay = data[Y_AXIS];
  az = data[Z_AXIS];
  return INT_SUCCESS;
}

int MPU9250::update_gyro(void) {
  short data[3];
  
  if (mpu_get_gyro_reg(data, &time))   {
    return INT_ERROR;   
  }
  gx = data[X_AXIS];
  gy = data[Y_AXIS];
  gz = data[Z_AXIS];
  return INT_SUCCESS;
}

int MPU9250::update_mag(void) {
  short data[3];
  
  if (mpu_get_compass_reg(data, &time))  {
    return INT_ERROR;   
  }
  mx = data[X_AXIS];
  my = data[Y_AXIS];
  mz = data[Z_AXIS];
  return INT_SUCCESS;
}

int_return_t MPU9250::update_temp(void) {
  return mpu_get_temperature(&temperature, &time);
}

int MPU9250::run_self_test(unsigned char debug) {
  long gyro[3], accel[3];
  return mpu_run_self_test(gyro, accel);
}

int_return_t MPU9250::begin_dmp(unsigned short features, unsigned short fifoRate) {
  unsigned short feat = features;
  unsigned short rate = fifoRate;

  if (load_dmp_image() != INT_SUCCESS)
    return INT_ERROR;
  
  // 3-axis and 6-axis LP quat are mutually exclusive.
  // If both are selected, default to 3-axis
  if (feat & DMP_FEATURE_LP_QUAT)   {
    feat &= ~(DMP_FEATURE_6X_LP_QUAT);
    dmp_enable_lp_quat(1);
  }
  else if (feat & DMP_FEATURE_6X_LP_QUAT) 
    dmp_enable_6x_lp_quat(1);
  
  if (feat & DMP_FEATURE_GYRO_CAL)
    dmp_enable_gyro_cal(1);
  
  if (enable_dmp_features(feat) != INT_SUCCESS)
    return INT_ERROR;
  
  rate = constrain(rate, 1, 200);
  if (set_dmp_fifo_rate(rate) != INT_SUCCESS)
    return INT_ERROR;
  
  return mpu_set_dmp_state(1);
}

int_return_t MPU9250::load_dmp_image(void)
{
  return dmp_load_motion_driver_firmware();
}

unsigned short MPU9250::get_dmp_fifo_rate(void) {
  unsigned short rate;
  if (dmp_get_fifo_rate(&rate) == INT_SUCCESS)
    return rate;
  
  return 0;
}

int_return_t MPU9250::set_dmp_fifo_rate(unsigned short rate) {
  if (rate > MAX_DMP_SAMPLE_RATE) rate = MAX_DMP_SAMPLE_RATE;
  return dmp_set_fifo_rate(rate);
}

int_return_t MPU9250::update_dmp_fifo(void) {
  short gyro[3];
  short accel[3];
  long quat[4];
  unsigned long timestamp;
  short sensors;
  unsigned char more;
  
  if (dmp_read_fifo(gyro, accel, quat, &timestamp, &sensors, &more)!= INT_SUCCESS)   {
     return INT_ERROR;
  }
  
  if (sensors & INV_XYZ_ACCEL) {
    ax = accel[X_AXIS];
    ay = accel[Y_AXIS];
    az = accel[Z_AXIS];
  }
  if (sensors & INV_X_GYRO)
    gx = gyro[X_AXIS];
  if (sensors & INV_Y_GYRO)
    gy = gyro[Y_AXIS];
  if (sensors & INV_Z_GYRO)
    gz = gyro[Z_AXIS];
  if (sensors & INV_WXYZ_QUAT) {
    qw = quat[0];
    qx = quat[1];
    qy = quat[2];
    qz = quat[3];
  }
  
  time = timestamp;
  
  return INT_SUCCESS;
}

int_return_t MPU9250::enable_dmp_features(unsigned short mask) {
  unsigned short enMask = 0;
  enMask |= mask;
  // Combat known issue where fifo sample rate is incorrect
  // unless tap is enabled in the DMP.
  enMask |= DMP_FEATURE_TAP; 
  return dmp_enable_feature(enMask);
}

unsigned short MPU9250::get_dmp_features(void) {
  unsigned short mask;
  if (dmp_get_enabled_features(&mask) == INT_SUCCESS)
    return mask;
  return 0;
}

int_return_t MPU9250::set_dmp_tap(
        unsigned short xThresh, unsigned short yThresh, unsigned short zThresh,
        unsigned char taps, unsigned short tapTime, unsigned short tapMulti) {
  unsigned char axes = 0;
  if (xThresh > 0)  {
    axes |= TAP_X;
    xThresh = constrain(xThresh, 1, 1600);
    if (dmp_set_tap_thresh(1<<X_AXIS, xThresh) != INT_SUCCESS)
      return INT_ERROR;
  }
  if (yThresh > 0)  {
    axes |= TAP_Y;
    yThresh = constrain(yThresh, 1, 1600);
    if (dmp_set_tap_thresh(1<<Y_AXIS, yThresh) != INT_SUCCESS)
      return INT_ERROR;
  }
  if (zThresh > 0)  {
    axes |= TAP_Z;
    zThresh = constrain(zThresh, 1, 1600);
    if (dmp_set_tap_thresh(1<<Z_AXIS, zThresh) != INT_SUCCESS)
      return INT_ERROR;
  }
  if (dmp_set_tap_axes(axes) != INT_SUCCESS)
    return INT_ERROR;
  if (dmp_set_tap_count(taps) != INT_SUCCESS)
    return INT_ERROR;
  if (dmp_set_tap_time(tapTime) != INT_SUCCESS)
    return INT_ERROR;
  if (dmp_set_tap_time_multi(tapMulti) != INT_SUCCESS)
    return INT_ERROR;
  
  dmp_register_tap_cb(tap_cb);
  
  return INT_SUCCESS;
}

unsigned char MPU9250::get_dmp_tap_dir(void) {
  _tap_available = false;
  return tap_direction;
}

unsigned char MPU9250::get_dmp_tap_count(void) {
  _tap_available = false;
  return tap_count;
}

bool MPU9250::is_dmp_tap_available(void) {
  return _tap_available;
}

int_return_t MPU9250::set_dmp_orientation(const signed char * orientationMatrix) {
  unsigned short scalar;
  scalar = orientation_row_2_scale(orientationMatrix);
  scalar |= orientation_row_2_scale(orientationMatrix + 3) << 3;
  scalar |= orientation_row_2_scale(orientationMatrix + 6) << 6;

  dmp_register_android_orient_cb(orient_cb);
  return dmp_set_orientation(scalar);
}

unsigned char MPU9250::get_dmp_orientation(void) {
  return mpu9250_orientation;
}

int_return_t MPU9250::enable_dmp_3quat(void) {
  unsigned short dmpFeatures;
  
  // 3-axis and 6-axis quat are mutually exclusive
  dmpFeatures = get_dmp_features();
  dmpFeatures &= ~(DMP_FEATURE_6X_LP_QUAT);
  dmpFeatures |= DMP_FEATURE_LP_QUAT;
  
  if (enable_dmp_features(dmpFeatures) != INT_SUCCESS)
    return INT_ERROR;
  
  return dmp_enable_lp_quat(1);
}
  
unsigned long MPU9250::get_dmp_pedometer_steps(void) {
  unsigned long steps;
  if (dmp_get_pedometer_step_count(&steps) == INT_SUCCESS)  {
    return steps;
  }
  return 0;
}

int_return_t MPU9250::set_dmp_pedometer_steps(unsigned long steps) {
  return dmp_set_pedometer_step_count(steps);
}

unsigned long MPU9250::get_dmp_pedometer_time(void) {
  unsigned long walkTime;
  if (dmp_get_pedometer_walk_time(&walkTime) == INT_SUCCESS)  {
    return walkTime;
  }
  return 0;
}

int_return_t MPU9250::set_dmp_pedometer_time(unsigned long time) {
  return dmp_set_pedometer_walk_time(time);
}

float MPU9250::calc_accel(int axis) {
  return (float) axis / (float) _accel_sensitivity;
}

float MPU9250::calc_gyro(int axis) {
  return (float) axis / (float) _gyro_sensitivity;
}

float MPU9250::calc_mag(int axis) {
  return (float) axis / (float) _mag_sensitivity;
}

float MPU9250::calc_quat(long axis) {
  return qn_fmt_2_float(axis, 30);
}
  
float MPU9250::qn_fmt_2_float(long number, unsigned char q) {
  unsigned long mask = 0;
  for (int i=0; i<q; i++)   {
    mask |= (1<<i);
  }
  return (number >> q) + ((number & mask) / (float) (2<<(q-1)));
}

void MPU9250::calc_euler_angles(bool degrees) {
    float dqw = qn_fmt_2_float(qw, 30);
    float dqx = qn_fmt_2_float(qx, 30);
    float dqy = qn_fmt_2_float(qy, 30);
    float dqz = qn_fmt_2_float(qz, 30);
    
    float ysqr = dqy * dqy;
    float t0 = -2.0f * (ysqr + dqz * dqz) + 1.0f;
    float t1 = +2.0f * (dqx * dqy - dqw * dqz);
    float t2 = -2.0f * (dqx * dqz + dqw * dqy);
    float t3 = +2.0f * (dqy * dqz - dqw * dqx);
    float t4 = -2.0f * (dqx * dqx + ysqr) + 1.0f;
  
  // Keep t2 within range of asin (-1, 1)
    t2 = t2 > 1.0f ? 1.0f : t2;
    t2 = t2 < -1.0f ? -1.0f : t2;
  
    pitch = asin(t2) * 2;
    roll = atan2(t3, t4);
    yaw = atan2(t1, t0);
  
  if (degrees)  {
    pitch *= (180.0 / PI);
    roll *= (180.0 / PI);
    yaw *= (180.0 / PI);
    if (pitch < 0) pitch = 360.0 + pitch;
    if (roll < 0) roll = 360.0 + roll;
    if (yaw < 0) yaw = 360.0 + yaw; 
  }
}

float MPU9250::calc_mag_heading(void) {
  if (my == 0)
    heading = (mx < 0) ? PI : 0;
  else
    heading = atan2(mx, my);
  
  if (heading > PI) heading -= (2 * PI);
  else if (heading < -PI) heading += (2 * PI);
  else if (heading < 0) heading += 2 * PI;
  
  heading*= 180.0 / PI;
  
  return heading;
}

unsigned short MPU9250::orientation_row_2_scale(const signed char *row) {
    unsigned short b;

    if (row[0] > 0)
        b = 0;
    else if (row[0] < 0)
        b = 4;
    else if (row[1] > 0)
        b = 1;
    else if (row[1] < 0)
        b = 5;
    else if (row[2] > 0)
        b = 2;
    else if (row[2] < 0)
        b = 6;
    else
        b = 7;    // error
    return b;
}
    
static void tap_cb(unsigned char direction, unsigned char count) {
  _tap_available = true;
  tap_count = count;
  tap_direction = direction;
}

static void orient_cb(unsigned char orient) {
  mpu9250_orientation = orient;
}


int_return_t MPU9250::cal_sensors() {
  long start = millis();
  float count = 0;
  aX_offset = 0.0; aY_offset=0.0; aZ_offset=0.0;
  gX_offset = 0.0; gY_offset=0.0; gZ_offset=0.0;

  do {
    count ++;
    update();
    aX_offset += calc_accel(ax);
    aY_offset += calc_accel(ay);
    aZ_offset += calc_accel(az); 

    gX_offset += calc_gyro(gx);
    gY_offset += calc_gyro(gy);
    gZ_offset += calc_gyro(gz);

    delay(10);

  }  while (millis() - start <= 10000);
    
  aX_offset = aX_offset/count; 
  aY_offset = aY_offset/count; 
  aZ_offset = aZ_offset/count; 
  gX_offset = gX_offset/count; 
  gY_offset = gY_offset/count; 
  gZ_offset = gZ_offset/count; 

  return 1;
};




int_return_t MPU9250::cal_mag() {
  return 1;
};



int_return_t MPU9250::config(int config_id) {
  if (config_id !=1) return 1;
  
  switch (config_id) {
    case 1 :
      this->begin_dmp(DMP_FEATURE_6X_LP_QUAT | DMP_FEATURE_GYRO_CAL,  60); // Set DMP FIFO rate to 60 Hz
      this->set_sensors(INV_XYZ_GYRO | INV_XYZ_ACCEL | INV_XYZ_COMPASS);
      this->set_gyro_scale(2000); // Set gyro to 2000 dps
      this->set_accel_scale(4); // Set accel to +/-2g
      this->setLPF(5); // Set LPF corner frequency to 5Hz
      this->set_sample_rate(60); // Set sample rate to 10Hz
      this->set_mag_sample_rate(10); // Set mag rate to 10Hz
      break;
    default:
      break;
  };

  return 0;
}


int_return_t MPU9250::refresh_reading() {
  if ( this->is_data_ready() )   {
    // Call update() to update the imu objects sensor data.
    // You can specify which sensors to update by combining
    // UPDATE_ACCEL, UPDATE_GYRO, UPDATE_COMPASS, and/or
    // UPDATE_TEMPERATURE.
    // (The update function defaults to accel, gyro, compass,
    //  so you don't have to specify these values.)
    this->update(UPDATE_ACCEL | UPDATE_GYRO | UPDATE_COMPASS);
    // Check for new data in the FIFO
    if ( this->get_fifo_available() )   {
      // Use update_dmp_fifo to update the ax, gx, mx, etc. values
      if ( this->update_dmp_fifo() == INT_SUCCESS)   {
        // calc_euler_angles can be used -- after updating the
        // quaternion values -- to estimate roll, pitch, and yaw
        this->calc_euler_angles();
      }
    }
  }
  return 0;
}