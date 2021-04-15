#ifndef _GT_MPU9250_COMMON_H_
#define _GT_MPU9250_COMMON_H_


// Optimally, these defines would be passed as compiler options, but Arduino
// doesn't give us a great way to do that.
#define AK8963_SECONDARY
#define COMPASS_ENABLED

#define INT_SUCCESS               0
#define INT_ERROR                 0x20

/* no need to change unless debugging */
#define MPU9250_I2C_SPEED         400000

// Define's passed to update(), to request a specific sensor (or multiple):
#define UPDATE_ACCEL              (1<<1)
#define UPDATE_GYRO               (1<<2)
#define UPDATE_COMPASS            (1<<3)
#define UPDATE_TEMP               (1<<4)

#define INT_ACTIVE_HIGH           0
#define INT_ACTIVE_LOW            1
#define INT_LATCHED               1
#define INT_50US_PULSE            0

#define MAX_DMP_SAMPLE_RATE       200 // Maximum sample rate for the DMP FIFO (200Hz)
#define FIFO_BUFFER_SIZE          512 // Max FIFO buffer size
#define ORIENT_PORTRAIT           0
#define ORIENT_LANDSCAPE          1
#define ORIENT_REVERSE_PORTRAIT   2
#define ORIENT_REVERSE_LANDSCAPE  3



typedef int int_return_t;

enum axis_order_t {
  X_AXIS, // 0
  Y_AXIS, // 1
  Z_AXIS  // 2
};

const signed char default_orientation[9] = {
  1, 0, 0,
  0, 1, 0,
  0, 0, 1
};




#endif // _GT_MPU9250_COMMON_H_