#ifndef IMU_H
#define IMU_H

#include <stdint.h>

typedef struct {
    float accel_x, accel_y, accel_z;
    float gyro_x, gyro_y, gyro_z;
    float device_motion;   // magnitude of accel deviation from 1g
    int   device_moving;   // 1 if device itself is being moved
} imu_data_t;

extern imu_data_t g_imu;

void imu_init(void);   // call AFTER touch_init (I2C already running)
void imu_read(void);   // update g_imu, call at ~20Hz

#endif
