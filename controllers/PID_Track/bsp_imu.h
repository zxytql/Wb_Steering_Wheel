#ifndef BSP_IMU_H
#define BSP_IMU_H

#include <webots/inertial_unit.h>

typedef struct 
{
    const double *imu_val;
}wb_imu_data_t;

extern wb_imu_data_t wb_imu_data;

void Read_Roll_Pitch_Yaw(WbDeviceTag Tag, wb_imu_data_t *wb_imu_data);
const double Get_IMU_Roll();
const double Get_IMU_Pitch();
const double Get_IMU_Yaw();

#endif