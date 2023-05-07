#include "bsp_imu.h"

wb_imu_data_t wb_imu_data;

void Read_Roll_Pitch_Yaw(WbDeviceTag Tag, wb_imu_data_t *wb_imu_data)
{
    wb_imu_data->imu_val = wb_inertial_unit_get_roll_pitch_yaw(Tag);
}

const double Get_IMU_Roll()
{
    return wb_imu_data.imu_val[0];
}

const double Get_IMU_Pitch()
{
    return wb_imu_data.imu_val[1];
}

const double Get_IMU_Yaw()
{
    return wb_imu_data.imu_val[2];
}

