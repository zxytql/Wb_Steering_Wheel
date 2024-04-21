#include "sensor.h"

wb_imu_data_t wb_imu_data;
wb_gps_data_t wb_gps_data;
wb_spvs_data_t wb_spvs_data;

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

void Read_GPS_Val(WbDeviceTag Tag, wb_gps_data_t *wb_gps_data)
{
    wb_gps_data->gps_val = wb_gps_get_values(Tag);
}

const double Get_GPS_X()
{
    return wb_gps_data.gps_val[0];
}

const double Get_GPS_Y()
{
    return wb_gps_data.gps_val[1];
}


const double Get_GPS_Z()
{
    return wb_gps_data.gps_val[2];
}

const double Get_GPS_X_mm()
{
    return wb_gps_data.gps_val[0]*1000.0f;
}

const double Get_GPS_Y_mm()
{
    return wb_gps_data.gps_val[1]*1000.0f;
}
const double Get_GPS_Z_mm()
{
    return wb_gps_data.gps_val[2]*1000.0f;
}

void Read_SPVS_Val(wb_spvs_data_t *wb_spvs_data)
{
    wb_spvs_data->node_name = wb_supervisor_node_get_self();
    wb_spvs_data->node_vel = wb_supervisor_node_get_velocity(wb_spvs_data->node_name);
}

const double Get_SPVS_X_Vel()
{
    return wb_spvs_data.node_vel[0];
}

const double Get_SPVS_Y_Vel()
{
    return wb_spvs_data.node_vel[1];
}

const double Get_SPVS_X_Vel_mm()
{
    return wb_spvs_data.node_vel[0] * 1000.0f;
}

const double Get_SPVS_Y_Vel_mm()
{
    return wb_spvs_data.node_vel[1] * 1000.0f;
}

const double Get_SPVS_Z_Angular_Vel()
{
    return wb_spvs_data.node_vel[5];
}
