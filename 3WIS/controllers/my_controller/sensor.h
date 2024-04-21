#ifndef SENSOR_H
#define SENSOR_H

#include <webots/inertial_unit.h>
#include <webots/gps.h>
#include <webots/supervisor.h>

typedef struct 
{
    const double *imu_val;
}wb_imu_data_t;

typedef struct
{
    const double *gps_val;
}wb_gps_data_t;

typedef struct
{
    WbNodeRef node_name;
    const double *node_vel;
}wb_spvs_data_t;

extern wb_imu_data_t wb_imu_data;
extern wb_gps_data_t wb_gps_data;
extern wb_spvs_data_t wb_spvs_data;

void Read_Roll_Pitch_Yaw(WbDeviceTag Tag, wb_imu_data_t *wb_imu_data);
const double Get_IMU_Roll();
const double Get_IMU_Pitch();
const double Get_IMU_Yaw();

void Read_GPS_Val(WbDeviceTag Tag, wb_gps_data_t *);
const double Get_GPS_X();
const double Get_GPS_Y();
const double Get_GPS_Z();
const double Get_GPS_X_mm();
const double Get_GPS_Y_mm();
const double Get_GPS_Z_mm();

void Read_SPVS_Val(wb_spvs_data_t *wb_spvs_data);
const double Get_SPVS_X_Vel();
const double Get_SPVS_Y_Vel();
const double Get_SPVS_X_Vel_mm();
const double Get_SPVS_Y_Vel_mm();
const double Get_SPVS_Z_Angular_Vel();

#endif