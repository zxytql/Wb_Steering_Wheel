#ifndef _MAIN_H
#define _MAIN_H

typedef struct
{
    const double *gps_value;
    const double *imu_value;
}robot_sensor_data_t;

extern robot_sensor_data_t robot_sensor_data;
#endif