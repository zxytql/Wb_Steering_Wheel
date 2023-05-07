#ifndef BSP_GPS_H
#define BSP_GPS_H

#include <webots/gps.h>

typedef struct
{
    const double *gps_val;
}wb_gps_data_t;

extern wb_gps_data_t wb_gps_data;

void Read_GPS_Val(WbDeviceTag Tag, wb_gps_data_t *);
const double Get_GPS_X();
const double Get_GPS_Y();
const double Get_GPS_Z();
const double Get_GPS_X_mm();
const double Get_GPS_Y_mm();
const double Get_GPS_Z_mm();
#endif