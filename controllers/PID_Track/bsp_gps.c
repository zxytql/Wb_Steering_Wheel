#include "bsp_gps.h"

wb_gps_data_t wb_gps_data;

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