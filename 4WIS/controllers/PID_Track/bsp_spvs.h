#ifndef BSP_SPVS_H
#define BSP_SPVS_H

#include <webots/supervisor.h>

typedef struct
{
    WbNodeRef node_name;
    const double *node_vel;
}wb_spvs_data_t;

extern wb_spvs_data_t wb_spvs_data;

void Read_SPVS_Val(wb_spvs_data_t *wb_spvs_data);
const double Get_SPVS_X_Vel();
const double Get_SPVS_Y_Vel();
const double Get_SPVS_X_Vel_mm();
const double Get_SPVS_Y_Vel_mm();
const double Get_SPVS_Z_Angular_Vel();
#endif