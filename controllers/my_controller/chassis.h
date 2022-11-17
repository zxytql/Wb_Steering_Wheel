#ifndef _CHASSIS_H
#define _CHASSIS_H

#include <webots/motor.h>

typedef struct
{
    float ct_vx;
    float ct_vy;
    float ct_wz;
}chassis_ct_t;

typedef struct 
{
    float ct_fdb_vx;
    float ct_fdb_vy;
    float ct_fdb_wz;
}chassis_fdb_ct_t;

typedef struct
{
    float angle_actual;
    float angle_target;
    float VN2X; //轮系线速度与X轴的夹角 -180° ~ 180°
    float v_target; //期望
}helm_wheel_t;

typedef enum
{
	GLOBAL_COORDINATE,	//基于全局坐标系控制
	CHASSIS_COORDINATE,	//基于机器人底盘坐标系控制
	LOCK_CHASSIS
}chassis_cor_t;

typedef struct
{
    chassis_ct_t chassis_ct;
    chassis_fdb_ct_t chassis_fdb_ct;
    helm_wheel_t wheel1;
    helm_wheel_t wheel2;
    helm_wheel_t wheel3;
    helm_wheel_t wheel4;
    
    float dir_chassis;

    chassis_cor_t chassis_cor;
}helm_chassis_t;

/** Extern **/
extern helm_chassis_t helm_chassis;

/* Function */
void Chassis_Wb_Init(void);
void Chassis_Self_Spin(void);
uint16_t Helm_Chassis_Init(void);
void Helm_Chassis_Ctrl(float, float, float, helm_chassis_t*);
#endif
