#include "chassis.h"
#include "stdint.h"

/* Global Variables */
helm_chassis_t helm_chassis;

/* Functions */
void Chassis_Wb_Init(void)
{

}

void Chassis_Self_Spin(void)
{
   
}

uint16_t Helm_Chassis_Init(void)
{
    helm_chassis.wheel1.VN2X = -135;
    helm_chassis.wheel1.angle_actual = 0;
    helm_chassis.wheel1.angle_target = 0;
    helm_chassis.wheel1.v_target = 0;

    helm_chassis.wheel2.VN2X = -45;
    helm_chassis.wheel2.angle_actual = 0;
    helm_chassis.wheel2.angle_target = 0;
    helm_chassis.wheel2.v_target = 0;

    helm_chassis.wheel3.VN2X = 45;
    helm_chassis.wheel3.angle_actual = 0;
    helm_chassis.wheel3.angle_target = 0;
    helm_chassis.wheel3.v_target = 0;

    helm_chassis.wheel4.VN2X = 135;
    helm_chassis.wheel4.angle_actual = 0;
    helm_chassis.wheel4.angle_target = 0;
    helm_chassis.wheel4.v_target = 0;

    return 1;
}

void Helm_Chassis_Ctrl(float vx_input, float vy_input, float wz_input, helm_chassis_t *chassis)
{
	static float vx;
	static float vy;
	//chassis_ptr->dir_chassis = -ops_data.val[0];
	switch(chassis->chassis_cor)
	{
		case CHASSIS_COORDINATE:
			vx = vx_input;
			vy = vy_input;
			break;
		case GLOBAL_COORDINATE:
			vx = vx_input*cosf(ANGLE2RAD(chassis_ptr->dir_chassis)) - vy_input*sinf(ANGLE2RAD(chassis_ptr->dir_chassis));
			vy = vy_input*cosf(ANGLE2RAD(chassis_ptr->dir_chassis)) + vx_input*sinf(ANGLE2RAD(chassis_ptr->dir_chassis));
			break;
		case LOCK_CHASSIS:
			vx = 0;
			vy = 0;
			break;
	}  
}