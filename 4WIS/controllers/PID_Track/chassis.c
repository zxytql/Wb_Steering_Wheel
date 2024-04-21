#include "chassis.h"
#include "stdint.h"
#include <webots/inertial_unit.h>
#include <stdio.h>

/* Global Variables */
helm_chassis_t helm_chassis;

/* Functions */
void Chassis_Wb_Init(void)
{

}

void Chassis_Self_Spin(void)
{
   
}

int Helm_Chassis_Init(void)
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

	helm_chassis.chassis_cor = CHASSIS_COORDINATE;
    return 1;
}

void Helm_Chassis_Ctrl(float vx_input, float vy_input, float wz_input, helm_chassis_t *chassis, float yaw)
{
	static float vx;
	static float vy;
	chassis->dir_chassis = yaw;
	//printf("In_func_yaw is %f \n",chassis->dir_chassis);
	switch(chassis->chassis_cor)
	{
		case CHASSIS_COORDINATE:
			vx = vx_input;
			vy = vy_input;
			break;
		case GLOBAL_COORDINATE:
			vx = vx_input*cosf(chassis->dir_chassis) + vy_input*sinf(chassis->dir_chassis);
			vy = vy_input*cosf(chassis->dir_chassis) - vx_input*sinf(chassis->dir_chassis);
			//printf("vx_in = %f, vx_out = %f",vx_input,vx);
			break;
		case LOCK_CHASSIS:
			vx = 0;
			vy = 0;
			break;
	}  
	if(fabsf(vx) <= 0.001f && fabsf(vy) <= 0.001f && fabsf(wz_input) <= 0.1f)
	{
		chassis->wheel1.v_target = 0;
		chassis->wheel2.v_target = 0;
		chassis->wheel3.v_target = 0;
		chassis->wheel4.v_target = 0;
	}
	else
	{
		Helm_Wheel_Ctrl(vx, vy, wz_input, &chassis->wheel1);
		Helm_Wheel_Ctrl(vx, vy, wz_input, &chassis->wheel2);
		Helm_Wheel_Ctrl(vx, vy, wz_input, &chassis->wheel3);
		Helm_Wheel_Ctrl(vx, vy, wz_input, &chassis->wheel4);
	}
}

void Helm_Wheel_Ctrl(float vx, float vy, float wz, helm_wheel_t *wheel_ptr)
{
	float vx_wheel = 0;
	float vy_wheel = 0;
	float VN = 0;
	float v_target = 0;
	float diraction = 0;
	
	VN = ANGLE2RAD(wz) * DIS_WHEEL2CENTER;
	
	vx_wheel = vx + VN*cosf(ANGLE2RAD(wheel_ptr->VN2X));
	vy_wheel = vy + VN*sinf(ANGLE2RAD(wheel_ptr->VN2X));
	
	v_target = sqrtf(vx_wheel*vx_wheel + vy_wheel*vy_wheel);
	wheel_ptr->v_target = v_target;
	diraction = atan2f(vy_wheel, vx_wheel);
	diraction = RAD2ANGLE(diraction) - 90.0f;
	Angle_Limit(&diraction);
	V_Dir2Wheel_Angle(wheel_ptr, diraction);
}

void V_Dir2Wheel_Angle(helm_wheel_t *wheel, float dir)
{
	float err_angle;
	float now_angle = wheel->angle_actual;
	Angle_Limit(&now_angle);

	err_angle = dir - now_angle;
	Angle_Limit(&err_angle);//获得目标方向相对于当前轮系方向的角度
	if(fabsf(err_angle) > 90.0f) //取劣弧
	{
		wheel->v_target = -wheel->v_target;
		if(err_angle > 0)
		{
			err_angle = err_angle - 180.0f;
		}
		else
		{
			err_angle = err_angle + 180.0f;
		}
	}	

	wheel->angle_target = wheel->angle_actual + err_angle;
	wheel->angle_actual = wheel->angle_target;
}

void Angle_Limit(float *angle)
{
	if(*angle>180.0f)
	{
		*angle-=360.0f;
		Angle_Limit(angle);
	}
	else if(*angle<=-180.0f)
	{
		*angle+=360.0f;
		Angle_Limit(angle);
	}
}

// Pose_t Get_Pos_Present()
// {
// 	Pose_t pos;
// 	pos.point.x = GetX();
// 	pos.point.y = GetY();
// 	pos.direction   = GetAngle();
// 	pos.vel = sqrt(GetSpeedWithoutOmega().xVel*GetSpeedWithoutOmega().xVel + GetSpeedWithoutOmega().yVel*GetSpeedWithoutOmega().yVel);
// 	return pos;	
// }

void Helm_Evaluation(helm_chassis_t *chassis)
{
	// float s1 = -(cosf(chassis->wheel1.rad_actual) / DIS_WHEEL2CENTER);
	// float s2 = (cosf(chassis->wheel2.rad_actual) - SQRT3 * sinf(chassis->wheel2.rad_actual)) / 2 / DIS_WHEEL2CENTER;
	// float s3 = (cosf(chassis->wheel3.rad_actual) + SQRT3 * sinf(chassis->wheel3.rad_actual)) / 2 / DIS_WHEEL2CENTER;

	// 1 是 LF
	float s1 = (BODY_W * sinf(chassis->wheel1.rad_actual) - BODY_L * cosf(chassis->wheel1.rad_actual)) / (BODY_L2 + BODY_W2) * chassis->wheel1.v_target;
	float s2 = (-BODY_W * sinf(chassis->wheel2.rad_actual) - BODY_L * cosf(chassis->wheel2.rad_actual)) / (BODY_L2 + BODY_W2) * chassis->wheel2.v_target;
	float s3 = (-BODY_W * sinf(chassis->wheel3.rad_actual) + BODY_L * cosf(chassis->wheel3.rad_actual)) / (BODY_L2 + BODY_W2) * chassis->wheel3.v_target;
	float s4 = (BODY_W * sinf(chassis->wheel4.rad_actual) + BODY_L * cosf(chassis->wheel4.rad_actual)) / (BODY_L2 + BODY_W2) * chassis->wheel4.v_target;
	float cal_vx, cal_vy, cal_wz;
	
	cal_vx = (chassis->wheel1.v_target * cosf(chassis->wheel1.rad_actual) + 
				chassis->wheel2.v_target * cosf(chassis->wheel2.rad_actual) + 
				chassis->wheel3.v_target * cosf(chassis->wheel3.rad_actual) + 
				chassis->wheel4.v_target * cosf(chassis->wheel4.rad_actual)) / 4;
	
	cal_vy = (chassis->wheel1.v_target * sinf(chassis->wheel1.rad_actual) + 
				chassis->wheel2.v_target * sinf(chassis->wheel2.rad_actual) + 
				chassis->wheel3.v_target * sinf(chassis->wheel3.rad_actual) + 
				chassis->wheel4.v_target * sinf(chassis->wheel4.rad_actual)) / 4;

	cal_wz = (s1 + s2 + s3 + s4) / 4;
	
	printf("s1 = %f, s2 = %f, s3 = %f, s4 = %f \n", s1, s2, s3, s4);
	printf("*******???W = %f \n", chassis->wheel1.v_target / DIS_WHEEL2CENTER * 4);
	printf("v1 = %f, v2 = %f, v3 = %f, v4 = %f \n", chassis->wheel1.v_target, chassis->wheel2.v_target, chassis->wheel3.v_target, chassis->wheel4.v_target);
	printf("Calculate vx: %f, vy: %f, wz: %f \n", cal_vx, cal_vy, cal_wz);
}	