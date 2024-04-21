#include "gps_data.h"
#include <stdio.h>
#include <math.h>
#include "chassis.h"
#include "calculate.h"
#include "Bspline.h"

static float walk_length = 0.0f;

static _Bool cal_length_flag = 0;

float pos_x_old = 0.0f;
float pos_y_old = 0.0f;


float Get_Now_X(void)
{
    return robot_sensor_data.gps_value[0];
}

float Get_Now_Y(void)
{
    return robot_sensor_data.gps_value[1];
}

float Get_Now_Angle(void)
{
    return robot_sensor_data.imu_value[2];
}

void Cal_Walk_Length(void)
{
    float err = -0.02f;
    float err_dir = 0.0f;
    float reduce_path = 0.0f;
    PointU_t virtual_pos = {0.0f};
    
    if (cal_length_flag == 1)       
    {
        err = sqrt((Get_Now_X() - pos_x_old) * (Get_Now_X() - pos_x_old) + (Get_Now_Y() - pos_y_old) * (Get_Now_Y() - pos_y_old));
        if(err > 0.2f)
        {
            err_dir = RAD2ANGLE(atan2f((Get_Now_Y() - pos_y_old), (Get_Now_X() - pos_x_old)));
            virtual_pos = SerchVirtualPoint2(Get_Walk_Path_Length());
            err = err * cosf(ANGLE2RAD(err_dir - virtual_pos.direction));

            if(err >= 0.0f)
            {
                if (err < 300.0f)
                {
                    walk_length += err;
                }
                else
                {
                    walk_length += err;
                    printf("ERR IS TO BIG : %d \n",err);
                    if(fabs(err) > 1000.0f)
                    {
                        printf("ERR IS BIG THAN 1000: %d \n",err);
                    }
                }
            }
            if(walk_length <= 0.0f)
            {
                walk_length = 0.0f;
            }
            pos_x_old = Get_Now_X();
            pos_y_old = Get_Now_Y();
        }
    }
    
}

void Update_Length_Start(void)
{
    cal_length_flag = 1;
}

void Update_Length_Stop(void)
{
    cal_length_flag = 0;
}

float Get_Walk_Path_Length(void)
{
    return walk_length;
}

float Get_Speed_X(void)
{
    return robot_sensor_data.velocity[0];
}

float Get_Speed_Y(void)
{
    return robot_sensor_data.velocity[1];
}

void Add_Path_Length(float dis)
{
    walk_length += dis;
}

void Reduce_Path_Length(float dis)
{
    walk_length -= dis;
}

float Get_AngularSpeed_W(void)
{
    return robot_sensor_data.velocity[5];
    
}
Pose_t Get_Pos_Present()
{
    Pose_t pos;
    pos.point.x = Get_Now_X();
    pos.point.y = Get_Now_Y();
    pos.direction = Get_Now_Angle();
    pos.vel = sqrt(Get_Speed_X() * Get_Speed_X() + Get_Speed_Y() * Get_Speed_Y());
    return pos;
}

