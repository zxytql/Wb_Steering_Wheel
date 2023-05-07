#include "PID_Nav.h"
#include "bsp_spvs.h"
#include "bsp_gps.h"
#include "stdio.h"

pid_nav_t pid_nav;
fzy_pid_t fzy_pid_x;
fzy_pid_t fzy_pid_y;

void PID_Nav_Init(pid_nav_t *pid_nav, fzy_pid_t *fzy_ptr1, fzy_pid_t *fzy_ptr2)
{
    //             ptr         e_down  e_up           ec_down  ec_up  kp_d kp_up               ki             kd       i_l                  out_limit 
    fuzzy_init(fzy_ptr1,-250.0f, 250.0f, -100.0f, 100.0f, 15.0f / 1000.0f, 18.0f / 1000.0f, 0.0f,0.0f,  10.0f / 1000.0f,12.0f / 1000.0f, 0.0f, 50.0f);
    fuzzy_init(fzy_ptr2,-250.0f, 250.0f, -100.0f, 100.0f, 15.0f / 1000.0f, 18.0f / 1000.0f, 0.0f,0.0f,  10.0f / 1000.0f,12.0f / 1000.0f, 0.0f, 50.0f);    
    //1000是将m -> mm的倍率

    pid_nav->nav_state = NAV_PATH_NOT_DONE;
    pid_nav->track_state = TRACK_POINT_NOT_DONE;

    pid_nav->err_cir_x = 0;
    pid_nav->err_cir_y = 0;
    pid_nav->path_index = 0;
    pid_nav->point_index = 0;

    pid_nav->now_theta = 0;
    pid_nav->vx_limit = 50.0f;
    pid_nav->vy_limit = 50.0f;
}

vel_t PID_Nav_Func(pid_nav_t *pid_nav, fzy_pid_t *fzy_ptr1, fzy_pid_t *fzy_ptr2, path_t *path)
{
    static vel_t nav_vel;
    if (pid_nav->point_index == 0) //设置误差圆大小
    {
        pid_nav->err_cir_x = 100.0f;
        pid_nav->err_cir_y = 100.0f;
    }

    if (pid_nav->point_index == PATH_1_NUM_DEF) //跟踪完最后一个点 跳出index递增 否则会出现数组越界
    {
        nav_vel.vx = fuzzypid_cal(fzy_ptr1,Get_SPVS_X_Vel() * 1000.0f, pid_nav->x_set, Get_GPS_X_mm());
        nav_vel.vy = fuzzypid_cal(fzy_ptr2,Get_SPVS_Y_Vel() * 1000.0f, pid_nav->y_set, Get_GPS_Y_mm());
        pid_nav->nav_state = NAV_PATH_DONE;
    } 

    if (pid_nav->nav_state != NAV_PATH_DONE && pid_nav->track_state == TRACK_POINT_DONE) //路径未跟踪完毕且已跟踪到上一个点
    {
        pid_nav->point_index++;
        pid_nav->track_state = TRACK_POINT_NOT_DONE; 
    }
    
    if (pid_nav->nav_state == NAV_PATH_NOT_DONE)
    {
        pid_nav->x_set = path[pid_nav->point_index].x;
        pid_nav->y_set = path[pid_nav->point_index].y;
        nav_vel.vx = fuzzypid_cal(fzy_ptr1,Get_SPVS_X_Vel() * 1000.0f, pid_nav->x_set, Get_GPS_X_mm());
        nav_vel.vy = fuzzypid_cal(fzy_ptr2,Get_SPVS_Y_Vel() * 1000.0f, pid_nav->y_set, Get_GPS_Y_mm());
    }

    if (fabs(pid_nav->x_set - Get_GPS_X_mm()) <= pid_nav->err_cir_x && fabs(pid_nav->y_set - Get_GPS_Y_mm()) <= pid_nav->err_cir_y)
    {
        printf("Err_x = %f, Err_y = %f \n",fabs(pid_nav->x_set - Get_GPS_X_mm()),fabs(pid_nav->y_set - Get_GPS_Y_mm()));
        pid_nav->track_state = TRACK_POINT_DONE;
    }
    else
    {
        pid_nav->track_state = TRACK_POINT_NOT_DONE; 
    }
    return nav_vel;
}