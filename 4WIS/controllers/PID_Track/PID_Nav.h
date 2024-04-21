#ifndef PID_NAV_H
#define PID_NAV_H

#include "stdint.h"
#include "fuzzy_pid.h"
#include "path.h"

#define NAV_PATH_DONE (10)
#define NAV_PATH_NOT_DONE (11)
#define NAV_TRACK_FINAL (12)

#define TRACK_POINT_DONE (20)
#define TRACK_POINT_NOT_DONE (21)

typedef struct
{
    float x_set;
    float y_set;
    float theta_set;

    float v_set;
    float v_plan;

    float tgt_theta;
    float now_theta;

    uint8_t path_index;
    uint8_t point_index;
    uint8_t track_state; //点跟踪状态 DONE/NOT DONE
    uint8_t nav_state;   //轨迹跟踪状态 DONE/ NOT DONE

    float err_cir_x;
    float err_cir_y;

    float vx_limit;
    float vy_limit;

    // point_t now_point;
    // point_t plan_point;
}pid_nav_t;


typedef struct 
{
    float x;
    float y;
}point_t;

typedef struct 
{
    float vx;
    float vy;
    float ang_w;
}vel_t;

extern pid_nav_t pid_nav;
extern fzy_pid_t fzy_pid_x;
extern fzy_pid_t fzy_pid_y;
extern fzy_pid_t fzy_pid_v;
extern fzy_pid_t fzy_pid_w;
void PID_Nav_Init(pid_nav_t *pid_nav, fzy_pid_t *fzy_ptr_x, fzy_pid_t *fzy_ptr_y, fzy_pid_t *fzy_ptr_w, fzy_pid_t *fzy_ptr_v);

void PID_Nav_Point_Handler(pid_nav_t *pid_nav, path_t *path);
vel_t PID_Nav_Point_Tracker(pid_nav_t *pid_nav, fzy_pid_t *fzy_ptr_x, fzy_pid_t *fzy_ptr_y, fzy_pid_t *fzy_pid_v, path_t *path);
float Navigation_Angle_Ctrl(float anglePresent,float angleTarget,float kp,float kd);
#endif