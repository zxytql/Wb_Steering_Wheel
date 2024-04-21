#include "PID_Nav.h"
#include "bsp_spvs.h"
#include "bsp_gps.h"
#include "stdio.h"
#include "bsp_imu.h"
#include "chassis.h"

pid_nav_t pid_nav;
fzy_pid_t fzy_pid_x;
fzy_pid_t fzy_pid_y;
fzy_pid_t fzy_pid_v;
fzy_pid_t fzy_pid_w;

void PID_Nav_Init(pid_nav_t *pid_nav, fzy_pid_t *fzy_ptr_x, fzy_pid_t *fzy_ptr_y, fzy_pid_t *fzy_ptr_w, fzy_pid_t *fzy_ptr_v)
{
    //             ptr         e_down  e_up           ec_down  ec_up  kp_d kp_up               ki             kd       i_l                  out_limit 
    fuzzy_init(fzy_ptr_x,-250.0f, 250.0f, -100.0f, 100.0f, 15.0f / 1000.0f, 18.0f / 1000.0f, 0.0f,0.0f,  12.0f / 1000.0f,15.0f / 1000.0f, 0.0f, 2000.0f);
    fuzzy_init(fzy_ptr_w,-250.0f, 250.0f, -100.0f, 100.0f, 15.0f, 18.0f, 1.0f,2.0f,  12.0f,15.0f, 10.0f, 50.0f); //航向角不需要转换倍率
    fuzzy_init(fzy_ptr_y,-250.0f, 250.0f, -100.0f, 100.0f, 15.0f / 1000.0f, 18.0f / 1000.0f, 0.0f,0.0f,  12.0f / 1000.0f,15.0f / 1000.0f, 0.0f, 2000.0f);    
    fuzzy_init(fzy_ptr_v,-250.0f, 250.0f, -100.0f, 100.0f, 15.0f / 1000.0f, 18.0f / 1000.0f, 0.0f,0.0f,  12.0f / 1000.0f,15.0f / 1000.0f, 0.0f, 500.0f);   
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

// vel_t PID_Nav_Func(pid_nav_t *pid_nav, fzy_pid_t *fzy_ptr1, fzy_pid_t *fzy_ptr2, path_t *path)
// {
//     static vel_t nav_vel;
//     static vel_t fzy_cal_vel;
//     static float dis, x_dis, y_dis;
//     static float x_k, y_k; //计算X Y方向的速度分量系数
//     static float dc; //位置的变化率 即是速度
//     if (pid_nav->nav_state == NAV_PATH_DONE)
//     {
//         nav_vel.vx = 0;
//         nav_vel.vy = 0;
//         return nav_vel;
//     }
//     if (pid_nav->nav_state == NAV_TRACK_FINAL)
//     {
//         nav_vel = PID_Nav_Final_Track(pid_nav,&fzy_pid_x,&fzy_pid_y,path);
//     }
    
//     if (pid_nav->point_index == 0) //设置误差圆大小
//     {
//         pid_nav->err_cir_x = 100.0f;
//         pid_nav->err_cir_y = 100.0f;
//     }

//     if (pid_nav->point_index == PATH_1_NUM_DEF-2) //跟踪完前K-1个点 最后一个点用点跟踪
//     {
//         pid_nav->nav_state = NAV_TRACK_FINAL;
//     } 

//     if (pid_nav->nav_state == NAV_PATH_NOT_DONE && pid_nav->track_state == TRACK_POINT_DONE) //路径未跟踪完毕且已跟踪到上一个点
//     {
//         pid_nav->point_index++;
//         pid_nav->track_state = TRACK_POINT_NOT_DONE; 
//     }
    
//     if (pid_nav->nav_state == NAV_PATH_NOT_DONE)
//     {
//         nav_vel.ang_w = Navigation_Angle_Ctrl(Get_IMU_Yaw(),path[pid_nav->point_index].theta,4.0f,4.0f);
//         pid_nav->x_set = path[pid_nav->point_index].x;
//         pid_nav->y_set = path[pid_nav->point_index].y;
//         pid_nav->v_plan = path[pid_nav->point_index].plan_v;
        
//         x_dis = pid_nav->x_set - Get_GPS_X_mm();
//         y_dis = pid_nav->y_set - Get_GPS_Y_mm();
//         dis = sqrt(pow(x_dis,2) + pow(y_dis,2));
//         x_k = x_dis / dis;
//         y_k = y_dis / dis;
        

//         // fzy_cal_vel.vx = fuzzypid_cal(fzy_ptr1,Get_SPVS_X_Vel_mm(), pid_nav->x_set, Get_GPS_X_mm());
//         // fzy_cal_vel.vy = fuzzypid_cal(fzy_ptr2,Get_SPVS_Y_Vel_mm(), pid_nav->y_set, Get_GPS_Y_mm());
//         //dc = sqrt(pow(fzy_cal_vel.vx,2) + pow(fzy_cal_vel.vy,2));

//         pid_nav->v_set = pid_nav->v_plan + fuzzypid_cal(fzy_ptr1,0,0.001f * dis,0);
//         nav_vel.vx = pid_nav->v_set * x_k;
//         nav_vel.vy = pid_nav->v_set * y_k;

//         printf("dis = %f, dc = %f, v_set = %f \n",dis,dc,pid_nav->v_set);
        
//     }

//     if (fabs(pid_nav->x_set - Get_GPS_X_mm()) <= pid_nav->err_cir_x && fabs(pid_nav->y_set - Get_GPS_Y_mm()) <= pid_nav->err_cir_y)
//     {
//         //printf("Err_x = %f, Err_y = %f \n",fabs(pid_nav->x_set - Get_GPS_X_mm()),fabs(pid_nav->y_set - Get_GPS_Y_mm()));
//         pid_nav->track_state = TRACK_POINT_DONE;
//     }
//     else
//     {
//         pid_nav->track_state = TRACK_POINT_NOT_DONE; 
//     }
//     printf("x_set = %f, y_set = %f \n",pid_nav->x_set,pid_nav->y_set);
//     return nav_vel;
// }

float Navigation_Angle_Ctrl(float anglePresent,float angleTarget,float kp,float kd)
{
	float angleErr = 0.0f,angularVel = 0.0f, angularVelErr = 0.0f;
	static float lastAngleErr = 0.0f, lastAngledTerm = 0.0f;
	float dTerm = 0.0f,dTermFliter = 0.0f;
	//PD控制器
	//目标角度减去当前角度
	angleErr = (angleTarget - anglePresent);
	dTerm = (angleErr - lastAngleErr);
	//低通滤波
	dTermFliter = 0.5f*dTerm + 0.5f*lastAngledTerm;

	angularVel = angleErr * kp + dTermFliter * kd;
	
	lastAngledTerm = dTerm;
	lastAngleErr = angleErr;
	
	angularVelErr = angularVel - Get_SPVS_Z_Angular_Vel();
	angularVel = angularVel + angularVelErr *0.2f;
	
	if(angularVel>90.0f)
	{
		angularVel = 90.0f;
	}
	else if(angularVel<-90.0f)
	{
		angularVel = -90.0f;
	}
	
	return angularVel;
}

// vel_t PID_Nav_Final_Track(pid_nav_t *pid_nav, fzy_pid_t *fzy_ptr, fzy_pid_t *fzy_ptr2, path_t *path)
// {
//     static vel_t vel_f;
//     pid_nav->x_set = path[pid_nav->point_index-1].x;
//     pid_nav->y_set = path[pid_nav->point_index-1].y;
//     pid_nav->err_cir_x = 50.0f;
//     pid_nav->err_cir_y = 50.0f;

//     if (fabs(pid_nav->x_set - Get_GPS_X_mm()) <= pid_nav->err_cir_x && fabs(pid_nav->y_set - Get_GPS_Y_mm()) <= pid_nav->err_cir_y)
//     {
//         pid_nav->nav_state = NAV_PATH_DONE;
//         vel_f.vx = 0;
//         vel_f.vy = 0;
//         return vel_f;
//     }
//     else
//     {
//         vel_f.vx = fuzzypid_cal(fzy_ptr,Get_SPVS_X_Vel_mm(),pid_nav->x_set,Get_GPS_X_mm());
//         vel_f.vy = fuzzypid_cal(fzy_ptr2,Get_SPVS_Y_Vel_mm(),pid_nav->y_set,Get_GPS_Y_mm());
//         return vel_f;
//     }

// }

void PID_Nav_Point_Handler(pid_nav_t *pid_nav, path_t *path)
{
    if (pid_nav->point_index == 0) //开始追踪 设定误差圆 给定第一个目标点
    {
        pid_nav->err_cir_x = 100.0f;
        pid_nav->err_cir_y = 100.0f;

        pid_nav->x_set = path[0].x;
        pid_nav->y_set = path[0].y;
        pid_nav->tgt_theta = path[0].theta;
        pid_nav->v_plan = path[0].plan_v;       
    }
    
    //每次进入该函数就判断该点有没有跟踪到 若跟踪到 则切换到下一个点 若无则不切换
    if (fabs(pid_nav->x_set - Get_GPS_X_mm()) <= pid_nav->err_cir_x && fabs(pid_nav->y_set - Get_GPS_Y_mm()) <= pid_nav->err_cir_y)
    {
        //printf("Err_x = %f, Err_y = %f \n",fabs(pid_nav->x_set - Get_GPS_X_mm()),fabs(pid_nav->y_set - Get_GPS_Y_mm()));
        if (pid_nav->point_index < PATH_1_NUM_DEF - 1) //有K个点 数组下标K-1即是最后一个点 
        {
            pid_nav->point_index++;
            //更新点追踪器数据
            pid_nav->x_set = path[pid_nav->point_index].x;
            pid_nav->y_set = path[pid_nav->point_index].y;
            pid_nav->tgt_theta = path[pid_nav->point_index].theta;
            pid_nav->v_plan = path[pid_nav->point_index].plan_v;
        }
        else
        {
            //最后一个点也跟踪到了 微调到最后一个点
            //微调时不需要速度规划 直接点对点用模糊PID跟踪
            //printf("Tracking final?? \n");
            pid_nav->nav_state = NAV_TRACK_FINAL;
        }
        
    }
    
}

vel_t PID_Nav_Point_Tracker(pid_nav_t *pid_nav, fzy_pid_t *fzy_ptr_x, fzy_pid_t *fzy_ptr_y, fzy_pid_t *fzy_pid_v, path_t *path)
{
    static vel_t nav_vel;
    static float x_dis, y_dis, dis, x_k, y_k;
    
    
    //有速度规划的路径跟踪
    if (pid_nav->nav_state == NAV_PATH_NOT_DONE)
    {
        //PD角度环
        //nav_vel.ang_w = Navigation_Angle_Ctrl(Get_SPVS_Z_Angular_Vel(), pid_nav->tgt_theta, 4.0f, 4.0f);
        //printf("Now tgt_theta: %f , nav_w = %f \n",pid_nav->tgt_theta,nav_vel.ang_w);
        x_dis = pid_nav->x_set - Get_GPS_X_mm();
        y_dis = pid_nav->y_set - Get_GPS_Y_mm();
        dis = sqrt(pow(x_dis,2) + pow(y_dis,2));
        x_k = x_dis / dis;
        y_k = y_dis / dis;   
        pid_nav->v_set = pid_nav->v_plan + fuzzypid_cal(fzy_pid_v,0,0.001f * dis,0);

        //模糊PID位置环
        nav_vel.vx = pid_nav->v_set * x_k;
        nav_vel.vy = pid_nav->v_set * y_k;

        printf("Now tracking point: %d; Total is: %d \n", pid_nav->point_index + 1, PATH_1_NUM_DEF);
    }
    
    //无速度规划的点对点跟踪 用于跟踪最后一个点 提高跟踪精度
    if (pid_nav->nav_state == NAV_TRACK_FINAL)
    {
        nav_vel.vx = fuzzypid_cal(fzy_ptr_x,Get_SPVS_X_Vel_mm(),pid_nav->x_set, Get_GPS_X_mm());
        nav_vel.vy = fuzzypid_cal(fzy_ptr_y,Get_SPVS_Y_Vel_mm(),pid_nav->y_set, Get_GPS_Y_mm());
    }
    
    return nav_vel;
}

// void PID_Nav_First_Aim(pid_nav_t *pid_nav, helm_chassis_t* chassis, path_t *path)
// {
//     static float ang;
//     //导航开始前 旋转舵轮瞄准第一个点
//     pid_nav->x_set = path[0].x;
//     pid_nav->y_set = path[0].y;
//     ang = atan2f(pid_nav->y_set - Get_GPS_Y_mm(),pid_nav->x_set - Get_GPS_X_mm());

// }