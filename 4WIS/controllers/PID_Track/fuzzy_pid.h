#ifndef __FUZZYPID_H
#define __FUZZYPID_H

typedef enum fzy_tab_val
{
	nb = -3,
	nm = -2,
	ns = -1,
	zo = 0,
	ps = 1,
	pm = 2,
	pb = 3,
}fzy_tab_e;

//模糊pid结构体定义
typedef struct fzy_pid
{
	float kp;
	float kp_down;
	float kp_up;
	float kp_step;
	
	float ki;
	float ki_down;
	float ki_up;
	float ki_step;
	
	float kd;
	float kd_down;
	float kd_up;
	float kd_step;
		
	float error;
	float last_error;
	float errorc;
	
	float error_down;
	float error_up;
	float error_step;
	
	float errorc_down;
	float errorc_up;
	float errorc_step;
	
	float ref;
	float fdb;
	
	float iout;
	float out;
	
	float inter_limit;
	float out_limit;
}fzy_pid_t;

void fuzzy_init(fzy_pid_t *ptr, float error_down, float error_up, float errorc_down, float errorc_up, float kp_down, float kp_up, 
	              float ki_down, float ki_up, float kd_down, float kd_up, float inter_limit, float out_limit);
float fuzzypid_cal(fzy_pid_t *ptr, float ec, float ref, float fdb);
#endif


