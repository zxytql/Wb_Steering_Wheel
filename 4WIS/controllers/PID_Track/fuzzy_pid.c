/*
@breif 模糊pid的实现
@usage 创建一个fzy_pid_t结构体，使用fuzzy_init初始化参数，使用fuzzypid_cal进行计算
@author hbut.pfy
@vision 2020.12.13   v1.0
*/
#include "fuzzy_pid.h"
#include "stdint.h"
			/********ec --->*********/
			/*   nb nm ns zo ps pm pb
			*** nb  
			*** nm
			e** ns    
			|** zo
			|** ps
			v** pm
			*** pb
			*/
//模糊pid参数表

//int pfuzzy_table[7][7]=
//{
//{pb, pb, pm, pm, ps, zo, zo},
//{pb, pb, pm, ps, ps, zo, ns},
//{pm, pm, pm, ps, zo, ns, ns},
//{pm, pm, ps, zo, ns, nm, nm},
//{ps, ps, zo, ns, ns, nm ,nm},
//{ps, zo, ns, nm, nm, nm, nb},
//{zo, zo, nm, nm, nm, nb, nb}
//};

//无超调p模糊表，自定义。
int pfuzzy_table[7][7]=
{
{pb, pm, ps, ps, pm, pm, ps},
{pm, ps, zo, zo, ps, ps, ps},
{ps, zo, ns, ns, zo, zo, zo},
{zo, ns, nm, nb, nm, ns, zo},
{zo, zo, zo, ns, ns, zo ,ps},
{ps, ps, ps, zo, zo, ps, pm},
{ps, pm, pm, ps, ps, pm, pb}
};


int ifuzzy_table[7][7]=
{
{nb, nb, nm, nm, ns, zo, zo},
{nb, nb, nm, ns, ns, zo, zo},
{nb, nm, ns, ns, zo, ps, ps},
{nm, nm, ns, zo, ps, pm, pm},
{nm, ns, zo, ps, ps, pm, pb},
{zo, zo, ps, ps, pm, pb, pb},
{zo, zo, ps, pm, pm, pb, pb}
};

//int dfuzzy_table[7][7]=
//{
//{ps, ns, nb, nb, nb, nm, ps},
//{ps, ns, nb, nm, nm, ns, zo},
//{zo, ns, nm, nm, ns, ns, zo},
//{zo, ns, ns, ns, ns, ns, zo},
//{zo, zo, zo, zo, zo, zo, zo},
//{pb, ns, ps, ps, ps, ps, pb},
//{pb, pm, pm, pm, ps, ps, pb}
//};


//自定义d模糊表
int dfuzzy_table[7][7]=
{
{pb, pb, ps, nb, ns, zo, zo},
{pb, pb, ps, nm, zo, ps, ps},
{zo, zo, zo, ns, zo, ps, pm},
{pm, ps, zo, ns, zo, ps, pm},
{pm, ps, zo, ns, zo, zo, zo},
{ps, ps, zo, nm, ps, pb, pb},
{zo, zo, ns, nb, ps, pb, pb}
};

#define abs(x)   x>0 ? x : (-x)
#define limit(x, dowm, up)   x > up ? up : x < dowm ? dowm : x
//初始化函数
void fuzzy_init(fzy_pid_t *ptr, float error_down, float error_up, float errorc_down, float errorc_up, float kp_down, float kp_up, float ki_down, float ki_up, float kd_down, float kd_up, float inter_limit, float out_limit)
{
	ptr->kp = 0;
	ptr->kp_down = kp_down;
	ptr->kp_up = kp_up;
	ptr->kp_step = (kp_up - kp_down)*0.1666667f;
	
	ptr->ki = 0;
	ptr->ki_down = ki_down;
	ptr->ki_up = ki_up;
	ptr->ki_step = (ki_up - ki_down)*0.1666667f;
	
	ptr->kd = 0;
	ptr->kd_down = kd_down;
	ptr->kd_up = kd_up;
	ptr->kd_step = (kd_up - kd_down)*0.1666667f;
	
    ptr->error = 0;
	ptr->last_error = 0;
	ptr->errorc = 0;
	
	ptr->error_down = error_down;
	ptr->error_up = error_up;
	ptr->error_step = (error_up - error_down)*0.1666667f;
	
	ptr->errorc_down = errorc_down;
	ptr->errorc_up = errorc_up;
	ptr->errorc_step = (errorc_up - errorc_down)*0.1666667f;
	
	ptr->ref = 0;
	ptr->fdb = 0;
	
	ptr->iout = 0;
	ptr->out = 0;
	
	ptr->inter_limit = inter_limit;
    ptr->out_limit = out_limit;
	
}
//解模糊
float exp_fuzzy(int zs, int ys, int zx, int yx, float a, float b)
{
  return (zs * a * b + ys * a * (1.0f-b) + zx * (1.0f-a) * b + yx * (1.0f-a) * (1.0f-b));
}

//模糊化
void fuzzification(fzy_pid_t *ptr,float ec)
{
    int ep = 0;
	int ecp = 0;
	float epf = 0;
	float ecpf = 0;
	epf = limit(ptr->error, ptr->error_down, ptr->error_up) / ptr->error_step;
	ecpf = limit(ec, ptr->errorc_down, ptr->errorc_up) / ptr->errorc_step;
	ep = (int16_t)(epf + 3);
    ecp = (int16_t)(ecpf + 3);
	epf = limit(epf, -3, 3);
	ecpf = limit(ecpf, -3, 3);
	ep = limit(ep, 0, 5);
	ecp = limit(ecp, 0, 5);
	ptr->kp = ptr->kp_up*0.5f + ptr->kp_down*0.5f + ptr->kp_step * exp_fuzzy(pfuzzy_table[ep][ecp], pfuzzy_table[ep][ecp+1], pfuzzy_table[ep+1][ecp], pfuzzy_table[ep+1][ecp+1],1-( epf + 3 - ep), 1-(ecpf + 3 - ecp));
	ptr->ki = ptr->ki_up*0.5f + ptr->ki_down*0.5f + ptr->ki_step * exp_fuzzy(ifuzzy_table[ep][ecp], ifuzzy_table[ep][ecp+1], ifuzzy_table[ep+1][ecp], ifuzzy_table[ep+1][ecp+1], 1-(epf + 3 - ep), 1-(ecpf + 3 - ecp));
	ptr->kd = ptr->kd_up*0.5f + ptr->kd_down*0.5f + ptr->kd_step * exp_fuzzy(dfuzzy_table[ep][ecp], dfuzzy_table[ep][ecp+1], dfuzzy_table[ep+1][ecp], dfuzzy_table[ep+1][ecp+1], 1-(epf + 3 - ep), 1-(ecpf + 3 - ecp));
}

//pid计算
float fuzzypid_cal(fzy_pid_t *ptr, float ec,float ref, float fdb)
{
	ptr->ref = ref;
	ptr->fdb = fdb;
	ptr->last_error = ptr->error;
    ptr->error = ptr->ref - ptr->fdb;
	ptr->errorc = ptr->error - ptr->last_error;
	
	fuzzification(ptr,ec);
	
    ptr->iout += ptr->ki * ptr->error;
	ptr->iout = limit(ptr->iout, -ptr->inter_limit, ptr->inter_limit);
	ptr->out = ptr->kp * ptr->error + ptr->iout + ptr->kd * ptr->errorc;
	return (limit(ptr->out, -ptr->out_limit, ptr->out_limit));
}


