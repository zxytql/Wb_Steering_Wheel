#include "path_following.h"
#include "chassis.h"
#include "calculate.h"
#include "gps_data.h"
#include "Bspline.h"
#include "ringbuffer.h"
#include "stdio.h"

int Path_Following(float percent)
{
	static float vell = 150.0f;
	float velDir = 0.0f;
	float angularVel = 0.0f;
	float angleErr = 0.0f;
	float posAngleVP = 0.0f;
	float posAngleVT = 0.0f;
	float robotlen = 0.0f;
	float disRealPos2VirTarget = 0.0f;
	float disRealPos2VirPos = 0.0f;
	robotVel_t adjustVel = {0.0f};
	PointU_t virtualPos, virtualTarget;

    if (percent < 0.0f || percent > 1.2f)
    {
        printf("Invalid Para");
        return -1;
    }
    
    float VIEW_L = 0.0f;
    VIEW_L = Get_Pos_Present().vel;
	
	//获取实际行走长度
	robotlen = Get_Walk_Path_Length();
    
	//虚拟位置点
	virtualPos = SerchVirtualPoint(robotlen);

	//计算当前位置与虚拟位置点的直线距离
	disRealPos2VirPos = CalculatePoint2PointDistance(Get_Pos_Present().point,virtualPos.point);

	if(VIEW_L - disRealPos2VirPos >= 0.0f)
	{
		robotlen = Get_Walk_Path_Length() + VIEW_L - disRealPos2VirPos;
	}
	//求虚拟目标点
	virtualTarget = SerchVirtualPoint2(robotlen);

	//计算当前位置与虚拟目标点距离
	disRealPos2VirTarget = CalculatePoint2PointDistance(Get_Pos_Present().point, virtualTarget.point);

	float disadd = (VIEW_L - disRealPos2VirPos) - disRealPos2VirTarget;

	if(Get_Walk_Path_Length() < GetLength())
	{
		if(disadd > 0)
		{
			Add_Path_Length(2 * disadd);
		}
	}
	else
	{
		//记录的路程大于轨迹总路程后停止记录路程
		Update_Length_Stop();
	}

	//虚拟位置点姿态角
	posAngleVP = CalculateAngleAdd(GetRingBufferPointPoseAngle(1),CalculateAngleSub(GetRingBufferPointPoseAngle(2),GetRingBufferPointPoseAngle(1)) * virtualPos.u);

	//两端点之间角度的插值
	angleErr = CalculateAngleSub(GetRingBufferPointPoseAngle(virtualTarget.endPtr), GetRingBufferPointPoseAngle(virtualTarget.startPtr));

	//虚拟目标点姿态角
	posAngleVT = CalculateAngleAdd(GetRingBufferPointPoseAngle(virtualTarget.startPtr),angleErr*virtualTarget.u);

	//角度闭环
	angularVel = Angle_Ctrl(Get_Pos_Present().direction, posAngleVT, 3.2f, 2.9f);

	//目标速度方向
	velDir = virtualTarget.direction;

	//速度方向限幅到-180 ~ 180
	Angle_Limit(&velDir);

	//目标速度
	vell = GetRingBufferPointVell(virtualTarget.startPtr)+(GetRingBufferPointVell(virtualTarget.endPtr) - GetRingBufferPointVell(virtualTarget.startPtr))*virtualTarget.u;
	vell *= percent;
	
	return 1;
}

//角度闭环
float Angle_Ctrl(float angle_now, float angle_tar, float kp, float kd)
{
	float angleErr = 0.0f,angularVel = 0.0f, angularVelErr = 0.0f;
	static float lastAngleErr = 0.0f, preAngleErr = 0.0f, lastAngledTerm = 0.0f;
	float dTerm = 0.0f,dTermFliter = 0.0f;

	angleErr = 	CalculateAngleSub(angle_tar, angle_now);
	dTerm = angleErr - lastAngleErr;

	dTermFliter = 0.5f * dTerm + 0.5f* lastAngledTerm;

	angularVel = angleErr * kp + dTermFliter *kd;

	lastAngledTerm = dTerm;
	lastAngleErr = angleErr;
	preAngleErr = lastAngleErr;

	angularVelErr = angularVel - Get_AngularSpeed_W();

	//增加角速度闭环
	angularVel = angularVel + angularVelErr * 0.2f;

	//限幅
	if(angularVel>240.0f)
	{
		angularVel = 240.0f;
	}
	else if(angularVel<-240.0f)
	{
		angularVel = -240.0f;
	}

	if(sqrt(Get_Speed_X() * Get_Speed_X() + Get_Speed_Y() * Get_Speed_Y()) < 250.0f)
	{
		if(angularVel > 30.0f)
		{
			angularVel = 30.0f;
		}
		else if(angularVel<-30.0f)
		{
			angularVel = -30.0f;
		}
	}

	//
	return angularVel;
}
