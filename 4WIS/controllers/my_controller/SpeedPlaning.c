#include "SpeedPlaning.h"
#include <stdlib.h>
#include "ringbuffer.h"
#include <math.h>
#include <stdio.h>
#include "calculate.h"
#include "chassis.h"

extern FILE *fpWrite;

//float GetAxisAccMax(void)
//{
//		return (6000.0f);
//}

//float GetAccMax(void)
//{  
//	return GetAxisAccMax()*1.414214f; 
//}

float GetVelMax(void)
{
	return MAX_TAR_VEL;
}

float GetOmegaMax(void)
{
	return RAD2ANGLE(Pulse2Vel(MAX_MOTOR_SPEED)/MOVEBASE_RADIUS);
}

//通过配置的轮子最大加速度进行降速
//适当比例的降速，算完后记得把最新速度数据放在ringbuffer里
//wheelOne 一号轮速度数组首地址
//wheelTwo 	二号轮速度数组首地址
//wheelThree 三号轮速度数组首地址
void DynamicalAjusting(float* wheelOne, float* wheelTwo, float* wheelThree)
{
	float time = 0.0f;
	int n = GetCount();
	float tempAcc = 0.0f;

	int8_t velDirection  = 0;
	
	//每次加速度降低至上次的百分值
	float percent = 0.05f;

	//先正向削减速度
	for (int i = 2; i < n + 1; i++)
	{
		//轮1
		//粗略计算每两示教点之间的运动的时间
		time = (GetRingBufferPointLen(i) - GetRingBufferPointLen(i - 1)) / (GetRingBufferPointVell(i) + GetRingBufferPointVell(i - 1)) * 2;
		
		//计算两点之间的姿态角度变化量
	    float angErr = GetRingBufferPointPoseAngle(i) - GetRingBufferPointPoseAngle(i - 1);
		angErr = angErr > 180 ? angErr - 360 : angErr;
		angErr = angErr < -180 ? 360 + angErr : angErr;
		
		//计算旋转角速度
		float rotationVell = angErr / time;
		
		wheelOne[i - 1] = CalcWheelSpeed(GetRingBufferPointVell(i), GetRingBufferPointAngle(i), rotationVell,\
												ONE_VERTICAL_ANG , GetRingBufferPointPoseAngle(i)).vel;

		//只处理速度同向的情况
		velDirection = wheelOne[i - 1] - wheelOne[i - 2] >= 0 ? 1 : -1;

		tempAcc = fabs(wheelOne[i - 1] - wheelOne[i - 2]) / time;
		
		if (tempAcc > GetAccLimit2())
		{
			for(uint8_t n = 30;n>0;n--)
			{
				velDirection = wheelOne[i - 1] - wheelOne[i - 2] >= 0 ? 1 : -1;
				
				//将机器人整体移动速度缩小0.05
				SetRingBufferPointVell(i, GetRingBufferPointVell(i)*(1 - velDirection * percent));

				//计算两点之间的姿态角度变化量
				float angErr = GetRingBufferPointPoseAngle(i) - GetRingBufferPointPoseAngle(i - 1);
				angErr = angErr > 180 ? angErr - 360 : angErr;
				angErr = angErr < -180 ? 360 + angErr : angErr;
				
				float time = 0.0f;

				//粗略估计两点之间运动时间
				time = (GetRingBufferPointLen(i) - GetRingBufferPointLen(i - 1)) / (GetRingBufferPointVell(i) + GetRingBufferPointVell(i - 1)) * 2;

				//计算旋转角速度
				float rotationVell = angErr / time;
				
				//计算减小整体速度后的轮速
				wheelOne[i - 1] = CalcWheelSpeed(GetRingBufferPointVell(i), GetRingBufferPointAngle(i), rotationVell,\
												ONE_VERTICAL_ANG , GetRingBufferPointPoseAngle(i)).vel;
			
				//计算轮子的加速度
				tempAcc = fabs(wheelOne[i - 1] - wheelOne[i - 2]) / time;
				
				//如果加速度小于限制则跳出
				if(tempAcc<=GetAccLimit2())
				{
					break;
				}
			}
		}
//		delay_us(100);
//		USARTDMAOUT(DEBUG_USART,DebugUSARTSendBuf,&DebugUSARTSendBuffCnt,DebugUSARTDMASendBuf,DEBUG_USART_SEND_BUF_CAPACITY,\
//					(uint8_t *)"1\t%d\t%d\t%d\r\n",(int)i,(int)GetRingBufferPointVell(i),(int)wheelOne[i-1]);
		
		//轮2
		
		time = (GetRingBufferPointLen(i) - GetRingBufferPointLen(i - 1)) / (GetRingBufferPointVell(i) + GetRingBufferPointVell(i - 1)) * 2;

		//计算两点之间的姿态角度变化量
		angErr = GetRingBufferPointPoseAngle(i) - GetRingBufferPointPoseAngle(i - 1);
		angErr = angErr > 180 ? angErr - 360 : angErr;
		angErr = angErr < -180 ? 360 + angErr : angErr;
		
		//计算旋转角速度
		rotationVell = angErr / time;
		
		wheelTwo[i - 1] = CalcWheelSpeed(GetRingBufferPointVell(i), GetRingBufferPointAngle(i), rotationVell,\
												TWO_VERTICAL_ANG, GetRingBufferPointPoseAngle(i)).vel;
		
		velDirection = wheelTwo[i - 1] - wheelTwo[i - 2] >= 0 ? 1 : -1;

		tempAcc = fabs(wheelTwo[i - 1] - wheelTwo[i - 2]) / time;

		if (tempAcc > GetAccLimit2())
		{
			for(uint8_t n = 30;n>0;n--)
			{
				velDirection = wheelTwo[i - 1] - wheelTwo[i - 2] >= 0 ? 1 : -1;
				
				//将机器人整体移动速度缩小0.05
				SetRingBufferPointVell(i, GetRingBufferPointVell(i)*(1 - velDirection * percent));
				
				//计算两点之间的姿态角度变化量
				float angErr = GetRingBufferPointPoseAngle(i) - GetRingBufferPointPoseAngle(i - 1);
				angErr = angErr > 180 ? angErr - 360 : angErr;
				angErr = angErr < -180 ? 360 + angErr : angErr;

				float time = 0.0f;

				//粗略估计两点之间运动时间
				time = (GetRingBufferPointLen(i) - GetRingBufferPointLen(i - 1)) / (GetRingBufferPointVell(i) + GetRingBufferPointVell(i - 1)) * 2;

				//计算旋转角速度
				float rotationVell = angErr / time;
				
				//计算减小整体速度后的轮速
				wheelTwo[i - 1] = CalcWheelSpeed(GetRingBufferPointVell(i), GetRingBufferPointAngle(i), rotationVell,\
												TWO_VERTICAL_ANG , GetRingBufferPointPoseAngle(i)).vel;
			
				//计算轮子的加速度
				tempAcc = fabs(wheelTwo[i - 1] - wheelTwo[i - 2]) / time;
				
				//如果加速度小于限制则跳出
				if(tempAcc<=GetAccLimit2())
				{
					break;
				}
			}
		}
//		delay_us(100);
//		USARTDMAOUT(DEBUG_USART,DebugUSARTSendBuf,&DebugUSARTSendBuffCnt,DebugUSARTDMASendBuf,DEBUG_USART_SEND_BUF_CAPACITY,\
//					(uint8_t *)"2\t%d\t%d\t%d\r\n",(int)i,(int)GetRingBufferPointVell(i),(int)wheelTwo[i-1]);
		
		
		//轮3
		time = (GetRingBufferPointLen(i) - GetRingBufferPointLen(i - 1)) / (GetRingBufferPointVell(i) + GetRingBufferPointVell(i - 1)) * 2;

		//计算两点之间的姿态角度变化量
		angErr = GetRingBufferPointPoseAngle(i) - GetRingBufferPointPoseAngle(i - 1);
		angErr = angErr > 180 ? angErr - 360 : angErr;
		angErr = angErr < -180 ? 360 + angErr : angErr;
		
		//计算旋转角速度
		rotationVell = angErr / time;
		
		wheelThree[i - 1] = CalcWheelSpeed(GetRingBufferPointVell(i), GetRingBufferPointAngle(i), rotationVell,\
													THREE_VERTICAL_ANG , GetRingBufferPointPoseAngle(i)).vel;
		
		velDirection = wheelThree[i - 1] - wheelThree[i - 2] >= 0 ? 1 : -1;

		tempAcc = fabs(wheelThree[i - 1] - wheelThree[i - 2]) / time;

		if (tempAcc > GetAccLimit2())
		{
			for(uint8_t n = 30;n>0;n--)
			{
				velDirection = wheelThree[i - 1] - wheelThree[i - 2] >= 0 ? 1 : -1;

				//将机器人整体移动速度缩小0.05
				SetRingBufferPointVell(i, GetRingBufferPointVell(i)*(1 - velDirection * percent));
				fprintf(fpWrite, "第%d点削减了%d次\r\n ", (int)i,(int)(30-n+1));
				//计算两点之间的姿态角度变化量
				float angErr = GetRingBufferPointPoseAngle(i) - GetRingBufferPointPoseAngle(i - 1);
				angErr = angErr > 180 ? angErr - 360 : angErr;
				angErr = angErr < -180 ? 360 + angErr : angErr;

				float time = 0.0f;

				//粗略估计两点之间运动时间
				time = (GetRingBufferPointLen(i) - GetRingBufferPointLen(i - 1)) / (GetRingBufferPointVell(i) + GetRingBufferPointVell(i - 1)) * 2;

				//计算旋转角速度
				float rotationVell = angErr / time;
				
				//计算减小整体速度后的轮速
				wheelThree[i - 1] = CalcWheelSpeed(GetRingBufferPointVell(i), GetRingBufferPointAngle(i), rotationVell,\
													THREE_VERTICAL_ANG , GetRingBufferPointPoseAngle(i)).vel;
			
				//计算轮子的加速度
				tempAcc = fabs(wheelThree[i - 1] - wheelThree[i - 2]) / time;
				
				//如果加速度小于限制则跳出
				if(tempAcc<=GetAccLimit2())
				{
					break;
				}
			}
		}	
//		delay_us(50);
//		USARTDMAOUT(DEBUG_USART,DebugUSARTSendBuf,&DebugUSARTSendBuffCnt,DebugUSARTDMASendBuf,DEBUG_USART_SEND_BUF_CAPACITY,\
//					(uint8_t *)"3\t%d\t%d\t%d\r\n",(int)i,(int)GetRingBufferPointVell(i),(int)wheelThree[i-1]);
		
		//粗略计算每两示教点之间的运动的时间
		time = (GetRingBufferPointLen(i) - GetRingBufferPointLen(i - 1)) / (GetRingBufferPointVell(i) + GetRingBufferPointVell(i - 1)) * 2;


		//计算两点之间的姿态角度变化量
		angErr = GetRingBufferPointPoseAngle(i) - GetRingBufferPointPoseAngle(i - 1);
		angErr = angErr > 180 ? angErr - 360 : angErr;
		angErr = angErr < -180 ? 360 + angErr : angErr;
		
		//计算旋转角速度
		rotationVell = angErr / time;
		
		//计算轮速
		wheelOne[i - 1] = CalcWheelSpeed(GetRingBufferPointVell(i), GetRingBufferPointAngle(i), rotationVell,\
												ONE_VERTICAL_ANG , GetRingBufferPointPoseAngle(i)).vel;
		wheelTwo[i - 1] = CalcWheelSpeed(GetRingBufferPointVell(i), GetRingBufferPointAngle(i), rotationVell,\
												TWO_VERTICAL_ANG, GetRingBufferPointPoseAngle(i)).vel;
		wheelThree[i - 1] = CalcWheelSpeed(GetRingBufferPointVell(i), GetRingBufferPointAngle(i), rotationVell,\
													THREE_VERTICAL_ANG , GetRingBufferPointPoseAngle(i)).vel;									
	}

}

//通过ringBuffer里的数据计算每一点处三个轮子的速度
//目的更新wheelOne wheelTwo wheelThree这三个数组里的三轮速度，便于下一次的速度削减
//wheelOne 一号轮速度数组首地址
//wheelTwo 	二号轮速度数组首地址
//wheelThree 三号轮速度数组首地址
void CalculateWheelVell(float* wheelOne,float* wheelTwo,float* wheelThree)
{
	//分解到三个轮对全局速度进行规划
	float n = GetCount();
	for (int i = 2; i < n + 1; i++)
	{
		float angErr = GetRingBufferPointPoseAngle(i) - GetRingBufferPointPoseAngle(i - 1);
		angErr = angErr > 180 ? angErr - 360 : angErr;
		angErr = angErr < -180 ? 360 + angErr : angErr;

		float time = 0.0f;

		time = (GetRingBufferPointLen(i) - GetRingBufferPointLen(i - 1)) / (GetRingBufferPointVell(i) + GetRingBufferPointVell(i - 1)) * 2;

		float rotationVell = angErr / time;

		wheelOne[i - 1] = CalcWheelSpeed(GetRingBufferPointVell(i), GetRingBufferPointAngle(i), rotationVell,\
										ONE_VERTICAL_ANG , GetRingBufferPointPoseAngle(i)).vel;

		wheelTwo[i - 1] = CalcWheelSpeed(GetRingBufferPointVell(i), GetRingBufferPointAngle(i), rotationVell,\
										TWO_VERTICAL_ANG , GetRingBufferPointPoseAngle(i)).vel;

		wheelThree[i - 1] = CalcWheelSpeed(GetRingBufferPointVell(i), GetRingBufferPointAngle(i), rotationVell,\
										THREE_VERTICAL_ANG , GetRingBufferPointPoseAngle(i)).vel;

	}
	wheelOne[0] = wheelOne[1];
	wheelTwo[0] = wheelTwo[1];
	wheelThree[0] = wheelThree[1];
}





//通过降低合速度保证某轮的速度要求
//vellCar 降速前的前进合速度 单位 mm/s
//orientation 速度朝向 单位 度
//rotationalVell 旋转速度 单位 度每秒
//wheelNum  所降速的轮号
 // targetWheelVell   所降速的目标
// 返回所降低后的合速度
float  DecreseVellByOneWheel(float vellCar, float orientation, float rotationalVell,float zAngle, int wheelNum, float targetWheelVell)
{
	#define DECREASE_PERCENT (0.95f)
	TriWheelVel_t vell;
	int i;
	switch (wheelNum)
	{
		case 1:
			//每次合速度乘0.9,直到满足一号轮速度降低至目标速度。对于一些不能满足的，循环10次后自动退出
			for (i = 0; i < 10; i++)
			{
				vell.v1 = CalcWheelSpeed(vellCar, orientation, rotationalVell,ONE_VERTICAL_ANG , zAngle).vel;
				if (fabs(vell.v1) <= fabs(targetWheelVell))
				{
					break;
				}
				vellCar *= DECREASE_PERCENT;			
			}
		break;

		case 2:
			//每次合速度乘0.9,直到满足二号轮速度降低至目标速度。对于一些不能满足的，循环10次后自动退出
			for (i = 0; i < 10; i++)
			{

				vell.v2 = CalcWheelSpeed(vellCar, orientation, rotationalVell,TWO_VERTICAL_ANG , zAngle).vel;
				if (fabs(vell.v2) < fabs(targetWheelVell))
				{
					break;
				}
				vellCar *= DECREASE_PERCENT;
			}
		break;

		case 3:
			//每次合速度乘0.9,直到满足三号轮速度降低至目标速度。对于一些不能满足的，循环10次后自动退出
			for (i = 0; i < 10; i++)
			{

				vell.v3 = CalcWheelSpeed(vellCar, orientation, rotationalVell,THREE_VERTICAL_ANG , zAngle).vel;
				if (fabs(vell.v3) < fabs(targetWheelVell))
				{
					break;
				}
				vellCar *= DECREASE_PERCENT;
			}
			break;

	}
	return vellCar;
}

// float GetAccLimit(void)
// {
	
// 	return 11000.0f;
	
// }

float GetAccLimit(float posX,float posY)
{
	if((gRobot.walkStatus == waitForEleventhPath && posY > 9500.0f) || (gRobot.walkStatus == waitForEighthPath && posY < 4000.f) || (gRobot.walkStatus == waitForForthPath && posY < 10800.f))
	{
		return  5000.0f;
	}
	else if(gRobot.walkStatus == waitForStart && posY > 8400.0f)
	{
		return 6000.f;
	}
	else if(gRobot.walkStatus == waitForEleventhPath && posY > 9500.0f)
	{
		return 6000.f;
	}
	else if(gRobot.walkStatus == waitForTenthPath && posY < 3000.f)
	{
		return 5000.f;
	}
	else
	{
		return 11000.0f;
	}
}

float GetAccLimit2(void)
{
	
	return 16000.0f;
	
}

float GetVelLimit(void)
{
	return GetVelMax();
}

// float CalculateAccT(float accN)
// {
// 	if(accN<GetAccLimit())
// 	{
// 		return sqrtf(GetAccLimit()*GetAccLimit() - accN*accN);
// 	}
// 	else
// 	{
// 		return 0.0f;
// 	}	
// }

#define MIN_VELL (150.0f)
#define SEND_DATA (0)
//速度规划函数
float testPlanVel = 0.0f;
void SpeedPlaning(void)
{
	int n = GetCount();
	int troughCnt = 0; //波谷个数
	int* troughNumRecord=NULL;
	float* vell = NULL;
	float* curvature = NULL;
	float* curveNormalDirection = NULL;	
	float* wheelOne = NULL;
	float* wheelTwo = NULL;
	float* wheelThree = NULL;
	

	troughNumRecord = (int *)malloc(n*sizeof(int));
	vell = (float *)malloc(n*sizeof(float));
	curvature = (float *)malloc(n*sizeof(float));	   
	curveNormalDirection = (float *)malloc(n*sizeof(float));
	wheelOne = (float *)malloc(n*sizeof(float));
	wheelTwo = (float *)malloc(n*sizeof(float));
	wheelThree = (float *)malloc(n*sizeof(float));

	//记录每一个点的曲率半径并计算每个点的法向方向
	for (int i = 0; i < n; i++)
	{ 
		curvature[i] = GetRingBufferAverCurvature(i + 1);
		curveNormalDirection[i] = GetRingBufferPointAngle(i + 1) + 90.0f;
		
		AngleLimit(&curveNormalDirection[i]);
	}
	//为曲率半径起止位置赋值
	curvature[0] = curvature[1];
	curvature[n - 1] = curvature[n - 2];
	if(SEND_DATA)
	{
		//输出未限幅的曲率半径
		// fprintf(fpWrite, "r ");
		// for (int i = 0; i < n; i++)//n个点
		// {
		// 	fprintf(fpWrite, "%d ", (int)curvature[i]);
		// }
		// fprintf(fpWrite, "\r\n");
		//寻找曲率半径最小点，将其所在的位置记录下来
	}

	for (int i = 1; i < n - 1; i++) //首尾不算波谷
	{
		if(curvature[i] <= curvature[i+1] && curvature[i] <= curvature[i-1])
		{
			troughNumRecord[troughCnt] = i;//记录的是波谷的下标
			troughCnt ++;//记录波谷的个数
			fprintf(fpWrite, "找到一个波谷，在%d处 \r\n", (int)i);
		}
	}
	
	for(int j = 0;j < 1;j++ )
	{
		for (int i = 1; i < n-2; i++)
		{
			if(curvature[i] < 20000 &&curvature[i+1] < 20000 &&curvature[i-1] < 20000)
			{
				if(curvature[i] >= curvature[i+1] && curvature[i] >= curvature[i-1])
				{
					curvature[i] = (curvature[i+1]+curvature[i-1])/2;
				}
			} 
		}
	}
	for(int i = 0;i < troughCnt - 1;i ++)//遍历所有的波谷（曲率半径最小点）,除了最后一个点
	{
		int cutFlag = 1;
		for(int j = (troughNumRecord[i]+1);j < troughNumRecord[i+1];j ++)//遍历和下一个波谷之间的所有点
		{
			if(curvature[j] > 5000)//存在曲率半径很大的点
				cutFlag = 0;
		}
		if(cutFlag == 1)//两波谷之间无大于5000的点
		{
			for(int j = (troughNumRecord[i]+1);j < troughNumRecord[i+1];j++)//遍历和下一个波谷之间的所有点,平滑线性化
			{
				curvature[j] = curvature[j-1] + (curvature[troughNumRecord[i+1]] - curvature[troughNumRecord[i]])/(troughNumRecord[i+1]-troughNumRecord[i]);
			}
		}
	}
	if(SEND_DATA)
	{
		// fprintf(fpWrite, "r ");
		// for (int i = 0; i < n; i++)
		// {
		// 	fprintf(fpWrite, "%d ", (int)curvature[i]);
		// }
		// fprintf(fpWrite, "\r\n");
	}
	
	//通过曲率半径计算该段能满足的最大速度
	for (int i = 0; i < n; i++)
	{
		//v ^ 2 = a * R 
		vell[i] = sqrt((1.0f * GetAccLimit(GetRingBufferPoint(i).x,GetRingBufferPoint(i).y)) * curvature[i]);
		if(vell[i]>GetVelLimit())
		{
			vell[i] = GetVelLimit();
		}
	}

	if(SEND_DATA)
	{
		fprintf(fpWrite, "v1 ");
		for (int i = 0; i < n; i++)
		{
			fprintf(fpWrite, "%d ", (int)vell[i]);
		}
		fprintf(fpWrite, "\r\n");
	}
	
	
	//将初始速度和终止速度改为给定值  与传入变量 vellMax比较
	for (int i = 0; i < n; i++)
	{
		if(vell[i]>GetRingBufferPointVell(i+1))
		{
			vell[i] = GetRingBufferPointVell(i+1);
		}
	}	
	
	if(vell[0]<150.0f)
	{
		vell[0] = 150.0f;
	}
	
	//临时计算速度变量
	float tempVell = 0.0f;
	//临时计算该段轨迹结束的最大速度
	float tempVirtualVel = 0.0f;
	//估计的该段轨迹平均速度
	float tempTargetVel = 0.0f;
	//法向加速度
	float accN = 0.0f;
	//切向加速度
	float accT = 0.0f;
	//法向加速度方向
	float accNAngle = 0.0f;
	//估计每段轨迹的速度方向
	float tempAngle = 0.0f;
	//速度变化方向
	float angleChange = 0.0f;
	//留有的加速度余量
	#define ACC_REDUNDANT (0.05f)
	//通过v2^2 - v1^2 = 2*a*s对速度再次规划
	for (int i = 0; i < n - 1; i++)
	{
		//加速过程
		if (vell[i + 1] > vell[i])
		{
			//计算两点间的角度平均值
			tempAngle = (GetRingBufferPointAngle(i + 1) + GetRingBufferPointAngle(i + 2));
			//计算速度方向的变化
			angleChange = GetRingBufferPointAngle(i + 2) - GetRingBufferPointAngle(i + 1);
			
			if(fabs(angleChange)>180.0f)
			{
				tempAngle+=360.0f;
			}
			
			tempAngle/=2.0f;
			
			AngleLimit(&tempAngle);
			
			AngleLimit(&angleChange);
			
			//根据速度方向变化计算切向加速度方向
			if(angleChange>=0.0f)
			{
				accNAngle = tempAngle + 90.0f;			
			}
			else
			{
				accNAngle = tempAngle - 90.0f;
			}
			
			AngleLimit(&accNAngle);
			
			//估算该段轨迹结束时的速度替代之前的最大速度 v2^2 - v1^2 = 2*a*s
			tempVirtualVel = sqrt(2 * GetAccLimit(GetRingBufferPoint(i + 1).x,GetRingBufferPoint(i + 1).y) * (GetRingBufferPointLen(i + 2) - GetRingBufferPointLen(i + 1)) + vell[i] * vell[i]);
			
			//该段轨迹结束的最大速度
			tempTargetVel = vell[i+1];
			//如果估算速度小于最大速度，用估算速度来估算法向加速度
			if(tempVirtualVel<vell[i+1])
			{
				tempTargetVel = tempVirtualVel;
			}
			//计算法向加速度
			accN = pow(tempTargetVel + vell[i],2)/(2.0f * (curvature[i] + curvature[i + 1]));
			//法向加速度较小时忽略不计
			if(accN<=100.0f)
			{
				accN = 0.0f;
			}
			//计算切向加速度
			// accT = CalculateAccT(accN);
			if(accN<GetAccLimit(GetRingBufferPoint(i + 1).x,GetRingBufferPoint(i + 1).y))
			{
				accT = sqrtf(GetAccLimit(GetRingBufferPoint(i + 1).x,GetRingBufferPoint(i + 1).y)*GetAccLimit(GetRingBufferPoint(i + 1).x,GetRingBufferPoint(i + 1).y) - accN*accN);
			}
			else
			{
				accT = 0.0f;
			}	
			//根据切向加速度估计该段轨迹结束时的速度 v2^2 - v1^2 = 2*a*s
			tempVell = sqrt(2 * ((1.0f - ACC_REDUNDANT) * accT) * (GetRingBufferPointLen(i + 2) - GetRingBufferPointLen(i + 1)) + vell[i] * vell[i]);
			if (tempVell < vell[i + 1])
			{
				vell[i + 1] = tempVell;
			}
		}
		
	}
	if(SEND_DATA)
	{
		fprintf(fpWrite, "vv ");
		for (int i = 0; i < n; i++)
		{
			fprintf(fpWrite, "%d ", (int)vell[i]);
		}
		fprintf(fpWrite, "\r\n");
	}
	
	
	
	for (int i = n - 1; i > 0; i--)
	{
		//减速过程
		if (vell[i - 1] > vell[i])
		{		
			//计算两点间的角度平均值
			tempAngle = (GetRingBufferPointAngle(i) + GetRingBufferPointAngle(i + 1));
			//计算速度方向的变化
			angleChange = GetRingBufferPointAngle(i + 1) - GetRingBufferPointAngle(i);
			
			if(fabs(angleChange)>180.0f)
			{
				tempAngle+=360.0f;
			}
			
			tempAngle/=2.0f;
			
			AngleLimit(&tempAngle);
			
			AngleLimit(&angleChange);
			
			//根据速度方向变化计算切向加速度方向
			if(angleChange>=0.0f)
			{
				accNAngle = tempAngle + 90.0f;			
			}
			else
			{
				accNAngle = tempAngle - 90.0f;
			}
			
			AngleLimit(&accNAngle);
			
			//估算该段轨迹结束时的速度替代之前的最大速度
			tempVirtualVel = sqrt(2 * GetAccLimit(GetRingBufferPoint(i - 1).x,GetRingBufferPoint(i - 1).y) * (GetRingBufferPointLen(i + 1) - GetRingBufferPointLen(i)) + vell[i] * vell[i]);
			//该段轨迹结束的最大速度
			tempTargetVel = vell[i - 1];
			//如果估算速度小于最大速度，用估算速度来估算法向加速度
			if(tempVirtualVel<vell[i - 1])
			{
				tempTargetVel = tempVirtualVel;
			}
			//计算法向加速度
			accN = pow(tempTargetVel + vell[i],2)/(2.0f * (curvature[i] + curvature[i - 1]));
			//法向加速度较小时忽略不计
			if(accN<=100.0f)
			{
				accN = 0.0f;
			}
			//计算切向加速度
			// accT = CalculateAccT(accN);
			if(accN<GetAccLimit(GetRingBufferPoint(i - 1).x,GetRingBufferPoint(i - 1).y))
			{
				accT = sqrtf(GetAccLimit(GetRingBufferPoint(i - 1).x,GetRingBufferPoint(i - 1).y)*GetAccLimit(GetRingBufferPoint(i - 1).x,GetRingBufferPoint(i - 1).y) - accN*accN);
			}
			else
			{
				accT = 0.0f;
			}	
			//根据切向加速度估计该段轨迹结束时的速度
			tempVell = sqrt(2 * ((1.0f - ACC_REDUNDANT) * accT) * (GetRingBufferPointLen(i + 1) - GetRingBufferPointLen(i)) + vell[i] * vell[i]);
			if (tempVell < vell[i - 1])
			{
				vell[i - 1] = tempVell;
			}
		}
	}

	if(SEND_DATA)
	{
		fprintf(fpWrite, "v2 ");
		for (int i = 0; i < n; i++)
		{
			fprintf(fpWrite, "%d ", (int)vell[i]);
		}
		fprintf(fpWrite, "\r\n");
	}
	

	//将暂时规划的速度放入环形数组里
	for (int i = 0; i < n; i++)
	{
		SetRingBufferPointVell(i + 1, vell[i]);
	}

	//计算此时轮的速度
	CalculateWheelVell(wheelOne, wheelTwo, wheelThree);
	if(SEND_DATA)
	{
		fprintf(fpWrite, "v3 ");
		for (int i = 0; i < n; i++)
		{
			fprintf(fpWrite, "%d ", (int)GetRingBufferPointVell(i+1));
		}
		fprintf(fpWrite, "\r\n");
	}	
	
	
	uint8_t ipointLast = 0;
	uint8_t ipointErr[10] = {0},ipointErrCount = 0;
	uint8_t dynamicalAjustingTimes = 0;		
	float carPercent  = 0.01f;

	//动态的对速度进行平衡
	while (1)
	{
		int ipoint = 0;
		
		for (ipoint = 3; ipoint < n; ipoint++)
		{
			float time = 0.01f;

			float lll;
			float vvv;
			lll = (GetRingBufferPointLen(ipoint) - GetRingBufferPointLen(ipoint - 1));
			vvv = (GetRingBufferPointVell(ipoint) + GetRingBufferPointVell(ipoint - 1)) / 2;
			time = lll / vvv;

			float a1,a2,a3;
			//如果判断某一个轮子加速度大于最大加速度时，进行调节

			a1 = fabs(wheelOne[ipoint - 1] - wheelOne[ipoint - 2]) / time;
			a2 = fabs(wheelTwo[ipoint - 1] - wheelTwo[ipoint - 2]) / time;
			a3 = fabs(wheelThree[ipoint - 1] - wheelThree[ipoint - 2]) / time;
			
//			delay_us(500);
//			USARTDMAOUT(DEBUG_USART,DebugUSARTSendBuf,&DebugUSARTSendBuffCnt,DebugUSARTDMASendBuf,DEBUG_USART_SEND_BUF_CAPACITY,\
//								(uint8_t *)"%d %d %d %d %d %d %d %d %d %d %d %d\r\n",ipoint,(int)lll,(int)vvv,(int)(time*1000),(int)a1,(int)a2,(int)a3,Err_ipoint[0],Err_ipoint[1],Err_ipoint[2],Err_ipoint[3],Err_ipoint[4]);
				
			if ( (((a1 > GetAccLimit2()) && (wheelOne[ipoint - 1] * wheelOne[ipoint - 2] > 0))\
				|| ((a2 > GetAccLimit2()) && (wheelTwo[ipoint - 1] * wheelTwo[ipoint - 2] > 0))\
				|| ((a3 > GetAccLimit2()) && (wheelThree[ipoint - 1] * wheelThree[ipoint - 2] > 0)))\
				&& ipoint != ipointErr[0]&& ipoint != ipointErr[1]&& ipoint != ipointErr[2]&& ipoint != ipointErr[3]&& ipoint != ipointErr[4]&&\
				ipoint != ipointErr[5]&& ipoint != ipointErr[6]&& ipoint != ipointErr[7]&& ipoint != ipointErr[8]&& ipoint != ipointErr[9])
			{
				if(ipoint == ipointLast)
				{
					dynamicalAjustingTimes++;
					
					for (int i = 0; i < n; i++)
					{
						//当一个点发生再次不满足时 整体削减车速
						SetRingBufferPointVell(i + 1, (1.0f - carPercent)*GetRingBufferPointVell(i+1));
					}
				}
				else
				{
					dynamicalAjustingTimes = 0;
				}
				
				//连续四次不满足的点长时间规划 标记该点不再规划该点 防止程序卡死
				if(dynamicalAjustingTimes > 2)
				{
					ipointErr[ipointErrCount] = ipoint;
					ipointErrCount++;
					ipointErrCount %= 10;
					
					dynamicalAjustingTimes = 0;
				}
				
				//每个轮子分别将所有点遍历一次  削减循环最多30次 结束后跳出重新从头计算加速度
				DynamicalAjusting(wheelOne, wheelTwo, wheelThree);
				
				ipointLast = ipoint;
				
				break;			
			}
		}

		if(SEND_DATA)
		{
			fprintf(fpWrite, "v4 ");
			for (int i = 0; i < n; i++)
			{
				fprintf(fpWrite, "%d ", (int)GetRingBufferPointVell(i+1));
			}
			fprintf(fpWrite, "\r\n");
		}	
		

		if (ipoint == n)
		{
			for (int i = 1; i < n; i++)
			{
				TriWheelVel_t tempTrueVell;
				tempTrueVell.v1 = wheelOne[i];
				tempTrueVell.v2 = wheelTwo[i];
				tempTrueVell.v3 = wheelThree[i];
				
				float vellCar1, vellCar2, vellCar3, vellCar;
				float angErr = GetRingBufferPointPoseAngle(i + 2) - GetRingBufferPointPoseAngle(i + 1);
				
				angErr = angErr > 180 ? angErr - 360 : angErr;
				angErr = angErr < -180 ? 360 + angErr : angErr;
				
				//粗略计算每两示教点之间的运动的时间
				float time = (GetRingBufferPointLen(i + 2) - GetRingBufferPointLen(i + 1)) / (GetRingBufferPointVell(i + 2) + GetRingBufferPointVell(i + 1)) * 2;

				//通过降低合速度保证某轮的速度要求
				vellCar1 = DecreseVellByOneWheel(GetRingBufferPointVell(i + 1), GetRingBufferPointAngle(i + 1), angErr / time, GetRingBufferPointPoseAngle(i + 1), 1, tempTrueVell.v1);

				vellCar2 = DecreseVellByOneWheel(GetRingBufferPointVell(i + 1), GetRingBufferPointAngle(i + 1), angErr / time, GetRingBufferPointPoseAngle(i + 1), 2, tempTrueVell.v2);

				vellCar3 = DecreseVellByOneWheel(GetRingBufferPointVell(i + 1), GetRingBufferPointAngle(i + 1), angErr / time, GetRingBufferPointPoseAngle(i + 1), 3, tempTrueVell.v3);

				if (fabs(vellCar1) >= fabs(vellCar2) && fabs(vellCar1) >= fabs(vellCar3))
				{
					vellCar = vellCar1;
					//将计算的最新合速度放入缓存池中
					SetRingBufferPointVell(i + 1, vellCar);
				}
				else if (fabs(vellCar2) >= fabs(vellCar1) && fabs(vellCar2) >= fabs(vellCar3))
				{
					vellCar = vellCar2;
					//将计算的最新合速度放入缓存池中
					SetRingBufferPointVell(i + 1, vellCar);
				}
				else if (fabs(vellCar3) >= fabs(vellCar1) && fabs(vellCar3) >= fabs(vellCar2))
				{
					vellCar = vellCar3;
					//将计算的最新合速度放入缓存池中
					SetRingBufferPointVell(i + 1, vellCar);
				}
			}
		}
		if(SEND_DATA)
		{
			fprintf(fpWrite, "v5 ");
			for (int i = 0; i < n; i++)
			{
				fprintf(fpWrite, "%d ", (int)GetRingBufferPointVell(i+1));
			}
			fprintf(fpWrite, "\r\n");
		}	
		

		//得到满足轮加速度、轮速度条件的车速后  再次按照v2^2 - v1^2 = 2*a*s规划一遍
		if (ipoint == n)
		{
			
			//轨迹起始速度较小时进行放大，避免输出速度较小时不会动
			if(GetRingBufferPointVell(1)<150.0f)
			{
				SetRingBufferPointVell(1, 150);
			}

			for (int i = n - 1; i > 0; i--)
			{
				if (GetRingBufferPointVell(i) > GetRingBufferPointVell(i + 1))
				{
					
					//计算两点间的角度平均值
					tempAngle = (GetRingBufferPointAngle(i) + GetRingBufferPointAngle(i + 1));
					//计算速度方向的变化
					angleChange = GetRingBufferPointAngle(i + 1) - GetRingBufferPointAngle(i);

					if(fabs(angleChange)>180.0f)
					{
						tempAngle+=360.0f;
					}

					tempAngle/=2.0f;

					AngleLimit(&tempAngle);
					
					AngleLimit(&angleChange);

					//根据速度方向变化计算切向加速度方向
					if(angleChange>=0.0f)
					{
						accNAngle = tempAngle + 90.0f;			
					}
					else
					{
						accNAngle = tempAngle - 90.0f;
					}

					AngleLimit(&accNAngle);

					//估算该段轨迹结束时的速度替代之前的最大速度
					tempVirtualVel = sqrt(2 * GetAccLimit(GetRingBufferPoint(i - 1).x,GetRingBufferPoint(i - 1).y) * (GetRingBufferPointLen(i + 1) - GetRingBufferPointLen(i)) + GetRingBufferPointVell(i + 1) * GetRingBufferPointVell(i + 1));
					//该段轨迹结束的最大速度
					tempTargetVel = GetRingBufferPointVell(i);
					//如果估算速度小于最大速度，用估算速度来估算法向加速度
					if(tempVirtualVel<GetRingBufferPointVell(i))
					{
						tempTargetVel = tempVirtualVel;
					}
					//计算法向加速度
					accN = pow(tempTargetVel + GetRingBufferPointVell(i + 1),2)/(2.0f * (curvature[i] + curvature[i - 1]));
					//法向加速度较小时忽略不计
					if(accN<=100.0f)
					{
						accN = 0.0f;
					}
					//计算切向加速度
					// accT = CalculateAccT(accN);
					if(accN<GetAccLimit(GetRingBufferPoint(i - 1).x,GetRingBufferPoint(i - 1).y))
					{
						accT = sqrtf(GetAccLimit(GetRingBufferPoint(i - 1).x,GetRingBufferPoint(i - 1).y)*GetAccLimit(GetRingBufferPoint(i - 1).x,GetRingBufferPoint(i - 1).y) - accN*accN);
					}
					else
					{
						accT = 0.0f;
					}	
					SetRingBufferPointAccT(i,-accT);
					// //根据切向加速度估计该段轨迹结束时的速度
					// tempVell = sqrt(2 * ((1.0f - ACC_REDUNDANT) * accT) * (GetRingBufferPointLen(i + 1) - GetRingBufferPointLen(i)) + GetRingBufferPointVell(i + 1) * GetRingBufferPointVell(i + 1));
					// if (tempVell < GetRingBufferPointVell(i))
					// {
					// 	SetRingBufferPointVell(i, tempVell);
					// }
				}
			}
			if(SEND_DATA)
			{
				fprintf(fpWrite,"v6 ");
				for (int i = 0; i < n; i++)
				{
					fprintf(fpWrite,"%d ", (int)GetRingBufferPointVell(i+1));
				}
				fprintf(fpWrite,"\r\n");
			}				

			for (int i = 0; i < n - 1; i++)
			{
				if (GetRingBufferPointVell(i + 2) > GetRingBufferPointVell(i + 1))
				{
					
					//计算两点间的角度平均值
					tempAngle = (GetRingBufferPointAngle(i + 1) + GetRingBufferPointAngle(i + 2));
					//计算速度方向的变化
					angleChange = GetRingBufferPointAngle(i + 2) - GetRingBufferPointAngle(i + 1);

					if(fabs(angleChange)>180.0f)
					{
						tempAngle+=360.0f;
					}

					tempAngle/=2.0f;

					AngleLimit(&tempAngle);

					AngleLimit(&angleChange);

					//根据速度方向变化计算切向加速度方向
					if(angleChange>=0.0f)
					{
						accNAngle = tempAngle + 90.0f;			
					}
					else
					{
						accNAngle = tempAngle - 90.0f;
					}

					AngleLimit(&accNAngle);

					//估算该段轨迹结束时的速度替代之前的最大速度
					tempVirtualVel = sqrt(2 * GetAccLimit(GetRingBufferPoint(i + 1).x,GetRingBufferPoint(i + 1).y) * (GetRingBufferPointLen(i + 2) - GetRingBufferPointLen(i + 1)) + GetRingBufferPointVell(i + 1) *GetRingBufferPointVell(i + 1));
					//该段轨迹结束的最大速度
					tempTargetVel = GetRingBufferPointVell(i + 2);
					//如果估算速度小于最大速度，用估算速度来估算法向加速度
					if(tempVirtualVel<GetRingBufferPointVell(i + 2))
					{
						tempTargetVel = tempVirtualVel;
					}
					//计算法向加速度
					accN = pow(tempTargetVel + GetRingBufferPointVell(i + 1),2)/(2.0f * (curvature[i] + curvature[i + 1]));
					//法向加速度较小时忽略不计
					if(accN<=100.0f)
					{
						accN = 0.0f;
					}
					//计算切向加速度
					// accT = CalculateAccT(accN);
					if(accN<GetAccLimit(GetRingBufferPoint(i + 1).x,GetRingBufferPoint(i + 1).y))
					{
						accT = sqrtf(GetAccLimit(GetRingBufferPoint(i + 1).x,GetRingBufferPoint(i + 1).y)*GetAccLimit(GetRingBufferPoint(i + 1).x,GetRingBufferPoint(i + 1).y) - accN*accN);
					}
					else
					{
						accT = 0.0f;
					}
					SetRingBufferPointAccT(i + 2,accT);
					// //根据切向加速度估计该段轨迹结束时的速度
					// tempVell = sqrt(2 * ((1.0f - ACC_REDUNDANT) * accT) * (GetRingBufferPointLen(i + 2) - GetRingBufferPointLen(i + 1)) + GetRingBufferPointVell(i + 1) * GetRingBufferPointVell(i + 1));
					// if (tempVell < GetRingBufferPointVell(i + 2))
					// {
					// 	SetRingBufferPointVell(i + 2, tempVell);
					// }
				}
			}

			if(SEND_DATA)
			{
				for (int i = 1; i < n; i++)
				{
					fprintf(fpWrite,"加速时加速度\r\n");
					fprintf(fpWrite,"%d %d ", (int)i,(int)GetRingBufferPointAccT(i));
				}
			}
			
			for (int i = 3; i < n-1; i++)
			{
				if((int)GetRingBufferPointAccT(i+1) *(int)GetRingBufferPointAccT(i-1) >= 0)
				{
					if((int)GetRingBufferPointAccT(i) > (int)GetRingBufferPointAccT(i-1) && (int)GetRingBufferPointAccT(i) > (int)GetRingBufferPointAccT(i+1))
					{
						SetRingBufferPointAccT(i ,((int)GetRingBufferPointAccT(i-1) + (int)GetRingBufferPointAccT(i+1))/2);
					}
					else if((int)GetRingBufferPointAccT(i) < (int)GetRingBufferPointAccT(i-1) && (int)GetRingBufferPointAccT(i) < (int)GetRingBufferPointAccT(i+1))
					{
						SetRingBufferPointAccT(i ,((int)GetRingBufferPointAccT(i-1) + (int)GetRingBufferPointAccT(i+1))/2);
					}
				}
			}
			if(SEND_DATA)
			{
				for (int i = 1; i < n; i++)
				{
					fprintf(fpWrite,"减速时加速度\r\n");
					fprintf(fpWrite,"%d %d ", (int)i,(int)GetRingBufferPointAccT(i));
				}
			}

			for (int i = n - 1; i > 0; i--)
			{
				if (GetRingBufferPointVell(i) > GetRingBufferPointVell(i + 1))
				{
					//根据切向加速度估计该段轨迹结束时的速度
					tempVell = sqrt(2 * ((1.0f - ACC_REDUNDANT) * fabs(GetRingBufferPointAccT(i))) * (GetRingBufferPointLen(i + 1) - GetRingBufferPointLen(i)) + GetRingBufferPointVell(i + 1) * GetRingBufferPointVell(i + 1));
					if (tempVell < GetRingBufferPointVell(i))
					{
						SetRingBufferPointVell(i, tempVell);
					}
				}
			}		
			for (int i = 0; i < n - 1; i++)
			{
				if (GetRingBufferPointVell(i + 2) > GetRingBufferPointVell(i + 1))
				{
					tempVell = sqrt(2 * ((1.0f - ACC_REDUNDANT) * fabs(GetRingBufferPointAccT(i+2))) * (GetRingBufferPointLen(i + 2) - GetRingBufferPointLen(i + 1)) + GetRingBufferPointVell(i + 1) * GetRingBufferPointVell(i + 1));
					if (tempVell < GetRingBufferPointVell(i + 2))
					{
						SetRingBufferPointVell(i + 2, tempVell);
					}
				}
			}
			if(SEND_DATA)
			{
				fprintf(fpWrite,"v7 ");
				for (int i = 0; i < n; i++)
				{
					fprintf(fpWrite,"%d ", (int)GetRingBufferPointVell(i+1));
				}
				fprintf(fpWrite,"\r\n");
			}
			
			
			//将速度小于最小速度的做处理
			for (int i = 1; i < n; i++)
			{
				if (GetRingBufferPointVell(i) < MIN_VELL)
				{
					SetRingBufferPointVell(i, MIN_VELL);
				}
			}
			
			//末速度很小置为0
			if(GetRingBufferPointVell(n)<10.0f)
			{
				SetRingBufferPointVell(n, 0);	
			}

			// SetRingBufferPointVell(n - 1, sqrtf(GetRingBufferPointVell(n-2)*GetRingBufferPointVell(n-2)*0.5f + \
			// 									GetRingBufferPointVell(n)*GetRingBufferPointVell(n)*0.5f));	

			
			for (int i = 1; i < n; i++)
			{
				SetRingBufferPointVell(i, GetRingBufferPointVell(i));
				testPlanVel = GetRingBufferPointVell(i);
				testPlanVel = testPlanVel;
			}
			
			free(wheelOne);
			free(wheelTwo);
			free(wheelThree);
			break;
		}
	}

	fprintf(fpWrite,"v8 ");
	for (int i = 0; i < n; i++)
	{
		fprintf(fpWrite,"%d ", (int)GetRingBufferPointVell(i+1));
	}
	fprintf(fpWrite,"\r\n");
	
	//发出错误点
	fprintf(fpWrite, "iErr %d %d %d %d %d %d %d %d %d %d\r\n",ipointErr[0],ipointErr[1],ipointErr[2],ipointErr[3],ipointErr[4],\
	ipointErr[5],ipointErr[6],ipointErr[7],ipointErr[8],ipointErr[9]);		
	
	
	free(curvature);
	free(vell);
	free(curveNormalDirection);
}
