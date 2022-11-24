#include <stdint.h>
#include <stdlib.h>
#include <stdio.h>
#include <math.h>

#include "Bspline.h"

#include "calculate.h"
#include "ringbuffer.h"

// #define TEST_PATH_FLAG (1)


//要走直线的话就定义

/*********************************************************************************
* @name 	judgeLineWant
* @brief  用于判断用户想走的是哪一条直线
* @param	
* @param	
* @param	
* @param	
* @retval 
*********************************************************************************/

// float judgeLineWant(int num,Pose_t* points)
// {

// 	return 0.0f;

// }

/*********************************************************************************
* @name 	modifyControlPoint
* @brief  用于在原来的B样条计算出控制点后对控制点进行修改进而规划出想要的曲线
* @param	
* @param	
* @param	
* @param	
* @retval 
*********************************************************************************/
void modifyControlPoint(Point_t* controlPoint)
{
	
}

/*********************************************************************************
* @name 	CaculateBsplineLen
* @brief    用于计算两点确定的一条B样条曲线的长度
* @param	point1 起始点				单位 mm
* @param	point2 结束点				单位 mm
* @param	angle1 起始点方向    单位 度
* @param	angle2 结束点方向    单位 度
* @retval   返回该样条曲线长度  单位 mm
*********************************************************************************/
float CaculateBsplineLen(Point_t point1, Point_t point2, float angle1, float angle2)
{
		//常数项
		Point_t P[2];
		//插值点
		//Point_t *passPoint = NULL;
		//系数矩阵的逆矩阵
		float invA[2][2] = { { 0.333333333f, 0.00000f }, { 0.0000f, 0.333333333f } };
		//控制到达切矢方向的提前量
		float length = 0.0f;
		//零时控制点坐标
		Point_t dataPoint[2];
		//最终控制点坐标
		Point_t finalDataPoint[6];
		Point_t tempPoint;
		Point_t tempPointOld;
		int i, j;
		//该B样条曲线长度
		float u, b1, b2, b3, b0;



		length = 0.60f*sqrt((point1.x - point2.x)*(point1.x - point2.x) + (point1.y - point2.y)*(point1.y - point2.y));

		//对常数项赋值    angle加90是为了处理陀螺仪角度与坐标系角度不符
		P[0].x = 3 * point1.x + length*cos((angle1) * CHANGE_TO_RADIAN);
		P[0].y = 3 * point1.y + length*sin((angle1) * CHANGE_TO_RADIAN);
		P[1].x = 3 * point2.x - length*cos((angle2) * CHANGE_TO_RADIAN);
		P[1].y = 3 * point2.y - length*sin((angle2) * CHANGE_TO_RADIAN);

		//清零
		for (i = 0; i < 2; i++)
		{
			dataPoint[i].x = 0;
			dataPoint[i].y = 0;
		}


		//d = inv(a).*P  求临时控制点坐标
		for (i = 0; i < 2; i++)
		{
			for (j = 0; j < 2; j++)
			{
				dataPoint[i].x += invA[i][j] * P[j].x;
				dataPoint[i].y += invA[i][j] * P[j].y;
			}
		}

		//复制最终控制点坐标
		finalDataPoint[0] = point1;
		finalDataPoint[1] = point1;
		finalDataPoint[2] = dataPoint[0];
		finalDataPoint[3] = dataPoint[1];
		finalDataPoint[4] = point2;
		finalDataPoint[5] = point2;


		length = 0.0f;
		//将B样条上第一点到首端点的值加上
		tempPointOld = point1;
		for (i = 0; i < 3; i++)
		{
			for (j = 0; j < 50; j++)
			{
				u = (float)(j)* 0.02f;
				b0 = 1.0f / 6 * (1 - u) * (1 - u) * (1 - u);
				b1 = 1.0f / 6 * (3 * u * u * u - 6 * u * u + 4);
				b2 = 1.0f / 6 * (-3 * u * u * u + 3 * u * u + 3 * u + 1);
				b3 = 1.0f / 6 * u * u * u;

				tempPoint.x = b0*finalDataPoint[i].x + b1*finalDataPoint[i + 1].x + b2*finalDataPoint[i + 2].x + b3*finalDataPoint[i + 3].x;
				tempPoint.y = b0*finalDataPoint[i].y + b1*finalDataPoint[i + 1].y + b2*finalDataPoint[i + 2].y + b3*finalDataPoint[i + 3].y;

				length += CalculatePoint2PointDistance(tempPoint,tempPointOld);

				tempPointOld = tempPoint;
			}
		}
		//将B样条上最后一点到末端点的距离加上
		length += CalculatePoint2PointDistance(tempPoint, point2);
		
		return length;
	}


	
	
/*********************************************************************************
* @name 	SerchVirtualPoint
* @brief  根据距离起点的路径长度确定所对应的虚拟点
* @param	robotLen 距离起点的路径长度  单位 mm 
* @retval 返回 虚拟点坐标 和相应在所在样条曲线上的比例系数
*********************************************************************************/
PointU_t SerchVirtualPoint(float robotLen)
{
	int i = 0;
	PointU_t result;

	//确定目标长度在哪一段上
	for (i = 2; i < GetCount(); i++)
	{
		if (robotLen < GetRingBufferPointLen(i))
			break;
	}
	
	//确定落在第i-1 到 i 上的距离
	robotLen -= GetRingBufferPointLen(i-1);
	
	result = SerchBsplineVirtualPoint(GetRingBufferPoint(i - 1), GetRingBufferPoint(i), GetRingBufferPointAngle(i - 1), GetRingBufferPointAngle(i), robotLen);

	if((i-2) > 0)
	{
		DeleteData(i-2);
	}
	
	return result;

}



/*********************************************************************************
* @name 	SerchBsplineVirtualPoint
* @brief  根据距离所在样条曲线长度确定所对应的虚拟点
* @param	robotLen 距离所在样条曲线长度  单位 mm
* @retval 返回 虚拟点坐标 和相应在所在样条曲线上的比例系数
*********************************************************************************/
PointU_t SerchBsplineVirtualPoint(Point_t point1, Point_t point2, float angle1, float angle2,float robotlen)
{
	//常数项
	Point_t P[2];

	//系数矩阵的逆矩阵
	float invA[2][2] = { { 0.333333333f, 0.00000f }, { 0.0000f, 0.333333333f } };
	//控制到达切矢方向的提前量
	float length = 0.0f;
	//零时控制点坐标
	Point_t dataPoint[2];
	//最终控制点坐标
	Point_t finalDataPoint[6];
	Point_t tempPoint;
	Point_t tempPointOld; 
	int i, j;
	//该B样条曲线长度
	float u, b1, b2, b3, b0;
	PointU_t result;

	length = 0.60f*sqrt((point1.x - point2.x)*(point1.x - point2.x) + (point1.y - point2.y)*(point1.y - point2.y));

	//对常数项赋值    angle加90是为了处理陀螺仪角度与坐标系角度不符
	P[0].x = 3 * point1.x + length*cos((angle1) * CHANGE_TO_RADIAN);
	P[0].y = 3 * point1.y + length*sin((angle1) * CHANGE_TO_RADIAN);
	P[1].x = 3 * point2.x - length*cos((angle2) * CHANGE_TO_RADIAN);
	P[1].y = 3 * point2.y - length*sin((angle2) * CHANGE_TO_RADIAN);

	//清零
	for (i = 0; i < 2; i++)
	{
		dataPoint[i].x = 0;
		dataPoint[i].y = 0;
	}


	//d = inv(a).*P  求临时控制点坐标
	for (i = 0; i < 2; i++)
	{
		for (j = 0; j < 2; j++)
		{
			dataPoint[i].x += invA[i][j] * P[j].x;
			dataPoint[i].y += invA[i][j] * P[j].y;
		}
	}


	//复制最终控制点坐标
	finalDataPoint[0] = point1;
	finalDataPoint[1] = point1;
	finalDataPoint[2] = dataPoint[0];
	finalDataPoint[3] = dataPoint[1];
	finalDataPoint[4] = point2;
	finalDataPoint[5] = point2;


	length = 0.0f;
	//将B样条上第一点到首端点的值加上
	tempPointOld = point1;
	int num = 0;
	for (i = 0; i < 3; i++)
	{
		for (j = 0; j < 50; j++)
		{
			//用于计算U值
			num++;

			u = (float)(j)* 0.02f;

			b0 = 1.0f / 6 * (1 - u) * (1 - u) * (1 - u);
			b1 = 1.0f / 6 * (3 * u * u * u - 6 * u * u + 4);
			b2 = 1.0f / 6 * (-3 * u * u * u + 3 * u * u + 3 * u + 1);
			b3 = 1.0f / 6 * u * u * u;

			tempPoint.x = b0*finalDataPoint[i].x + b1*finalDataPoint[i + 1].x + b2*finalDataPoint[i + 2].x + b3*finalDataPoint[i + 3].x;
			tempPoint.y = b0*finalDataPoint[i].y + b1*finalDataPoint[i + 1].y + b2*finalDataPoint[i + 2].y + b3*finalDataPoint[i + 3].y;

			length += CalculatePoint2PointDistance(tempPoint, tempPointOld);
			if (robotlen < length)
			{
				float Db0 = -1.0f / 2 * (u - 1) * (u - 1);
				float Db1 = 1.0f / 2 * (3 * u * u - 4 * u);
				float Db2 = 1.0f / 2 * (-3 * u * u + u * 2 + 1);
				float Db3 = 1.0f / 2 * u * u;
				
				//切线方向
				result.direction = CHANGE_TO_ANGLE * atan2f((float)(Db0 * finalDataPoint[i].y + Db1 * finalDataPoint[i + 1].y +\
																	Db2 * finalDataPoint[i + 2].y + Db3 * finalDataPoint[i + 3].y), \
				(float)(Db0 * finalDataPoint[i].x + Db1 * finalDataPoint[i + 1].x + Db2 * finalDataPoint[i + 2].x +\
						Db3 * finalDataPoint[i + 3].x));

				result.point = tempPoint;
				result.u = (float)num / 123.0f;
				return result;
			}

			tempPointOld = tempPoint;
		}
	}

	//将B样条上最后一点到末端点的距离加上
	result.direction = angle2;
	
	result.point = point2;

	result.u = 1.0f;

	return result;
}

//确定虚拟目标点
PointU_t SerchVirtualPoint2(float robotLen)
{
	int i = 0;
	PointU_t result;

	//确定目标长度在哪一段上
	for (i = 2; i < GetCount(); i++)
	{
		if (robotLen < GetRingBufferPointLen(i))
			break;
	}
	
	//确定落在第i-1 到 i 上的距离
	robotLen -= GetRingBufferPointLen(i-1);
	
	result = SerchBsplineVirtualPoint(GetRingBufferPoint(i - 1), GetRingBufferPoint(i), GetRingBufferPointAngle(i - 1), GetRingBufferPointAngle(i), robotLen);

	result.startPtr = i-1;
	result.endPtr = i;
	
	return result;

}





/***************************************************************************
* @name		BspSegment
* @brief	把传入的B样条的关键点分段成15cm一段的若干有切线方向的点
* @num		传入的点个数
* @points	传入的B样条关键点
* @keyPoint	分段后存入的点
**************************************************************************/

// int BspSegment(int num, Pose_t* points, KeyPointInf_t* keyPoint)
// {
// 	//系数矩阵对角列
// 	float* a = NULL;
// 	a = (float *)malloc(sizeof(float)* num);
// 	//系数矩阵对角上列
// 	float* b = NULL;
// 	b = (float *)malloc(sizeof(float)* num);
// 	//系数矩阵对角下列
// 	float* c = NULL;
// 	c = (float *)malloc(sizeof(float)* num);
// 	//定义soluctionX、soluctionY为线性方程的解
// 	float* soluctionX = NULL;
// 	soluctionX = (float *)malloc(sizeof(float)* num);
// 	float* soluctionY = NULL;
// 	soluctionY = (float *)malloc(sizeof(float)* num);
// 	//定义dataX和dataY,用来存放inPoint里的X和Y坐标
// 	float* dataX = NULL;
// 	dataX = (float *)malloc(sizeof(float)* num);
// 	float* dataY = NULL;
// 	dataY = (float *)malloc(sizeof(float)* num);

// 	//定义controlPoint用来存放控制点
// 	Point_t* controlPoint = NULL;
// 	controlPoint = (Point_t*)malloc(sizeof(Point_t)*(num + 4));
// 	//存放画线的两个使用点
//  	Point_t lines[2];

// 	//初始化 a,b,c
// 	a[0] = 18;
// 	a[num - 1] = 18;

// 	for (int i = 1; i < num - 1; i++)
// 	{
// 		a[i] = 4;
// 	}

// 	for (int i = 1; i < num - 1; i++)
// 	{
// 		b[i] = 1;
// 		c[i] = 1;
// 	}

// 	c[num - 1] = -9;
// 	b[0] = -9;
	
// 	//用于判断四点共线变量
// 	float x1,x2,x3,y1,y2,y3 = 0.0f;
	
// 	//更改直线段对应系数矩阵
// 	for(int i = 0;i < num - 4;i++)
// 	{
// 		x1 = points[i].point.x - points[i+1].point.x;
// 		x2 = points[i].point.x - points[i+2].point.x;
// 		x3 = points[i].point.x - points[i+3].point.x;
// 		y1 = points[i].point.y - points[i+1].point.y;
// 		y2 = points[i].point.y - points[i+2].point.y;
// 		y3 = points[i].point.y - points[i+3].point.y;
		
// 		if((fabsf(x1) < 0.0001f && fabsf(x2) < 0.0001f && fabsf(x3) < 0.0001f)||
// 			(fabsf(y1) < 0.0001f && fabsf(y2) < 0.0001f && fabsf(y3) < 0.0001f)||
// 			(fabsf((x2*y1-y2*x1)/(x1*y1))<0.0001f&&fabsf((x3*y1-y3*x1)/(x1*y1))<0.0001f))
// 		{
// 			a[i] = 6;a[i+1] = 6;a[i+2] = 6;a[i+3] = 6;
// 			b[i] = 0;b[i+1] = 0;b[i+2] = 0;b[i+3] = 0;
// 			c[i] = 0;c[i+1] = 0;c[i+2] = 0;c[i+3] = 0;
// 			if(i==0)
// 			{
// 				a[i] = a[i]*1.5f;
// 			}
// 			else if(i==num-4)
// 			{
// 				a[i+3] = a[i+3]*1.5f;
// 			}
// 		}
		
// 	}

// 	for (int i = 0; i<num; i++)
// 	{
// 		dataX[i] = 6.0f * points[i].point.x;
// 		dataY[i] = 6.0f * points[i].point.y;
// 	}

// 	dataX[0] *= 1.5f;
// 	dataY[0] *= 1.5f;
// 	dataX[num - 1] *= 1.5f;
// 	dataY[num - 1] *= 1.5f;

// 	//计算soluctionX,soluctionY;

// 	//调用Matrix用追赶法求解线性方程
// 	Matrix(dataX, num, a, b, c, soluctionX);
// 	Matrix(dataY, num, a, b, c, soluctionY);

// 	controlPoint[num + 3].x = dataX[num - 1] / 9;
// 	controlPoint[num + 2].x = dataX[num - 1] / 9;
// 	controlPoint[0].x = dataX[0] / 9;
// 	controlPoint[1].x = dataX[0] / 9; 
   

// 	for (int i = 0; i < num; i++)
// 	{
// 		controlPoint[i + 2].x = soluctionX[i];
// 	}

// 	controlPoint[num + 3].y = dataY[num - 1] / 9;
// 	controlPoint[num + 2].y = dataY[num - 1] / 9;
// 	controlPoint[0].y = dataY[0] / 9;
// 	controlPoint[1].y = dataY[0] / 9;


// 	for (int i = 0; i < num; i++)
// 	{
// 		controlPoint[i + 2].y = soluctionY[i];
// 	}
	
// //	for (int i = 0; i < num + 1; i++)
// //	{
// //		USARTDMAOUT(DEBUG_USART,DebugUSARTSendBuf,&DebugUSARTSendBuffCnt,DebugUSARTDMASendBuf,DEBUG_USART_SEND_BUF_CAPACITY,\
// //					(uint8_t *)"%d %d\r\n",(int)controlPoint[i].x,(int)controlPoint[i].y);
// //	}
	

// 	//计算型值点，分成很近的点，取15cm一段
// 	//从初始点开始
// 	lines[0].x = (int)controlPoint[0].x;
// 	lines[0].y = (int)controlPoint[0].y;

// 	//累积的长度存入length
// 	float length = 0;
// 	float Len = 0;
// 	float startLength = 0;
// 	float beforeLength = 0;
// 	float endLength = 0;

// 	//计算分段之后的点的个数，然后分配内存空间
// 	int number = 0;


// 	for (int i = 0; i < num + 1; i++)
// 	{
// 		if (i == num - 1)
// 			beforeLength = length;
// 		for (float u = 0.01f; u <= 1; u += 0.01f)
// 		{
// 			float b0 = 1.0f / 6 * (1 - u) * (1 - u) * (1 - u);
// 			float b1 = 1.0f / 6 * (3 * u * u * u - 6 * u * u + 4);
// 			float b2 = 1.0f / 6 * (-3 * u * u * u + 3 * u * u + 3 * u + 1);
// 			float b3 = 1.0f / 6 * u * u * u;

// 			lines[1].x = (int)(b0 * controlPoint[i].x + b1 * controlPoint[i + 1].x + b2 * controlPoint[i + 2].x + b3 * controlPoint[i + 3].x);
// 			lines[1].y = (int)(b0 * controlPoint[i].y + b1 * controlPoint[i + 1].y + b2 * controlPoint[i + 2].y + b3 * controlPoint[i + 3].y);

// 			//gra.DrawLine(pen, lines[0], lines[1]);
// 			length += CalculatePoint2PointDistance(lines[0], lines[1]);
// 			if ( i <= 1)
// 				startLength += CalculatePoint2PointDistance(lines[0], lines[1]);
// 			if (i >= num -1)
// 				endLength += CalculatePoint2PointDistance(lines[0], lines[1]);
// 			if (length -Len > 122.25f)//149.25
// 			{
// 				Len = length;
// 				number++;
// 			}

// 			lines[0] = lines[1];
// 		}
// 	}
// //	keyPoint = (Act3WheelPose *)malloc(sizeof(ActLine2)* number);
// 	int resultNum = number + 1;
	
// 	float sumLen = length;
// 	Len = 0;
// 	length = 0;
	
// 	lines[0].x = (int)controlPoint[0].x;
// 	lines[0].y = (int)controlPoint[0].y;
	

// 	number = 1;
// 	for (int i = 0; i < num + 1; i++)
// 	{

// 		for (float u = 0.01f; u <= 1; u += 0.01f)
// 		{
// 			float b0 = 1.0f / 6 * (1 - u) * (1 - u) * (1 - u);
// 			float b1 = 1.0f / 6 * (3 * u * u * u - 6 * u * u + 4);
// 			float b2 = 1.0f / 6 * (-3 * u * u * u + 3 * u * u + 3 * u + 1);
// 			float b3 = 1.0f / 6 * u * u * u;

// 			lines[1].x = (int)(b0 * controlPoint[i].x + b1 * controlPoint[i + 1].x + b2 * controlPoint[i + 2].x + b3 * controlPoint[i + 3].x);
// 			lines[1].y = (int)(b0 * controlPoint[i].y + b1 * controlPoint[i + 1].y + b2 * controlPoint[i + 2].y + b3 * controlPoint[i + 3].y);

// 			//gra.DrawLine(pen, lines[0], lines[1]);
// 			length += CalculatePoint2PointDistance(lines[0], lines[1]);
// 			if (length - Len > 122)
// 			{
// 				Len = length;
// 				float Db0 = -1.0f / 2 * (u - 1) * (u - 1);
// 				float Db1 = 1.0f / 2 * (3 * u * u - 4 * u);
// 				float Db2 = 1.0f / 2 * (-3 * u * u + u * 2 + 1);
// 				float Db3 = 1.0f / 2 * u * u;
				
// 				//切线方向
// 				keyPoint[number].angle = CHANGE_TO_ANGLE * atan2f((float)(Db0 * controlPoint[i].y + Db1 * controlPoint[i + 1].y + Db2 * controlPoint[i + 2].y + Db3 * controlPoint[i + 3].y), \
// 					(float)(Db0 * controlPoint[i].x + Db1 * controlPoint[i + 1].x + Db2 * controlPoint[i + 2].x + Db3 * controlPoint[i + 3].x));
								
// 				keyPoint[number].point.x = lines[1].x;
// 				keyPoint[number].point.y = lines[1].y;

// //				delay_ms(5);
// //				USARTDMAOUT(DEBUG_USART,DebugUSARTSendBuf,&DebugUSARTSendBuffCnt,DebugUSARTDMASendBuf,DEBUG_USART_SEND_BUF_CAPACITY,\
// //					(uint8_t *)"%d %d %d ",(int)keyPoint[number].angle,(int)keyPoint[number].point.x,(int)keyPoint[number].point.y);
// //				USARTDMAOUT(DEBUG_USART,DebugUSARTSendBuf,&DebugUSARTSendBuffCnt,DebugUSARTDMASendBuf,DEBUG_USART_SEND_BUF_CAPACITY,\
// //					(uint8_t *)"%d %d %d %d ",(int)Db0,(int)Db1,(int)Db2,(int)Db3);
// //				USARTDMAOUT(DEBUG_USART,DebugUSARTSendBuf,&DebugUSARTSendBuffCnt,DebugUSARTDMASendBuf,DEBUG_USART_SEND_BUF_CAPACITY,\
// //					(uint8_t *)"%d %d %d %d ",(int)controlPoint[i].y,(int)controlPoint[i+1].y,(int)controlPoint[i+2].y,(int)controlPoint[i+3].y);
// //				USARTDMAOUT(DEBUG_USART,DebugUSARTSendBuf,&DebugUSARTSendBuffCnt,DebugUSARTDMASendBuf,DEBUG_USART_SEND_BUF_CAPACITY,\
// //					(uint8_t *)"%d %d %d %d\r\n",(int)controlPoint[i].x,(int)controlPoint[i+1].x,(int)controlPoint[i+2].x,(int)controlPoint[i+3].x);
// 				//姿态角度
// 				if (num == 2)
// 				{
// 					keyPoint[number].poseAngle =CalculateAngleAdd( points[0].direction, CalculateAngleSub(points[1].direction,points[0].direction) / sumLen * length);
// 				}
// 				else 
// 				{
// 					if (i > 1 && i < num - 1)
// 					{
// 						keyPoint[number].poseAngle =CalculateAngleAdd( points[i - 1].direction , u * CalculateAngleSub(points[i].direction,points[i - 1].direction));
// 					}
// 					if (i <= 1)
// 					{
// 						keyPoint[number].poseAngle = CalculateAngleAdd(points[0].direction , CalculateAngleSub(points[1].direction,points[0].direction) / startLength * length);
// 					}
// 					if (i >= num - 1)
// 					{
// 						keyPoint[number].poseAngle = CalculateAngleAdd(points[num - 2].direction , CalculateAngleSub(points[num - 1].direction,points[num - 2].direction) * (length - beforeLength) / endLength);
// 					}
// 				}
				
// 				//速度
// 				if (num == 2)
// 				{
// 					keyPoint[number].vellMax = sqrtf((1.0f - length/sumLen) * points[0].vel * points[0].vel + length/sumLen*points[1].vel*points[1].vel);
// 				}
// 				else 
// 				{
// 					if (i > 1 && i < num - 1)
// 					{
// 						keyPoint[number].vellMax = sqrtf((1.0f - u) * points[i-1].vel * points[i-1].vel + u*points[i].vel*points[i].vel);
// 					}
// 					if (i <= 1)
// 					{
// 						keyPoint[number].vellMax = sqrtf((1.0f - length/startLength) * points[0].vel * points[0].vel + length/startLength*points[1].vel*points[1].vel);
// 					}
// 					if (i >= num - 1)
// 					{
// 						keyPoint[number].vellMax = sqrtf((1.0f - (length - beforeLength)/endLength) * points[num-2].vel * points[num-2].vel + (length - beforeLength)/endLength*points[num-1].vel*points[num-1].vel);
// 					}
// 				}
// 				number++;
// 			}

// 			lines[0] = lines[1];
// 		}
// 	}
	

// 	// if(gRobot.walkStatus == waitForStart)
// 	// {
// 	// 	keyPoint[0].point =  (Point_t){0.00f,0.00f};
// 	// }
// 	// else
// 	// {
// 	keyPoint[0].point = GetPosPresent().point;
// 	keyPoint[0].angle = keyPoint[1].angle;
// 	keyPoint[0].poseAngle = GetPosPresent().direction;
// 	#ifdef TEST_PATH_FLAG
// 		keyPoint[0].point = testPath[0].point;
// 		keyPoint[0].poseAngle = testPath[0].direction;
// 	#endif
	
// 	//}
// 	// keyPoint[0].point =  (Point_t){514.773	,	5781.569};
	

// 	if(gRobot.walkStatus == waitForStart)
// 	{
// 		// keyPoint[0].poseAngle =31.647768f;
		
		
// 	}
// 	else
// 	{
// 		keyPoint[0].poseAngle = GetPosPresent().direction;
// 	}

// 	// float velXcoor;
// 	// float velYcoor;
// 	// velXcoor = GetSpeedX();
// 	// velYcoor = GetSpeedY();

// 	// if(velXcoor < 100.0f && velYcoor <100.0f)
// 	// {
// 		keyPoint[0].vellMax = points[0].vel;
// 	// }
// 	// else
// 	// {
// 	// 	keyPoint[0].vellMax = sqrtf(GetSpeedX() * GetSpeedX() + GetSpeedY() * GetSpeedY());
// 	// }
	
// 	keyPoint[resultNum].point = points[num-1].point;
// 	keyPoint[resultNum].angle = keyPoint[resultNum - 1].angle;
// 	keyPoint[resultNum].poseAngle = points[num - 1].direction;
// 	keyPoint[resultNum].vellMax = points[num - 1].vel;
	
// 	// for (int i = 0; i < resultNum-1; i++)
// 	// {
// 	// 	printf("BEFORE %d %d %d",(int)keyPoint[i].point.x,(int)keyPoint[i].point.y, (int)keyPoint[i].angle*10);
// 	// 	printf("\n");
// 	// }
// 	for(int i = 1; i < resultNum-1; i++)
// 	{
// 		if((keyPoint[i].angle - keyPoint[i-1].angle) * (keyPoint[i].angle - keyPoint[i+1].angle) > 0 && fabsf(keyPoint[i-1].angle-keyPoint[i+1].angle)<150)
// 		{
// 			if(fabsf((keyPoint[i].angle - keyPoint[i-1].angle))>20&&fabsf((keyPoint[i].angle - keyPoint[i+1].angle))>20)
// 			{
// 				keyPoint[i].angle = 2*keyPoint[i-1].angle/5.0f + 3*keyPoint[i+1].angle/5.0f;
// 			}
// 		}
// 	}
// 	// for (int i = 0; i < resultNum-1; i++)
// 	// {
// 	//	printf("AFTER %d %d %d",(int)keyPoint[i].point.x, (int)keyPoint[i].point.y, (int)keyPoint[i].angle*10);
// 	// 	printf("\n");
// 	// }
	
// 	resultNum++;
	
// 	free(a);
// 	free(b);
// 	free(c);
// 	free(soluctionX);
// 	free(soluctionY);
// 	free(dataX);
// 	free(dataY);
// 	free(controlPoint);
// 	return resultNum;
// }
