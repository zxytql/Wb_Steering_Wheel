/********************************************************************
*Copyright(C）2014-2016,沈阳艾克申机器人技术开发有限责任公司
*FileName：	   caculate.h
*Author：      Peng Xu
*Date：        2016/10/21
*Description： 平面的数学计算函数头文件
*
*Version：     V1.0
*
********************************************************************/


#ifndef _CALCULATE_H
#define _CALCULATE_H

#include "chassis.h"

//角度制转换为弧度制系数
#define CHANGE_TO_RADIAN    (0.01745329251994f)   
//弧度制转换为角度制系数
#define CHANGE_TO_ANGLE     (57.29577951308232f)				
//圆周率
#ifndef PI
#define PI                  3.1415926f
#endif

#ifndef NULL
#define NULL 0
#endif

//点的结构体 单位mm
typedef struct
{
	float x;
	float y;
}Point_t;

//点斜式结构体 ，斜率用角度制的角度代替
typedef struct
{
	Point_t point;
	//角度制
	float   direction;
	//速度
	float vel;
}Pose_t;

//三轮控制姿态参数
typedef struct
{
	Point_t  point;
	float    angle;
	float    poseAngle;
	float    length;
	float    curvatureR;
	float    vellMax;
	float    accT;
}KeyPointInf_t;

typedef struct
{
	float speed;
	float direction;
	float rotationVell;
}TriWheelVel2_t;


//KeyPointInf_t结构体中存放变量所占字节数大小，用于flash存数。
#define BYTESNUM 7



typedef struct
{
	Point_t point;
	float u;
	float direction;
	unsigned short startPtr;
	unsigned short endPtr;
}PointU_t;

typedef struct
{
	float module;
	float direction;
}vector_t;


//对一个值进行限幅
void ValueClamp(float *value , float max , float min);

float ReturnValueClamp(float value , float max , float min);

float CalculateAngleAdd(float angle1, float angle2);

float CalculateAngleSub(float minuend, float subtrahend);

Point_t CalculateTwoLineIntersection2(Pose_t line1, Pose_t line2);

float CalculateLineAngle(Point_t pointStart, Point_t pointEnd);

Pose_t CalculateLine2(Point_t pointStart, Point_t pointEnd);

float CalculatePoint2PointDistance(Point_t point1, Point_t point2);

float CalculateDisPointToLine(Point_t point, PointU_t line);

void FreeMemory(float **a, int n);

float **CreateMemory(int row_h, int row_l);

void Gauss(float** A, float** B, int n);

void Matrix(float* constantTerm, int num,  float* m,  float* n,  float* k,  float* solution);
/*********************************************************************************
* @name 	CalculateVectorAdd
* @brief	计算用方向和模长表示的两个向量的和
* @param	vector1:向量1;
* @param    vector2:向量2; 
* @retval	返回两个向量的模（用模长和方向表示）
**********************************************************************************/

vector_t CalculateVectorAdd(vector_t vector1 , vector_t vector2);
/*********************************************************************************
* @name 	CalculateVectorProject
* @brief	计算一个向量向另一个向量的投影
* @param	vector:向量;
* @param    axis:要投影的轴; 
* @retval	向量在轴上的投影
**********************************************************************************/

float CalculateVectorProject(vector_t vector , vector_t axis);
/*********************************************************************************
* @name 	CalculateVectorFromProject
* @brief	根据投影计算向量的模长和角度
* @param	project:投影;
* @param    vectorDirection:要求的向量的方向; 
* @param    axisDirection:投影轴的方向;
* @retval	向量在轴上的投影
**********************************************************************************/
vector_t CalculateVectorFromProject(float project , float vectorDirection , float axisDirection);

#endif

