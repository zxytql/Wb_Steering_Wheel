/********************************************************************
*Copyright(C）2014-2016,沈阳艾克申机器人技术开发有限责任公司
*FileName：	   caculate.c
*Author：      Peng Xu
*Date：        2016/10/21
*Description： 平面的数学计算函数
*
*Version：     V1.0
*
********************************************************************/


#include "math.h"
#include "calculate.h"
#include <stdlib.h>

//对一个值进行限幅
void ValueClamp(float *value , float max , float min)
{
	*value = (*value) < (min) ? (min) : ((*value) > (max) ? (max) : (*value));
}

float ReturnValueClamp(float value , float max , float min)
{
	value = (value) < (min) ? (min) : ((value) > (max) ? (max) : (value));
	return value;
}

/*********************************************************************************
* @name 	CalculateVectorAdd
* @brief	计算用方向和模长表示的两个向量的和
* @param	vector1:向量1;
* @param    vector2:向量2; 
* @retval	返回两个向量的模（用模长和方向表示）
**********************************************************************************/

vector_t CalculateVectorAdd(vector_t vector1 , vector_t vector2)
{
	vector_t result = {0};
	Point_t vector1InCoodinate = {0} , vector2InCoodinate = {0} , resultInCoodinate = {0};
	
	//将两个向量转换为坐标表示
	vector1InCoodinate.x = vector1.module*cosf(CHANGE_TO_RADIAN*vector1.direction);
	vector1InCoodinate.y = vector1.module*sinf(CHANGE_TO_RADIAN*vector1.direction);
	
	vector2InCoodinate.x = vector2.module*cosf(CHANGE_TO_RADIAN*vector2.direction);
	vector2InCoodinate.y = vector2.module*sinf(CHANGE_TO_RADIAN*vector2.direction);
	
	//通过坐标表示两个向量的和
	resultInCoodinate.x = vector1InCoodinate.x + vector2InCoodinate.x;
	resultInCoodinate.y = vector1InCoodinate.y + vector2InCoodinate.y;
	
	//将结果表示为模长和方向的形式
	result.module = sqrtf(pow(resultInCoodinate.x,2) + pow(resultInCoodinate.y , 2));
	result.direction = (atan2f(resultInCoodinate.y, resultInCoodinate.x) * CHANGE_TO_ANGLE);
	
	return result;
}
/*********************************************************************************
* @name 	CalculateVectorProject
* @brief	计算一个向量向另一个向量的投影
* @param	vector:向量;
* @param    axis:要投影的轴; 
* @retval	向量在轴上的投影
**********************************************************************************/

float CalculateVectorProject(vector_t vector , vector_t axis)
{
	float result = {0};
	float includedAngle = 0.0f;
	
	includedAngle = vector.direction - axis.direction;
	
	result = vector.module * cosf(CHANGE_TO_RADIAN*includedAngle);
	
	return result;
}

/*********************************************************************************
* @name 	CalculateVectorFromProject
* @brief	根据投影计算向量的模长和角度
* @param	project:投影;
* @param    vectorDirection:要求的向量的方向; 
* @param    axisDirection:投影轴的方向;
* @retval	向量在轴上的投影
**********************************************************************************/


vector_t CalculateVectorFromProject(float project , float vectorDirection , float axisDirection)
{
	vector_t result = {0};
	float includedAngle = 0.0f;
	
	includedAngle = vectorDirection - axisDirection;
	
	result.module = project/cosf(CHANGE_TO_RADIAN*includedAngle);
	
	result.direction = vectorDirection;
	
	return result;
}


/*********************************************************************************
* @name 	CalculateAngleAdd
* @brief	对-180,180交界处作处理
* @param	angle1:角度1;
* @param    angle2:角度2;
* @retval   返回相加之后的角度 -180~180
**********************************************************************************/
float CalculateAngleAdd(float angle1, float angle2)
{
	float result = 0.0f;
	result = angle1 + angle2;
	if (result >  180.0f)  result -= 360.0f;
	if (result < -180.0f)  result += 360.0f;
	return result;
}



/*********************************************************************************
* @name 	CalculateAngleSub
* @brief	对-180,180交界处作处理
* @param	minuend: 被减数;
			subtrahend: 减数 A - B,A为被减数，B为减数;
* @retval	返回计算后的角度值 -180~180
*********************************************************************************/
float CalculateAngleSub(float minuend, float subtrahend)
{
	float result = 0.0f;
	result = minuend - subtrahend;
	if (result >  180.0f)  {result -= 360.0f;}
	if (result < -180.0f)  {result += 360.0f;}
	return result;
}

/*********************************************************************************
* @name 	CalculateTwoLineIntersection2
* @brief	计算两条直线的交点
* @param	line1:直线1;
* @param    line2:直线2; 
* @retval	返回交点坐标
**********************************************************************************/
Point_t CalculateTwoLineIntersection2(Pose_t line1, Pose_t line2)
{
	Point_t intersection;
	//斜率
	float k1 = 0.0f;
	float k2 = 0.0f;

	//因为浮点运算,未对与x轴垂直的直线处理。
	k1 = tan(line1.direction * CHANGE_TO_RADIAN);
	k2 = tan(line2.direction * CHANGE_TO_RADIAN);

	intersection.x = (line1.point.x*k1 - line1.point.y - line2.point.x * k2 + line2.point.y)
		/ (k1 - k2);
	intersection.y = k1 * (intersection.x - line1.point.x) + line1.point.y;

	return intersection;
}


/*********************************************************************************
* @name 	CalculateLineAngle
* @brief	计算两点矢量方向角度
* @param	pointStart:起始点：
* @param	pointEnd:终止点;
* @retval	两点矢量方向角度 -180~180
*********************************************************************************/
float CalculateLineAngle(Point_t pointStart, Point_t pointEnd)
{
	float a = 0.0f;
	float b = 0.0f;

	a = pointEnd.y - pointStart.y;
	b = pointEnd.x - pointStart.x;
	//atan2f范围可以包含-180到180  
	return (atan2f(a, b) * CHANGE_TO_ANGLE);
}

/*********************************************************************************
* @name 	CalculateLine2
* @brief	两点确定一条直线
* @param	pointStart:起始点;
* @param	pointEnd:终止点;
* @retval   Pose_t形式，包括直线方向角度，直线上一点坐标
*********************************************************************************/
Pose_t CalculateLine2(Point_t pointStart, Point_t pointEnd)
{
	float a = 0.0f;
	float b = 0.0f;
	Pose_t line2;

	a = pointEnd.y - pointStart.y;
	b = pointEnd.x - pointStart.x;
	line2.direction = (atan2f(a, b) * CHANGE_TO_ANGLE);
	line2.point = pointStart;
	//atan2f范围可以包含-180到180  
	return line2;
}

/*********************************************************************************
* @name 	CalculatePoint2PointDistance
* @brief	计算点到点的距离
* @param	point1 起始点
* @param	point2 结束点
* @retval   返回两点之间的距离
*********************************************************************************/
float CalculatePoint2PointDistance(Point_t point1, Point_t point2)
{
	float dis;
	dis = sqrt((point1.x - point2.x) * (point1.x - point2.x) + (point1.y - point2.y) * (point1.y - point2.y));
	return dis;
}





/***********************************************************************************
* @name 	CalculateDisPointToLine
* @brief	计算点到直线距离 返回值为负代表在直线右侧，为正代表在直线左侧
* @param	point 点坐标
* @param	line  直线参数
* @retval	点到直线的距离
**********************************************************************************/
float CalculateDisPointToLine(Point_t point, PointU_t line)
{
	float dis, k, b;

	if(fabs(line.direction) == 90.0f)
	{
		dis = point.x;
	}
	else
	{
		k = tanf(line.direction * CHANGE_TO_RADIAN);
		b = line.point.y - k * line.point.x;
		dis = -(k * point.x + b - point.y) / sqrt(1 + k * k);
	}

	return dis;
}

/*********************************************************************************
* @name 	**CreateMemory(int row_h, int row_l )
* @brief	开辟row_h*row_l的内存，并赋初值为0
* @param	row_h:行数 row_l:列数
* @retval
**********************************************************************************/
float **CreateMemory(int row_h, int row_l )
{
	float **a = NULL;
	int i, j;

	//为二维数组分配row行
	a = (float**)malloc(sizeof(void *)* row_h);
	//为每列分配n个大小空间
	for (i = 0; i < row_h; ++i){
		a[i] = (float*)malloc(sizeof(float)* row_l);
	}
	for (i = 0; i < row_h; i++){
		for (j = 0; j < row_l; j++){
			a[i][j] = 0;
		}
	}

	return a;
}

/*********************************************************************************
* @name 	FreeMemory(float **a, int n)
* @brief	释放内存函数
* @param	a：所释放的二位数组首地址；   row_h:数组行数；
* @retval
**********************************************************************************/
void FreeMemory(float **a, int row_h)
{
	
	
	int i;
	for (i = 0; i < row_h; ++i){
		free(a[i]);
	}
	free(a);
}

/*********************************************************************************
* @name 	Gauss
* @brief	采用部分主元的高斯消去法求方阵A的逆矩阵B
* @param	A:输入方阵;B:输出方阵;
* @retval
**********************************************************************************/
void Gauss(float** A, float** B, int n)
{
	int i, j, k;
	float max, temp;
	float **t = NULL;

	t = CreateMemory(n,n);
	// **t  临时矩阵  
	//将A矩阵存放在临时矩阵t[n][n]中  
	for (i = 0; i < n; i++)
	{
		for (j = 0; j < n; j++)
		{
			t[i][j] = A[i][j];
		}
	}
	//初始化B矩阵为单位阵  
	for (i = 0; i < n; i++)
	{
		for (j = 0; j < n; j++)
		{
			B[i][j] = (i == j) ? (float)1 : 0;
		}
	}
	for (i = 0; i < n; i++)
	{
		//寻找主元  
		max = t[i][i];
		k = i;
		for (j = i + 1; j < n; j++)
		{
			if (fabs(t[j][i]) > fabs(max))
			{
				max = t[j][i];
				k = j;
			}
		}
		//如果主元所在行不是第i行，进行行交换  
		if (k != i)
		{
			for (j = 0; j < n; j++)
			{
				temp = t[i][j];
				t[i][j] = t[k][j];
				t[k][j] = temp;
				//B伴随交换  
				temp = B[i][j];
				B[i][j] = B[k][j];
				B[k][j] = temp;
			}
		}
		//判断主元是否为0, 若是, 则矩阵A不是满秩矩阵,不存在逆矩阵  
		if (t[i][i] == 0)
		{

		}
		//消去A的第i列除去i行以外的各行元素  
		temp = t[i][i];
		for (j = 0; j < n; j++)
		{
			//主对角线上的元素变为1  
			t[i][j] = t[i][j] / temp;
			//伴随计算  			
			B[i][j] = B[i][j] / temp;
		}
		//第0行->第n行  
		for (j = 0; j < n; j++)
		{
			//不是第i行  
			if (j != i)
			{
				temp = t[j][i];
				//第j行元素 - i行元素*j列i行元素  
				for (k = 0; k < n; k++)
				{
					t[j][k] = t[j][k] - t[i][k] * temp;
					B[j][k] = B[j][k] - B[i][k] * temp;
				}
			}
		}
	}

	FreeMemory(t,n);
}


/*************************************************************
* @name Matrix
* @brief 追赶法求解线性方程组
* @constantTerm 线性方程组等号右边的列矩阵
* @solution 解
* @num 行数或列数
* @m 系数矩阵对角数组
* @n 对角上方数组
* @k 对角下方数组
* ***********************************************************/

void Matrix(float* constantTerm, int num,  float* m,  float* n,  float* k,  float* solution)
{
	//a为分解后的下三角矩阵的对角数组
	float* a = NULL;
	a = (float *)malloc(sizeof(float)* num);
	//b为分解后的单位上三角矩阵的对角上方数组
	float* b = NULL;
	b = (float *)malloc(sizeof(float)* (num - 1));
	//c为分解后的单位上三角矩阵的对角上方数组
	float* c = NULL;
	c = (float *)malloc(sizeof(float)* num);
	//x为求解过程中的间接解
	float* x = NULL;
	x = (float *)malloc(sizeof(float)* num);
	int i;

	a[0] = m[0];
	b[0] = n[0] / a[0];

	//给分解后下三角矩阵的对角下方数组c赋值
	for (i = 1; i < num; i++)
	{
		c[i] = k[i];
	}


	//给分解后的单位上三角矩阵的对角上方数组a和分解后的单位上三角矩阵的对角上方数组b赋值
	for (i = 1; i < num - 1; i++)
	{
		a[i] = m[i] - k[i] * b[i - 1];
		b[i] = n[i] / a[i];

	}

	a[num - 1] = m[num - 1] - k[num - 1] * b[num - 2];
	//中间解x的初始值
	x[0] = constantTerm[0] / a[0];

	//给中间解赋值
	for (i = 1; i < num; i++)
	{
		x[i] = (constantTerm[i] - k[i] * x[i - 1]) / a[i];
	}

	//解出最终解
	solution[num - 1] = x[num - 1];

	for (i = num - 1; i > 0; i--)
	{
		solution[i - 1] = x[i - 1] - solution[i] * b[i - 1];
	}
	free(a);
	free(b);
	free(c);
	free(x);
}
