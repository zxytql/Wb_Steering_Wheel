#ifndef _RINGBUFFER_H
#define _RINGBUFFER_H

#include <stdint.h>

/*********************************************************************
* @name 		addring
* @brief  	向环形缓冲区中放入一个点
* @param  	pose：添加的元素
* @retval 	返回是否将环形数组填满 1：未满； 0：已经满了
********************************************************************/
int PutRingBuffer(KeyPointInf_t pose);

/*********************************************************************
* @name 		GetRingBufferPoint
* @brief  	返回一个点    
* @param  	num取1代表取区间里的第一个数，没有第0个数
* @retval 	返回所取点坐标
********************************************************************/
Point_t GetRingBufferPoint(int num);

/*********************************************************************
* @name 	SetRingBufferPointAccT
* @brief  	设置一个加速度    
* @param  	num取1代表取区间里的第一个数，没有第0个数
* @retval
********************************************************************/
void SetRingBufferPointAccT(int num,float accT);


/*********************************************************************
* @name 	GetRingBufferPointAccT
* @brief  	返回一个加速度    
* @param  	num取1代表取区间里的第一个数，没有第0个数
* @retval 	返回所取点坐标
********************************************************************/
float GetRingBufferPointAccT(int num);


/*********************************************************************
* @name 		GetRingBufferPointAngle
* @brief  	返回一个角度    
* @param  	num取1代表取区间里的第一个数，没有第0个数
* @retval 	返回所取点角度信息
********************************************************************/
float GetRingBufferPointAngle(int num);


/*********************************************************************
* @name 		GetRingBufferPointPoseAngle
* @brief  	返回一个三轮的姿态角度    
* @param  	num取1代表取区间里的第一个数，没有第0个数
* @retval 	返回所取点角度信息
********************************************************************/
float GetRingBufferPointPoseAngle(int num);


/*********************************************************************
* @name 		GetCount
* @brief  	返回储存器元素个数    
* @param  	无
* @retval 	无
********************************************************************/
int GetCount(void);

/*********************************************************************
* @name 		GetUpPointer
* @brief  	返回上指针   
* @param  	无
* @retval 	无
********************************************************************/
int GetUpPointer(void);



/*********************************************************************
* @name 		GetDownPointer
* @brief  	返回下指针   
* @param  	无
* @retval 	无
********************************************************************/
int GetDownPointer(void);



/*********************************************************************
* @name 		DeleteData
* @brief  	删除数据点  
* @param  	num 所要删除点的个数。从第一个点开始删除
* @retval 	无
********************************************************************/
void DeleteData(int num);



/*********************************************************************
* @name 		SetUpPointer
* @brief  	设置上指针   
* @param  	无
* @retval 	无
********************************************************************/
void SetUpPointer(int num);


uint32_t* GetFristAdress(void);

void SetRingBufferPointVell(int num,float vell);
float GetRingBufferPointVell(int num);
float GetRingBufferPointLen(int num);
float GetRingBufferAverCurvature(int num);
float GetLength(void);

void dotNumAcquire(int num);
int dotNumPresent(void);

void SetLength(float len);
float GetPredictTime(void);

#endif


