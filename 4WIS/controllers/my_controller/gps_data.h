#ifndef _GPS_DATA_H
#define _GPS_DATA_H

#include "main.h"
#include "calculate.h"

float Get_Now_X(void);
float Get_Now_Y(void);
float Get_Walk_Path_Length(void);
void Update_Length_Start(void);
void Update_Length_Stop(void);
float Get_Now_Angle(void);
void Cal_Walk_Length(void);
Pose_t Get_Pos_Present(void);
float Get_Speed_X(void);
float Get_Speed_Y(void);
void Add_Path_Length(float);
void Reduce_Path_Length(float);
float Get_AngularSpeed_W(void);

#endif