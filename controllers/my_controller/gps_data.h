#ifndef _GPS_DATA_H
#define _GPS_DATA_H

#include "main.h"

float Get_Now_X(void);
float Get_Now_Y(void);
float Get_Walk_Path_Length(void);
void Update_Length_Start(void);
void Update_Length_Stop(void);

void Cal_Walk_Length(void);

#endif