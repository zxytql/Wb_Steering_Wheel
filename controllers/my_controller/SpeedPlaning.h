#ifndef _SPEEDPLANING_H
#define _SPEEDPLANING_H

#include "MotionCard.h"
float GetVelLimit(void);
float GetAccLimit(float posX,float posY);
float CalculateAccT(float accN);
void CalculateWheelVell(float* wheelOne,float* wheelTwo,float* wheelThree);
void DynamicalAjusting(float* wheelOne, float* wheelTwo, float* wheelThree);
float DecreseVellByOneWheel(float vellCar, float orientation, float rotationalVell,float zAngle, int wheelNum, float targetWheelVell);
void SpeedPlaning(void);
float GetOmegaMax(void);
float GetAccLimit2(void);




#endif

