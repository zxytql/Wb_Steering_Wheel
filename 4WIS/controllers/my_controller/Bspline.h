#ifndef _BSPLINE_H
#define _BSPLINE_H
#include "calculate.h"


float CaculateBsplineLen(Point_t point1, Point_t point2, float angle1, float angle2);
PointU_t SerchBsplineVirtualPoint(Point_t point1, Point_t point2, float angle1, float angle2, float robotlen);
PointU_t SerchVirtualPoint(float robotLen);
PointU_t SerchVirtualPoint2(float robotLen);
int BspSegment(int num, Pose_t* points, KeyPointInf_t* keyPoint);

#endif



