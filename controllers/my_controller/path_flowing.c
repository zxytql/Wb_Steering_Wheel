#include "path_flowing.h"
#include "chassis.h"
#include "calculate.h"

int PathFollowing(float percent)
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
	PointU_t virtualPos,virtualTarget;

    if (percent < 0.0f || percent > 1.2f)
    {
        printf("Invalid Para");
        return -1;
    }
    
    float VIEW_L = 0.0f;
    VIEW_L = GetPosPresent();
    
}