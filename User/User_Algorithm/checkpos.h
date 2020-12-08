#ifndef _CHECKPOS_H
#define _CHECKPOS_H

#include "math.h"

#include "movebase.h"
#include "motion.h"

#include "pps.h"
#include "robot.h"


void CheckPos(void);

uint8_t JudgeStop(float disChange,uint8_t countTime);
uint8_t JudgeStop2(float disChange,uint8_t countTime);
uint8_t JudgeSpeedLessEqual(float speedCompared);

float CalculateLineAngle(Point_t actualPos, Point_t targetPos);
Pose_t GetPosPresent(void);


#endif
