#ifndef _TASK_H
#define _TASK_H

#include "bsp_main.h"
#include "robot.h"
#include "cylinder.h"

extern vector_t finalVel;

extern gRobot_t gRobot;

extern void Task_VelControl(void);
extern void Task_GetCasterStatus(void);
//extern void Task_KickControl(void);

extern void Task_RemoteAnalyse(void);
extern void Task_JudgeKickerFlag(void);

void ConfigTask(void);
void TIM2_5ms_Tasks(void);
void TIM5_10ms_Tasks(void);
	

#endif
