#include "task.h"
#include "cylinder.h"


vector_t finalVel = {0};

gRobot_t gRobot = {0};

void ConfigTask(void)
{
	BSP_Init();
	Beep_On();
	Delay_ms(50);
	Beep_Off();
	RobotInit();
}


void TIM2_5ms_Tasks(void)
{
	static uint32_t Counter = 0;
	Counter++;

//	Task_VelControl();
	VelControl();	
//	KickControl();

	if(Counter >= 200)
	{
		Counter = 0;
		
		LED0_Toggle();
	}
}

void TIM5_10ms_Tasks(void)
{
	static uint32_t Counter = 0;
	Counter++;
	
	Task_GetCasterStatus();
	Task_JudgeKickerFlag();

	if(Counter >= 50)
	{
		Counter = 0;
		
		LED1_Toggle();
	}
}
