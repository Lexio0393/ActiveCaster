#include "stm32f4xx.h"
#include "robot.h"
#include "bsp_io.h"
#include "bsp_serial.h"

#include "pps.h"
#include "esc.h"
#include "remote.h"
#include "cylinder.h"

#include "joystick.h"


volatile CasterStatus_t CasterStatus = CasterStatus_LostForce;

extern volatile RemoteStatus_t RemoteStatus;

void RobotInit(void)
{

	LocatorInit();
	Beep_On();
	Delay_ms(50);
	Beep_Off();
	
	Motor_Init();
	Joystick_Init();
	KickerReset();
	OrientationClear();
}


void Task_GetCasterStatus(void)
{
	/* 底盘控制模式选择 */
	switch((uint8_t)Remote_GetChanalValue(Remote_B))
	{
		case Remote_SW0:
			CasterStatus = CasterStatus_AutoMode;			//刹车模式
			break;
		case Remote_SW1:
			CasterStatus = CasterStatus_Remote;			//遥控模式 车身速度控制、踢球任务
			break;
		case Remote_SW2:
			CasterStatus = CasterStatus_Reset;		//待机模式 航向轮复位到初始位置，驱动轮抱死
			break;
		default:
			CasterStatus = CasterStatus_LostForce;	 //失能保护
	}
	
		/* 模式触发事件 */
	switch((uint8_t)CasterStatus)
	{
		/* 待机模式 */
		case CasterStatus_LostForce:
			finalVel.module = 0.0f;
			finalVel.direction = GetAngle();
			break;
		
		/* 重置模式 */
		case CasterStatus_Reset:
			finalVel.module = 0;
			finalVel.direction = GetAngle();
			break;
		
		/* 自动模式 */
		case CasterStatus_AutoMode:
//			Task_AutoRunning();	暂时用失力模式代替
			finalVel.module = 0;
			finalVel.direction = 90.0f;
			break;	
	
		/* 遥控模式 */
		case CasterStatus_Remote:
			Task_RemoteAnalyse();
			break;
		
		default:
			finalVel.module = 0;
			finalVel.direction = GetAngle();
			break;
	}
	
//	// 额外功能控制 待添加
//	switch((uint8_t)Remote_GetChanalValue(Remote_E))
//	{
//		case Remote_SW0:	//Value Locked 
//			break;
//		case Remote_SW2:	// Value Unlock
//			break;
//	}
}	



