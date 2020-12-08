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
	/* ���̿���ģʽѡ�� */
	switch((uint8_t)Remote_GetChanalValue(Remote_B))
	{
		case Remote_SW0:
			CasterStatus = CasterStatus_AutoMode;			//ɲ��ģʽ
			break;
		case Remote_SW1:
			CasterStatus = CasterStatus_Remote;			//ң��ģʽ �����ٶȿ��ơ���������
			break;
		case Remote_SW2:
			CasterStatus = CasterStatus_Reset;		//����ģʽ �����ָ�λ����ʼλ�ã������ֱ���
			break;
		default:
			CasterStatus = CasterStatus_LostForce;	 //ʧ�ܱ���
	}
	
		/* ģʽ�����¼� */
	switch((uint8_t)CasterStatus)
	{
		/* ����ģʽ */
		case CasterStatus_LostForce:
			finalVel.module = 0.0f;
			finalVel.direction = GetAngle();
			break;
		
		/* ����ģʽ */
		case CasterStatus_Reset:
			finalVel.module = 0;
			finalVel.direction = GetAngle();
			break;
		
		/* �Զ�ģʽ */
		case CasterStatus_AutoMode:
//			Task_AutoRunning();	��ʱ��ʧ��ģʽ����
			finalVel.module = 0;
			finalVel.direction = 90.0f;
			break;	
	
		/* ң��ģʽ */
		case CasterStatus_Remote:
			Task_RemoteAnalyse();
			break;
		
		default:
			finalVel.module = 0;
			finalVel.direction = GetAngle();
			break;
	}
	
//	// ���⹦�ܿ��� �����
//	switch((uint8_t)Remote_GetChanalValue(Remote_E))
//	{
//		case Remote_SW0:	//Value Locked 
//			break;
//		case Remote_SW2:	// Value Unlock
//			break;
//	}
}	



