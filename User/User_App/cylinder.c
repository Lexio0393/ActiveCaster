#include "cylinder.h"
#include "robot.h"
#include "manul.h"

#include "pack_deal.h"
#include "joystick.h"
#include "remote.h"

extern volatile RemoteStatus_t RemoteStatus;
extern volatile CasterStatus_t CasterStatus;

void KickerPush(void)
{
		g_Joystick_t.Key1 |= KEY1_R_DIR_UP;		//	GPIO_SetBits(GPIO_Port_Push, GPIO_Pin_Push);
}

void KickerPull(void)
{
	g_Joystick_t.Key1 |= KEY1_R_DIR_DOWN;		//	GPIO_ResetBits(GPIO_Port_Push, GPIO_Pin_Push);
	g_Joystick_t.Key2 |= KEY2_L_DIR_UP;			//	GPIO_SetBits(GPIO_Port_Pull, GPIO_Pin_Pull);
	g_Joystick_t.Key2 |= KEY2_L_DIR_DOWN;		//	GPIO_ResetBits(GPIO_Port_Pull, GPIO_Pin_Pull);
}

void KickerHold(void)
{
	g_Joystick_t.Key1 |= KEY1_R_DIR_DOWN;		//GPIO_ResetBits(GPIO_Port_Push, GPIO_Pin_Push);
	g_Joystick_t.Key2 |= KEY2_L_DIR_DOWN;		//GPIO_ResetBits(GPIO_Port_Pull, GPIO_Pin_Pull);
}
void KickerCmdReset(void)
{
	gRobot.kicker.kickFlag = 0;
	gRobot.kicker.ready    = 0;
}

void KickerReset(void)
{
	gRobot.kicker.ballNum = 0;
//	GasPressureControl(0.0f);
	KickerCmdReset();
}

void KickerKick(void)
{
		Delay_ms(50);
		KickerPush();
		Delay_ms(50);
		
		KickerPull();
		KickerPull();
		Delay_ms(50);
//	if(KickerCheckState() == KICKER_ADJ_READY)
//	{
//		Delay_ms(50);
//		KickerPush();
//		Delay_ms(50);
//		KickerPull();
//		Delay_ms(50);
//		gRobot.kicker.kickFlag = KICKER_FINISH;
//	}
//	else if(KickerCheckState() == KICKER_ADJUSTING)
//	{
//		KickerHold();
//	}
}

uint8_t KickerCheckState(void)
{
	return gRobot.kicker.ready;
}

uint8_t KickerFinishState(void)
{
	return gRobot.kicker.kickFlag;
}

//	void GasPressureControl(float gasPressure)
//	{
//	//对输入气压值进行限幅
//	if(gasPressure<=0.0f)
//	{
//		gasPressure = 0.0f;
//	}
//	else if(gasPressure>=0.9f)
//	{
//		gasPressure = 0.9f;	//数值待修^改
//	}
//	gRobot.debugInformation.gasPressure = gasPressure;

//	//xinbilifa 公式待修改
//	gasPressure = (10000.f/3060.f*4096/3.3f)*((gasPressure+0.0007f)/0.9f);

//	//	DAC_SetChannel1Data(DAC_Align_12b_R,gasPressure);//12bit align right
//	}

//float GetGasPressure(void)
//{
//	float gasADCValue = GetGasPressureADCValue();
//	while(1)
//	{
//		
//	}
//	//fix me hard number
//	//jiubilifa
//	return (gasADCValue - GAS_ADC_ZERO)*1.132f/2048.f + 0.003f;
//	//xinbilifa
//	return (gasADCValue - 338.1667f)*1.16f/2048.f - 0.02f;
//}

void Task_JudgeKickerFlag(void)
{
//	KickerReset();

	switch((uint8_t)Remote_GetChanalValue(Remote_E))
	{
		case Remote_SW0:
			
			gRobot.kicker.kickFlag = KICKER_UNFINISH;
			gRobot.kicker.ready    = KICKER_ADJ_READY;
			break;
		
		case Remote_SW2:
			gRobot.kicker.kickFlag = KICKER_FINISH;
			gRobot.kicker.ready    = KICKER_ADJUSTING;				
			break;
		
		default:
			KickerCmdReset();
	}
}

//void KickControl(void)
//{
//	switch((uint8_t)RemoteStatus)
//	{
//		case RemoteStatus_LostForce:
//		case RemoteStatus_Breaking:
//		case RemoteStatus_VelLoop:
//		
//				KickerReset();
//				break;
//		
//		case	RemoteStatus_Agjust:
//				gRobot.kicker.ready = KICKER_ADJUSTING;
//				KickerPull();
//		
//				Delay_ms(50);
//				KickerHold();
//				break;
//		
//		case	RemoteStatus_Kick:
//				
//			if(KickerFinishState() == KICKER_UNFINISH)
//			{
//				if(KickerCheckState() == KICKER_ADJ_READY)
//				{
//					KickerKick();
//					Delay_ms(50);
//				
//					gRobot.kicker.kickFlag = KICKER_FINISH;
//					gRobot.kicker.ballNum += 1;
//				}
//			}
//			else
//			{
//				KickerPull();
//			}
//			break;
//			
//		default:
//				KickerReset();
//	}
//}

//void Task_GetKickerStatus(void)
//{
//	switch((uint8_t)Remote_GetChanalValue(Remote_B))
//	{
//		case Remote_SW0:
//			CasterStatus = CasterStatus_Break;			//刹车模式
//			break;
//		case Remote_SW1:
//			CasterStatus = CasterStatus_Remote;			//遥控模式 车身速度控制、踢球任务
//			break;
//		case Remote_SW2:
//			CasterStatus = CasterStatus_Standby;		//待机模式 航向轮复位到初始位置，驱动轮抱死
//			break;
//		default:
//			CasterStatus = CasterStatus_LostForce;	 //失能保护
//	}
//}


//void Task_KickControl(void)
//{
//	switch((uint8_t)RemoteStatus)
//	{
//		case RemoteStatus_VelLoop:
//				KickerReset();
//		
//		case	RemoteStatus_Agjust:
//				gRobot.kicker.ready = KICKER_ADJUSTING;
//				KickerPull();
//		
//				Delay_ms(50);
//				KickerHold();
//			
//		case	RemoteStatus_Kick:
//				gRobot.kicker.ready = KICKER_ADJ_READY;
//				Delay_ms(50);
//				KickerKick();
//				
//				Delay_ms(50);
//				KickerCmdReset();
//		default:
//				KickerReset();
//	}
//}




