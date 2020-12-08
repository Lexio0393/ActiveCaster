#include "main.h"

#include "joystick.h"
extern TYPE_Joystick_t g_Joystick_t;
uint8_t g_Display_usart[500];
uint8_t g_Print_FinishFlag = 0;

extern volatile CasterStatus_t CasterStatus;
extern volatile RemoteStatus_t RemoteStatus;

extern float RockerValue_LX, RockerValue_LY;
extern float RockerValue_RX, RockerValue_RY;
extern gRobot_t gRobot;
extern LocatorInfo_t LocatorInfo;

int main(void)
{

	uint8_t i;
	
	ConfigTask();
	
	while (1)
	{
		Delay_ms(5);
    i++;
		
		if( (i % 2) == 0)
		{
			#if 1
			Analog_Joystick(&g_Joystick_t);
			#endif        
		}
		
		if( (i % 500) == 0)
		{
			#if 1
			g_Print_FinishFlag = 0;
			sprintf((char *)g_Display_usart, 
//			CasterStatus:%d (0x03 is Remote)\r\n
//			RemoteStatus:%d (0x00 is Kick)\r\n
			"\n\
			RockerValue_LX:%f\r\n\
			RockerValue_LY:%f\r\n\
			RockerValue_RX:%f\r\n\
			RockerValue_RY:%f\r\n\
			finalVel.Module: %f\r\n\
			finalVel.Direction: %f\r\n\
			Actual RF LF LR RR: %f %f %f %f\r\n\
			Target RF LF LR RR:	%f %f %f %f\r\n\
			LocatorInfo Yaw:%f\r\n"
			, RockerValue_LX, RockerValue_LY, RockerValue_RX, RockerValue_RY, finalVel.module, finalVel.direction,\
			gRobot.wheelState.rightFrontAct.direction, gRobot.wheelState.leftFrontAct.direction, gRobot.wheelState.leftRearAct.direction, gRobot.wheelState.rightRearAct.direction,\
			gRobot.wheelState.rightFrontTarget.direction, gRobot.wheelState.leftFrontTarget.direction, gRobot.wheelState.leftRearTarget.direction, gRobot.wheelState.rightRearTarget.direction, LocatorInfo.Yaw);
//												, CasterStatus, RemoteStatus, RockerValue_LX, RockerValue_LY, RockerValue_RX, RockerValue_RY, finalVel.module, finalVel.direction);
			g_Print_FinishFlag = 1;
			#endif  
		}
	}
}

#ifdef  USE_FULL_ASSERT

/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t* file, uint32_t line)
{ 
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */

  /* Infinite loop */
  while (1)
  {
  }
}
#endif
