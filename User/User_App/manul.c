#include "movebase.h"
#include "motion.h"

#include "pps.h"
#include "esc.h"
#include "manul.h"
#include "remote.h"
#include "robot.h"

const float Caster_VelRatio = 3.2721054f;
const float	Caster_OmeRatio = 0.0107458f;
//M3508极限转速为8192rpm,则最大轮速为8192*Pi*D/60/19 = 3160.5523869f mm/s
//摇杆值范围为-683*SQRT_2~0~683*SQRT_2  速度系数为MaxVelocity/(683*SQRT_2)

//旋转线速度为3160.5523869f mm/s	MaxVelocity/MOVEBASE_RADIUS (430.63f) = 7.3393688°/s
//摇杆值范围为-683~0~683			7.3393688/683 = 0.01074578°/s

//RockerValue * Caster_Ratio所得值的单位为mm/s或°，范围为(-3160 ~ 3160)(-180° ~ 180°)
//该系数负责将摇杆值转化为车速或旋转角度

/*精确位数有待修改 */
float RockerValue_LX = 0.000f, RockerValue_LY = 0.000f;
float RockerValue_RX = 0.000f, RockerValue_RY = 0.000f;
float Vel_Vector = 0.000f, Vel_ControlValue = 0.000f;	//Vel_ControlValue = Vel_Vector*Caster_VelRatio
	
volatile RemoteStatus_t RemoteStatus = RemoteStatus_Reset;

void Task_RemoteAnalyse(void)
{
	float Dir_Change = 0.0f;
	
	switch((uint8_t)Remote_GetChanalValue(Remote_E))
	{
	
		case Remote_SW0:
			
			RemoteStatus = RemoteStatus_VelLoop_Kick;
			RockerValue_RX = -Remote_GetChanalValue(Remote_RX);		
			RockerValue_RY = Remote_GetChanalValue(Remote_RY);
			RockerValue_LX = Remote_GetChanalValue(Remote_LX);
			
			gRobot.debugInformation.CasterVel.omega = RockerValue_LX * Caster_OmeRatio * 2;
		
			arm_sqrt_f32(RockerValue_RX * RockerValue_RX + RockerValue_RY * RockerValue_RY, &Vel_Vector);
			Vel_ControlValue = Vel_Vector * Caster_VelRatio	/ 3;		//3为限速，可去
		
			finalVel.module = Vel_ControlValue;
		
			if(finalVel.module>0.01f)
			{
				if(RockerValue_RY == 0 && RockerValue_RX == 0)
					finalVel.direction = 90.0f;
				else
					finalVel.direction = RAD2ANGLE(atan2f(RockerValue_RY, RockerValue_RX));
			}
			else
			{
				finalVel.direction = 90.0f;
			}	
			break;
			
		case Remote_SW2:
			
			RemoteStatus = RemoteStatus_VelLoop_Rec;	
			RockerValue_RX = -Remote_GetChanalValue(Remote_RX);		
			RockerValue_RY = Remote_GetChanalValue(Remote_RY);
			RockerValue_LX = Remote_GetChanalValue(Remote_LX);
			
			gRobot.debugInformation.CasterVel.omega = RockerValue_LX * Caster_OmeRatio * 2;		//*2
		
			arm_sqrt_f32(RockerValue_RX * RockerValue_RX + RockerValue_RY * RockerValue_RY, &Vel_Vector);
			Vel_ControlValue = Vel_Vector * Caster_VelRatio	/ 3;		//3为限速，可去
		
			finalVel.module = Vel_ControlValue;
		
			if(finalVel.module>0.01f)
			{
				if(RockerValue_RY == 0 && RockerValue_RX == 0)
				{
					finalVel.direction = -90.0f;
				}
				else if(RockerValue_RY >= 0)
				{
					Dir_Change = RAD2ANGLE(atan2f(RockerValue_RY, RockerValue_RX)) - 180.0f;
					finalVel.direction = ReturnLimitAngle(Dir_Change);
				}
				else if(RockerValue_RY <= 0)
				{
					Dir_Change = RAD2ANGLE(atan2f(RockerValue_RY, RockerValue_RX)) + 180.0f;
					finalVel.direction = ReturnLimitAngle(Dir_Change);
				}
				else
				{
					finalVel.direction = -90.0f;
				}
			}
			else
			{
				finalVel.direction = -90.0f;
			}	
			break;
		
		default:
			
			RemoteStatus = RemoteStatus_Reset;
		
			finalVel.module = 0;
			finalVel.direction = 90.0f;
			break;
	}
}


//			RemoteStatus = RemoteStatus_Rocker;
////		RockerValue_LX = Remote_GetChanalValue(Remote_LX) * Caster_DirRatio;	//-180°到180° 对应摇杆值 -683到683
//			RockerValue_LX = Remote_GetChanalValue(Remote_LX) * Caster_DirRatio / 2.0f;	//-90°到90° 对应摇杆值 -683到683
//			RockerValue_LY = -Remote_GetChanalValue(Remote_LY) * Caster_VelRatio;	//mm/s
//			
//			finalVel.module    = RockerValue_LY;			
//			
//			if(finalVel.module>0.01f)
//			{
//				finalVel.direction = 90.0f - RockerValue_LX;
//			}
//			else
//			{
//				finalVel.direction = 90.0f;
//			}
//			break;

