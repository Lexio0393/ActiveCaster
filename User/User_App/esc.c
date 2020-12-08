#include "esc.h"
#include "movebase.h"

#include "robot.h"
#include "remote.h"
#include "task.h"
#include "manul.h"

extern volatile CasterStatus_t CasterStatus;
extern volatile RemoteStatus_t RemoteStatus;
extern vector_t finalVel;
extern float RockerValue_LX, RockerValue_LY;

Motor_t C610[4],C620[4] = {0};
PID_Regular_t PID_C610_Position[4], PID_C620_Position[4];
PID_Increment_t PID_C610_Velocity[4], PID_C620_Velocity[4];

void VelControl(void)
{
	switch((uint8_t)CasterStatus)
	{
		case CasterStatus_LostForce:
			
			Task_DisableControl();
			break;

		case CasterStatus_AutoMode:
			
			Task_DisableControl();
			break;
		
		case CasterStatus_Remote:
		
			gRobot.debugInformation.CasterVel.vel 			= finalVel.module;
			gRobot.debugInformation.CasterVel.direction = finalVel.direction;
		
			Task_RemoteControl();
			break;
		
		case CasterStatus_Reset:	//预计改为reset模式
			
			gRobot.debugInformation.CasterVel.vel 			= finalVel.module;
			gRobot.debugInformation.CasterVel.direction = finalVel.direction;
		
			OutputVel2Wheel(0, 90.0f, 0);
//			SendCmd2Driver(0.0f, 135.0f, 0.0f, 135.0f, 0.0f, 135.0f, 0.0f, 135.0f);
			SendPosCmd(CAN2, ID_C610_CTRL, C610);
			SetJoggingVel(CAN1, ID_C620_CTRL, C620);
			break;
				
		default:
			Task_DisableControl();
			break;
	}
}

void Task_RemoteControl(void)
{
	switch((uint8_t)RemoteStatus)
	{
		case RemoteStatus_VelLoop_Rec:	/* 接球任务方向 */
			
			OutputVel2Wheel(gRobot.debugInformation.CasterVel.vel, \
							gRobot.debugInformation.CasterVel.direction, 
							gRobot.debugInformation.CasterVel.omega);
			
			SendPosCmd(CAN2, ID_C610_CTRL, C610);
			SetJoggingVel(CAN1, ID_C620_CTRL, C620);
			break;
		
		case RemoteStatus_VelLoop_Kick:	/* 踢球任务方向 */

			OutputVel2Wheel(gRobot.debugInformation.CasterVel.vel, \
							gRobot.debugInformation.CasterVel.direction, 
							gRobot.debugInformation.CasterVel.omega);
			
			SendPosCmd(CAN2, ID_C610_CTRL, C610);
			SetJoggingVel(CAN1, ID_C620_CTRL, C620);
			break;		
		
		case RemoteStatus_Reset:
			
			OutputVel2Wheel(0, 90.0f, 0);
		
			SendPosCmd(CAN2, ID_C610_CTRL, C610);
			SetJoggingVel(CAN1, ID_C620_CTRL, C620);
			break;

	}
}

void Motor_GetFeedbackInfo(CanRxMsg* RxMsg)
{
	static uint16_t PrevAngle;
	static int16_t DeltaPulse;
	
	static uint16_t ESC_ID;
	static uint8_t  ESC_Index;
	
	ESC_ID = RxMsg->StdId;
	
	switch(ESC_ID)
	{
		case RIGHT_FRONT_TURNING_ID:
		case LEFT_FRONT_TURNING_ID:
		case LEFT_REAR_TURNING_ID:
		case RIGHT_REAR_TURNING_ID:
			
			ESC_ID = ESC_ID - ID_C610_Base;
			ESC_Index = ESC_ID;
			
			PrevAngle = C610[ESC_Index].Angle;
			C610[ESC_Index].Angle		= (uint16_t)(RxMsg -> Data[0] << 8 | RxMsg -> Data[1]);
			C610[ESC_Index].Velocity	= ( int16_t)(RxMsg -> Data[2] << 8 | RxMsg -> Data[3]);
			C610[ESC_Index].Current		= ( int16_t)(RxMsg -> Data[4] << 8 | RxMsg -> Data[5]);
			DeltaPulse = C610[ESC_Index].Angle - PrevAngle;
			
			if(DeltaPulse > 4095)
				DeltaPulse -= 8192;
			else if(DeltaPulse < - 4096)
				DeltaPulse += 8192;
				
			C610[ESC_Index].Position += DeltaPulse;
			break;		
		
		case RIGHT_FRONT_ID:
		case LEFT_FRONT_ID:
		case LEFT_REAR_ID:
		case RIGHT_REAR_ID:
			
			ESC_ID = ESC_ID - ID_C620_Base;
			ESC_Index = ESC_ID;
		
			PrevAngle = C620[ESC_Index].Angle;
			C620[ESC_Index].Angle		= (uint16_t)(RxMsg -> Data[0] << 8 | RxMsg -> Data[1]);
			C620[ESC_Index].Velocity	= ( int16_t)(RxMsg -> Data[2] << 8 | RxMsg -> Data[3]);
			C620[ESC_Index].Current		= ( int16_t)(RxMsg -> Data[4] << 8 | RxMsg -> Data[5]);
			C620[ESC_Index].Temprature	= (uint8_t)(RxMsg -> Data[6]);
			DeltaPulse = C620[ESC_Index].Angle - PrevAngle;
		
			if(DeltaPulse > 4095)
				DeltaPulse -= 8192;
			else if(DeltaPulse < - 4096)
				DeltaPulse += 8192;
			
			C620[ESC_Index].Position += DeltaPulse;
			break;
	}
}

void C610_GetFeedbackInfo(CanRxMsg* RxMsg)
{
	uint8_t ESC_ID = RxMsg -> StdId - ID_C610_Base;
	
	uint16_t LastPulse;
	int16_t DeltaPulse;
	if(ValueInRange_u(ESC_ID, 0, 3))
	{
		LastPulse = C610[ESC_ID].Angle;
		
		C610[ESC_ID].Angle 	  	= (uint16_t)(RxMsg -> Data[0] << 8 | RxMsg -> Data[1]);
		C610[ESC_ID].Velocity   = (int16_t)(RxMsg -> Data[2] << 8 | RxMsg -> Data[3]);
		C610[ESC_ID].Current	= (int16_t)(RxMsg -> Data[4] << 8 | RxMsg -> Data[5]);
		C610[ESC_ID].Temprature = (uint8_t)(RxMsg -> Data[6]);
		
		DeltaPulse = C610[ESC_ID].Angle - LastPulse;
		
		if(DeltaPulse > 4095)
			DeltaPulse -= 8192;
		else if(DeltaPulse < -4096)
			DeltaPulse += 8192;
		C610[ESC_ID].Position += DeltaPulse;
	}
}

void C620_GetFeedbackInfo(CanRxMsg* RxMsg)
{
	uint8_t ESC_ID = RxMsg -> StdId - ID_C620_Base;
	
	uint16_t LastPulse;
	int16_t DeltaPulse;
	if(ValueInRange_u(ESC_ID, 0, 3))
	{
		LastPulse = C620[ESC_ID].Angle;
		
		C620[ESC_ID].Angle 	  	= (uint16_t)(RxMsg -> Data[0] << 8 | RxMsg -> Data[1]);
		C620[ESC_ID].Velocity	= (int16_t)(RxMsg -> Data[2] << 8 | RxMsg -> Data[3]);
		C620[ESC_ID].Current	= (int16_t)(RxMsg -> Data[4] << 8 | RxMsg -> Data[5]);
		C620[ESC_ID].Temprature = (uint8_t)(RxMsg -> Data[6]);
		
		DeltaPulse = C620[ESC_ID].Angle - LastPulse;
		
		if(DeltaPulse > 4095)
			DeltaPulse -= 8192;
		else if(DeltaPulse < -4096)
			DeltaPulse += 8192;
		C620[ESC_ID].Position += DeltaPulse;
	}
}

void Motor_Init(void)
{
	/* 设置航向电机(C610)工作模式 */
	Motor_SetControlMode(RIGHT_FRONT_TURNING_ID, MotorMode_Position);
	Motor_SetControlMode(LEFT_FRONT_TURNING_ID, MotorMode_Position);
	Motor_SetControlMode(LEFT_REAR_TURNING_ID, MotorMode_Position);
	Motor_SetControlMode(RIGHT_REAR_TURNING_ID, MotorMode_Position);
		
	/* 设置驱动电机(C620)工作模式 */
	Motor_SetControlMode(RIGHT_FRONT_ID, MotorMode_Velocity);
	Motor_SetControlMode(LEFT_FRONT_ID, MotorMode_Velocity);
	Motor_SetControlMode(LEFT_REAR_ID, MotorMode_Velocity);
	Motor_SetControlMode(RIGHT_REAR_ID, MotorMode_Velocity);
	
	/* 设置航向电机(C610)位置环PID参数 */
	Motor_SetPositionPIDRatio(RIGHT_FRONT_TURNING_ID, 0.1f, 0.00f, 0.00f);
	Motor_SetPositionPIDRatio(LEFT_FRONT_TURNING_ID, 0.1f, 0.00f, 0.00f);
	Motor_SetPositionPIDRatio(LEFT_REAR_TURNING_ID, 0.1f, 0.00f, 0.00f);
	Motor_SetPositionPIDRatio(RIGHT_REAR_TURNING_ID, 0.1f, 0.00f, 0.00f);
	
	Motor_SetVelocityPIDRatio(RIGHT_FRONT_TURNING_ID, 5.0f, 0.05f, 0.00f);
	Motor_SetVelocityPIDRatio(LEFT_FRONT_TURNING_ID, 5.0f, 0.05f, 0.00f);
	Motor_SetVelocityPIDRatio(LEFT_REAR_TURNING_ID, 5.0f, 0.05f, 0.00f);
	Motor_SetVelocityPIDRatio(RIGHT_REAR_TURNING_ID, 5.0f, 0.05f, 0.00f);
	
	/* 设置驱动电机(C620)速度环PID参数 */
	Motor_SetVelocityPIDRatio(RIGHT_FRONT_ID, 5.0f, 0.5f, 0.00f);
	Motor_SetVelocityPIDRatio(LEFT_FRONT_ID, 5.0f, 0.5f, 0.00f);
	Motor_SetVelocityPIDRatio(LEFT_REAR_ID, 5.0f, 0.5f, 0.00f);
	Motor_SetVelocityPIDRatio(RIGHT_REAR_ID, 5.0f, 0.5f, 0.00f);
}


void Motor_SetVelocityPIDRatio(uint16_t Motor_ID, float Kp, float Ki, float Kd)
{
	switch(Motor_ID)
	{
		case RIGHT_FRONT_TURNING_ID:
		case LEFT_FRONT_TURNING_ID:
		case LEFT_REAR_TURNING_ID:
		case RIGHT_REAR_TURNING_ID:
			PID_C610_Velocity[Motor_ID - ID_C610_Base].Kp = Kp;
			PID_C610_Velocity[Motor_ID - ID_C610_Base].Ki = Ki;
			PID_C610_Velocity[Motor_ID - ID_C610_Base].Kd = Kd;
			PID_C610_Velocity[Motor_ID - ID_C610_Base].MaxOutput = 6000;
		
			PID_C610_Velocity[Motor_ID - ID_C610_Base].Deadband    = 50.0f;
			PID_C610_Velocity[Motor_ID - ID_C610_Base].MaxIncrease = 600.0f;
			PID_C610_Velocity[Motor_ID - ID_C610_Base].MaxInt 	   = 3000.0f;
		    break;

		case RIGHT_FRONT_ID:
		case LEFT_FRONT_ID:
		case LEFT_REAR_ID:
		case RIGHT_REAR_ID:
			PID_C620_Velocity[Motor_ID - ID_C620_Base].Kp = Kp;
			PID_C620_Velocity[Motor_ID - ID_C620_Base].Ki = Ki;
			PID_C620_Velocity[Motor_ID - ID_C620_Base].Kd = Kd;
			PID_C620_Velocity[Motor_ID - ID_C620_Base].MaxOutput = 10000;
		
			PID_C620_Velocity[Motor_ID - ID_C620_Base].Deadband    = 50.0f;
			PID_C620_Velocity[Motor_ID - ID_C620_Base].MaxIncrease = 600.0f;
			PID_C620_Velocity[Motor_ID - ID_C620_Base].MaxInt 	   = 3000.0f;
		    break;
	}
}

void Motor_SetPositionPIDRatio(uint16_t Motor_ID, float Kp, float Ki, float Kd)
{
	switch(Motor_ID)
	{
		case RIGHT_FRONT_TURNING_ID:
		case LEFT_FRONT_TURNING_ID:
		case LEFT_REAR_TURNING_ID:
		case RIGHT_REAR_TURNING_ID:
			PID_C610_Position[Motor_ID - ID_C610_Base].Kp = Kp;
			PID_C610_Position[Motor_ID - ID_C610_Base].Ki = Ki;
			PID_C610_Position[Motor_ID - ID_C610_Base].Kd = Kd;
			PID_C610_Position[Motor_ID - ID_C610_Base].MaxOutput = 20000.0f;
		
			PID_C610_Velocity[Motor_ID - ID_C610_Base].Deadband  = 1000.0f;
			PID_C610_Velocity[Motor_ID - ID_C610_Base].MaxInt 	 = 5000.0f;
			break;

		case RIGHT_FRONT_ID:
		case LEFT_FRONT_ID:
		case LEFT_REAR_ID:
		case RIGHT_REAR_ID:
			PID_C620_Position[Motor_ID - ID_C620_Base].Kp = Kp;
			PID_C620_Position[Motor_ID - ID_C620_Base].Ki = Ki;
			PID_C620_Position[Motor_ID - ID_C620_Base].Kd = Kd;
			PID_C620_Position[Motor_ID - ID_C620_Base].MaxOutput = 2000.0f;
		
			PID_C620_Velocity[Motor_ID - ID_C620_Base].Deadband  = 50.0f;
			PID_C620_Velocity[Motor_ID - ID_C620_Base].MaxInt 	 = 3000.0f;
			break;
	}
}

void Motor_SetVelocity(uint16_t Motor_ID, int16_t Velocity)
{
	switch(Motor_ID)
	{
		case RIGHT_FRONT_TURNING_ID:
		case LEFT_FRONT_TURNING_ID:
		case LEFT_REAR_TURNING_ID:
		case RIGHT_REAR_TURNING_ID:
			C610[Motor_ID - ID_C610_Base].Velocity_Ref = Velocity;
			break;

		case RIGHT_FRONT_ID:
		case LEFT_FRONT_ID:
		case LEFT_REAR_ID:
		case RIGHT_REAR_ID:
			C620[Motor_ID - ID_C620_Base].Velocity_Ref = Velocity;
			break;
	}
}

void Motor_SetPosition(uint16_t Motor_ID, int32_t Position)
{
	switch(Motor_ID)
	{
		case RIGHT_FRONT_TURNING_ID:
		case LEFT_FRONT_TURNING_ID:
		case LEFT_REAR_TURNING_ID:
		case RIGHT_REAR_TURNING_ID:
			C610[Motor_ID - ID_C610_Base].Pulse_Ref = Position;
			break;

		case RIGHT_FRONT_ID:
		case LEFT_FRONT_ID:
		case LEFT_REAR_ID:
		case RIGHT_REAR_ID:
			C620[Motor_ID - ID_C620_Base].Pulse_Ref = Position;
			break;
	}
}

void Motor_SetControlMode(uint16_t Motor_ID, MotorMode_TypeDef MotorMode)
{
	switch(Motor_ID)
	{
		case RIGHT_FRONT_TURNING_ID:
		case LEFT_FRONT_TURNING_ID:
		case LEFT_REAR_TURNING_ID:
		case RIGHT_REAR_TURNING_ID:
			C610[Motor_ID - ID_C610_Base].Mode = MotorMode;
			break;

		case RIGHT_FRONT_ID:
		case LEFT_FRONT_ID:
		case LEFT_REAR_ID:
		case RIGHT_REAR_ID:
			C620[Motor_ID - ID_C620_Base].Mode = MotorMode;
			break;
	}
}

void VelCrl(uint16_t Motor_ID, int32_t vel)
{
	static uint8_t Index;
	
 	Motor_SetVelocity(Motor_ID, vel);
	
	switch(Motor_ID)
	{                 
		
		case RIGHT_FRONT_ID:
		case LEFT_FRONT_ID:
		case LEFT_REAR_ID:
		case RIGHT_REAR_ID:
				Index = Motor_ID - ID_C620_Base;
				PID_C620_Velocity[Index].Ref = C620[Index].Velocity_Ref;
				PID_C620_Velocity[Index].Feedback = C620[Index].Velocity;
		
				PID_Increment_Calc(&PID_C620_Velocity[Index]);
				C620[Index].Current_Ref = PID_C620_Velocity[Index].Output;
		
			break;

		case RIGHT_FRONT_TURNING_ID:
		case LEFT_FRONT_TURNING_ID:
		case LEFT_REAR_TURNING_ID:
		case RIGHT_REAR_TURNING_ID:
				Index = Motor_ID - ID_C610_Base;
				PID_C610_Velocity[Index].Ref = C610[Index].Velocity_Ref;
				PID_C610_Velocity[Index].Feedback = C610[Index].Velocity;
		
				PID_Increment_Calc(&PID_C610_Velocity[Index]);
				C610[Index].Current_Ref = PID_C620_Velocity[Index].Output;
		
			break;
	}
}

void PosCrl(uint16_t Motor_ID, int32_t pos)
{
	static uint8_t Index;
	
	Motor_SetPosition(Motor_ID, pos);
	
	switch(Motor_ID)
	{
		case RIGHT_FRONT_ID:
		case LEFT_FRONT_ID:
		case LEFT_REAR_ID:
		case RIGHT_REAR_ID:
				Index = Motor_ID - ID_C620_Base;
				PID_C620_Position[Index].Ref = C620[Index].Pulse_Ref;
				PID_C620_Position[Index].Feedback = C620[Index].Position;
				PID_Regular_Cacl(&PID_C620_Position[Index]);

				PID_C620_Velocity[Index].Ref = PID_C620_Position[Index].Output;
				PID_C620_Velocity[Index].Feedback = C620[Index].Velocity;
				PID_Increment_Calc(&PID_C620_Velocity[Index]);
				C620[Index].Current_Ref = PID_C620_Velocity[Index].Output;
		
			break;

		case RIGHT_FRONT_TURNING_ID:
		case LEFT_FRONT_TURNING_ID:
		case LEFT_REAR_TURNING_ID:
		case RIGHT_REAR_TURNING_ID:
				Index = Motor_ID - ID_C610_Base;
				PID_C610_Position[Index].Ref = C610[Index].Pulse_Ref;
				PID_C610_Position[Index].Feedback = C610[Index].Position;
				PID_Regular_Cacl(&PID_C610_Position[Index]);
		
				PID_C610_Velocity[Index].Ref = PID_C610_Position[Index].Output;
				PID_C610_Velocity[Index].Feedback = C610[Index].Velocity;
				PID_Increment_Calc(&PID_C610_Velocity[Index]);
				C610[Index].Current_Ref = PID_C610_Velocity[Index].Output;
		
			break;
	}
}

uint8_t SetJoggingVel(CAN_TypeDef* CANx, uint16_t Motor_CTRL, Motor_t * Motor_Array)
{
	uint8_t mbox = 0;
	
	Motor_t *pMotor;
	int32_t SendData[4] = {0};
	
	int i;
	
	pMotor = Motor_Array;
	
	const int16_t s_max_current_val = (int32_t) 16384 * 5000 / 20000;		//2000 to 5000
	
	for(i = 0; i < 4; i++,pMotor++)
	{
		SendData[i] = pMotor->Current_Ref;
		if(SendData[i] >s_max_current_val )
		{
			SendData[i] = s_max_current_val;
		}
		else if(SendData[i] < -s_max_current_val)
		{
			SendData[i] = -s_max_current_val;
		}
	}
	
	static CanTxMsg TxMessage;
	switch((uint32_t)CANx)
	{
		case CAN1_BASE:
				TxMessage.StdId = Motor_CTRL;
				TxMessage.DLC	= 0x08;
				TxMessage.IDE	= CAN_Id_Standard;
				TxMessage.RTR	= CAN_RTR_Data;
				TxMessage.Data[0] = SendData[0] >> 8;
				TxMessage.Data[1] = SendData[0];
				TxMessage.Data[2] = SendData[1] >> 8;
				TxMessage.Data[3] = SendData[1];
				TxMessage.Data[4] = SendData[2] >> 8;
				TxMessage.Data[5] = SendData[2];
				TxMessage.Data[6] = SendData[3] >> 8;
				TxMessage.Data[7] = SendData[3];
			break;
		case CAN2_BASE:
				TxMessage.StdId = Motor_CTRL;
				TxMessage.DLC	= 0x08;
				TxMessage.IDE	= CAN_Id_Standard;
				TxMessage.RTR	= CAN_RTR_Data;
				TxMessage.Data[0] = SendData[0] >> 8;
				TxMessage.Data[1] = SendData[0];
				TxMessage.Data[2] = SendData[1] >> 8;
				TxMessage.Data[3] = SendData[1];
				TxMessage.Data[4] = SendData[2] >> 8;
				TxMessage.Data[5] = SendData[2];
				TxMessage.Data[6] = SendData[3] >> 8;
				TxMessage.Data[7] = SendData[3];
			break;
	}
	
	mbox = CAN_Transmit(CANx, &TxMessage);
	i = 0;
	
	if(mbox == CAN_TxStatus_NoMailBox)
		return CAN_TxStatus_Failed;
	return CAN_TxStatus_Ok;
	
	//CAN_TxMsg(CANx, TxMessage.StdId, TxMessage.Data);
}


uint8_t SendPosCmd(CAN_TypeDef* CANx, uint16_t Motor_CTRL, Motor_t * Motor_Array)
{
	uint8_t mbox = 0;
	int32_t SendData[4] = {0};
	int16_t i;
	
	const int16_t s_max_current_val = (int32_t) 16384 * 5000 / 20000;		
	
	for(i = 0; i < 4; i++)
	{
		SendData[i] = (Motor_Array + i)->Current_Ref;
		if(SendData[i] >s_max_current_val )
		{
			SendData[i] = s_max_current_val;
		}
		else if(SendData[i] < -s_max_current_val)
		{
			SendData[i] = -s_max_current_val;
		}
	}
	
	
	static CanTxMsg TxMessage;
	
	switch((uint32_t)CANx)
	{
		case CAN1_BASE:
				TxMessage.StdId = Motor_CTRL;
				TxMessage.DLC	= 0x08;
				TxMessage.IDE	= CAN_Id_Standard;
				TxMessage.RTR	= CAN_RTR_Data;
				TxMessage.Data[0] = SendData[0] >> 8;
				TxMessage.Data[1] = SendData[0];
				TxMessage.Data[2] = SendData[1] >> 8;
				TxMessage.Data[3] = SendData[1];
				TxMessage.Data[4] = SendData[2] >> 8;
				TxMessage.Data[5] = SendData[2];
				TxMessage.Data[6] = SendData[3] >> 8;
				TxMessage.Data[7] = SendData[3];
			break;
		case CAN2_BASE:
				TxMessage.StdId = Motor_CTRL;
				TxMessage.DLC	= 0x08;
				TxMessage.IDE	= CAN_Id_Standard;
				TxMessage.RTR	= CAN_RTR_Data;
				TxMessage.Data[0] = SendData[0] >> 8;
				TxMessage.Data[1] = SendData[0];
				TxMessage.Data[2] = SendData[1] >> 8;
				TxMessage.Data[3] = SendData[1];
				TxMessage.Data[4] = SendData[2] >> 8;
				TxMessage.Data[5] = SendData[2];
				TxMessage.Data[6] = SendData[3] >> 8;
				TxMessage.Data[7] = SendData[3];
			break;
	}
	
	mbox = CAN_Transmit(CANx, &TxMessage);
	i = 0;
	
	if(mbox == CAN_TxStatus_NoMailBox)
		return CAN_TxStatus_Failed;
	return CAN_TxStatus_Ok;
	//CAN_TxMsg(CANx, TxMessage.StdId, TxMessage.Data);
}

void Task_DisableControl(void)
{
	DisableCrl(RIGHT_FRONT_ID);
	DisableCrl(LEFT_FRONT_ID);
	DisableCrl(RIGHT_REAR_ID);
	DisableCrl(LEFT_REAR_ID);
	
	DisableCrl(RIGHT_FRONT_TURNING_ID);
	DisableCrl(LEFT_FRONT_TURNING_ID);
	DisableCrl(RIGHT_REAR_TURNING_ID);
	DisableCrl(LEFT_REAR_TURNING_ID);
	
	SendPosCmd(CAN2, ID_C610_CTRL, C610);
	SetJoggingVel(CAN1, ID_C620_CTRL, C620);
}

void DisableCrl(uint16_t Motor_ID)
{
	static uint8_t Index;
	
	Motor_SetVelocity(Motor_ID, 0.0f);
	
	switch(Motor_ID)
	{
		case RIGHT_FRONT_ID:
		case LEFT_FRONT_ID:
		case LEFT_REAR_ID:
		case RIGHT_REAR_ID:
				Index = Motor_ID - ID_C620_Base;
				C620[Index].Current_Ref = 0.0f;
			break;

		case RIGHT_FRONT_TURNING_ID:
		case LEFT_FRONT_TURNING_ID:
		case LEFT_REAR_TURNING_ID:
		case RIGHT_REAR_TURNING_ID:
				Index = Motor_ID - ID_C610_Base;
				C610[Index].Current_Ref = 0.0f;
			break;
	}
}
