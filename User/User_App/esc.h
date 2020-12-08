#ifndef _MOTOR_H
#define _MOTOR_H

#include "stm32f4xx.h"

#include "bsp_can.h"

#include "pid.h"


/* 电机控制参数 */
typedef struct
{
	/* 实时数据 */
	uint16_t Angle;
	int16_t  Velocity;
	int16_t  Current;
	uint8_t  Temprature;
	int32_t  Position;
	
	/* 模式选择 */
	uint8_t  Mode;
	
	/* PID输入参数 */
	int16_t Current_Ref;
	int16_t Velocity_Ref;
	int32_t Pulse_Ref;
}Motor_t;

/* 电机工作模式 */
typedef enum
{
	MotorMode_Disable  = (uint8_t)0x00,		//失能模式(卸力状态) 
	MotorMode_Current  = (uint8_t)0x01,		//电流模式
	MotorMode_Velocity = (uint8_t)0x02,		//速度模式
	MotorMode_Position = (uint8_t)0x03,		//位置模式
}MotorMode_TypeDef;

//From "esc.c"
extern Motor_t C620[4],C610[4];

extern PID_Regular_t	PID_C610_Position[4], PID_C620_Position[4];
extern PID_Increment_t 	PID_C610_Velocity[4], PID_C620_Velocity[4];
extern float leftFrontAng, rightFrontAng, leftRearAng, rightRearAng;

/* C610接收地址(航向轮) */
#define ID_C610_CTRL			0x200
#define ID_C610_Base			0x201
#define RIGHT_FRONT_TURNING_ID	0x201		//ID_C610_01
#define LEFT_FRONT_TURNING_ID	0x202		//ID_C610_02
#define LEFT_REAR_TURNING_ID	0x203		//ID_C610_03
#define RIGHT_REAR_TURNING_ID	0x204		//ID_C610_04
//#define CAN_C610					CAN2

/* C620接收地址(驱动轮) */
#define ID_C620_CTRL		0x1FF		
#define ID_C620_Base		0x205
#define RIGHT_FRONT_ID		0x205		//ID_C620_01
#define LEFT_FRONT_ID		0x206		//ID_C620_02
#define LEFT_REAR_ID		0x207		//ID_C620_03
#define RIGHT_REAR_ID		0x208		//ID_C620_04
//#define CAN_C620			CAN1

void Task_VelControl(void);		//The task which is running recyclely
void Task_DisableControl(void);
void VelControl(void);

//void Motor_GetFeedbackInfo(CanRxMsg* RxMsg);
void C610_GetFeedbackInfo(CanRxMsg* RxMsg);
void C620_GetFeedbackInfo(CanRxMsg* RxMsg);

void Motor_Init(void);		//电机控制相关参数初始化函数

//用于修改PID控制相关参数
void Motor_SetVelocityPIDRatio(uint16_t Motor_ID, float Kp, float Ki, float Kd);
void Motor_SetPositionPIDRatio(uint16_t Motor_ID, float Kp, float Ki, float Kd);

//用于修改Ref，PID控制值
void Motor_SetVelocity(uint16_t Motor_ID, int16_t Velocity);
void Motor_SetPosition(uint16_t Motor_ID, int32_t Position);
void Motor_SetControlMode(uint16_t Motor_ID, MotorMode_TypeDef MotorMode);	//用不上

//用于PID计算并记录结果
void VelCrl(uint16_t Motor_ID, int32_t vel);
void PosCrl(uint16_t Motor_ID, int32_t pos);

void DisableCrl(uint16_t Motor_ID);

void Task_RemoteControl(void);
//用CAN将计算结果发送
uint8_t SetJoggingVel(CAN_TypeDef* CANx, uint16_t Motor_CTRL, Motor_t* Motor_Array);
uint8_t SendPosCmd(CAN_TypeDef* CANx, uint16_t Motor_CTRL, Motor_t* Motor_Array);

#endif
