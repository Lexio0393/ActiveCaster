#ifndef _ROBOT_H
#define _ROBOT_H

//#include "bsp_main.h"
#include "movebase.h"
#include "motion.h"

#include "pps.h"
#include "esc.h"
#include "manul.h"


typedef struct
{
	Pose_t actPos;		//当前所在位置点
	Pose_t nextPos;		//下一个要跑到的位置点
	
	/* 定位系统获取信息 */
	float tempX;				//当前坐标X值
	float tempY;				//当前坐标Y值
	float tempAngle;			//当前姿态角
	float tempSpeedX;			//当前X轴方向上速度
	float tempSpeedY;			//当前Y轴方向上速度
	float tempWZ;				//当前角速度
	
	float posAngle;				//底盘姿态角
	float giveOmega;			//预设角速度
	float maxVel;				//预设最大速度
	
	float outputVel;			//速度环输出的速度大小
	float outputDirection;		//速度环输出的速度方向
	
	/* 运动学解算速度相关信息 */
	CasterVel_t	CasterVel;		//舵轮底盘速度信息 速度、方向、角速度  好像没啥用
	float casterVel;			//底盘速度大小	
	float casterVelDir;			//底盘平移速度方向
//	float casterOmega;			//角速度大小（度/秒）
			
	/* 各车轮速度信息 */
	wheel_t wheelVel;			//分解到各个轮子的速度大小和方向
	
	float posAngleShift;		//踢球时姿态角偏移量
	float gasPressure;			//踢球时气压	

	//待添加判断到达终点死区距离

}debugInfo_t;

typedef enum
{
	waitForStart,
	go2Kick1stBall,
	Kick1stBall,

	//wait for adding other status
}kickStatus_t;

//踢球机构结构体
typedef struct
{
	//踢球标志位
	uint8_t kickFlag;
	//踢球准备标志
	uint8_t ready;
	//气压值
	float gasPressure;
	
	uint8_t ballNum;
}kicker_t;

typedef struct
{

	wheelState_t 		wheelState;				//车轮状态信息
	debugInfo_t 		debugInformation;	//调试数据
	kickStatus_t 	  kickStatus;		  	//踢球状态
	kicker_t				kicker;						//踢球信息
}gRobot_t;

extern gRobot_t gRobot;

typedef enum
{
    CasterStatus_LostForce  = (uint8_t)0x00,
    CasterStatus_Reset	    = (uint8_t)0x01,
    CasterStatus_AutoMode   = (uint8_t)0x02,
    CasterStatus_Remote     = (uint8_t)0x03,
}CasterStatus_t;


void Task_GetRunningStatus(void);
void Task_JudgeBreaking(void);
void Task_GetCasterStatus(void);

void RobotInit(void);
#define SQRT_2	1.414213562373f

#endif
