#ifndef _MOVEBASE_H
#define _MOVEBASE_H

#include "stm32f4xx.h"

/* 舵轮底盘机器人速度结构体 */
//描述车体速度
typedef struct
{
	float vel;			//速度大小
	float direction;	//速度方向
	float omega;		//角速度大小
}CasterVel_t;

/* 单个车轮状态结构体 */
typedef struct
{
	float vel;			//轮系速度大小（驱动轮转速）
	float direction;	//轮系速度方向（航向轮偏航角度）
}wheelVel_t;


/* 四个轮系状态结构体 */
typedef struct
{
	wheelVel_t rightFront;	//右前轮
	wheelVel_t leftFront;	//左前轮
	wheelVel_t leftRear;	//左后轮
	wheelVel_t rightRear;	//右后轮
}wheel_t;


typedef struct
{
	int32_t lf;
	int32_t rf;
	int32_t lr;
	int32_t rr;
}steerLoopShift_t;

typedef struct
{
	float lf;
	float rf;
	float lr;
	float rr;
}ResetAngle_t;

/* 四个轮系状态结构体 */
/*是否可以替换为
typedef struct
{
	wheel_t Target;
	wheel_t Act;
	uint8_t disenableFlag; 	//轮子失能标志位
}wheelState_t;
*/
typedef struct
{
	wheelVel_t	rightFrontTarget;
	wheelVel_t	rightFrontAct;
	
	wheelVel_t	leftFrontTarget;
	wheelVel_t	leftFrontAct;
	
	wheelVel_t	leftRearTarget;
	wheelVel_t	leftRearAct;
	
	wheelVel_t	rightRearTarget;
	wheelVel_t	rightRearAct;
	
	steerLoopShift_t steerLoopShift;
	
	uint8_t disenableFlag; 	//轮子失能标志位
}wheelState_t;

//电机旋转一周的机械转子角度
#define COUNTS_PER_ROUND (8191)
#define SECOUND_PER_MIN  (60)

/* 底盘基本参数 */
#define WHEEL_DIAMETER  (140.0f)		//轮子直径（单位：mm）
#define DISX_OPS2CENTER (0.0f)			//定位系统X轴方向到中心距离
#define DISY_OPS2CENTER (-253.6f)		//定位系统Y轴方向到中心距离
#define MOVEBASE_RADIUS (430.63f)	//底盘旋转半径								

/* 角度制弧度制互换 */
#define ANGLE2RAD(x) (x / 180.0f * PI)			//角度制转化为弧度制
#define RAD2ANGLE(x) (x /PI * 180.0f)			//弧度制转换为角度制

/* 按照平面坐标系象限数排序 */
#define RIGHT_FRONT_TURNING_NUM 	(1)				//右前轮转向ID号  	ID_C610_01	0x201
#define LEFT_FRONT_TURNING_NUM 		(2)				//左前轮转向ID号	ID_C610_02	0x202
#define LEFT_REAR_TURNING_NUM		(3)				//左后轮转向ID号	ID_C610_03	0x203
#define RIGHT_REAR_TURNING_NUM 		(4)				//右后轮转向ID号	ID_C610_04	0x204

#define RIGHT_FRONT_NUM 			(5)				//右前轮ID号		ID_C620_01	0x205
#define LEFT_FRONT_NUM 				(6)				//左前轮ID号		ID_C620_02	0x206
#define LEFT_REAR_NUM				  (7)				//左后轮ID号		ID_C620_03	0x207
#define RIGHT_REAR_NUM 				(8)				//右后轮ID号		ID_C620_04	0x208

/* 车轮位置 
 * 定位器所在Y轴为起始轴，逆时针为正 	    //起始轴方向待定，修改后需修改相应角度
 */
#define LEFT_FRONT_VERTICAL_ANG  (-135.0f)	//左前轮与中心连线切线方向
#define RIGHT_FRONT_VERTICAL_ANG (135.0f)	//右前轮与中心连线切线方向
#define LEFT_REAR_VERTICAL_ANG   (-45.0f)	//左后轮与中心连线切线方向
#define RIGHT_REAR_VERTICAL_ANG  (45.0f)	//右后轮与中心连线切线方向

/* 减速比 */
#define M3508_REDUCTION_RATIO	(3591.f/187.0f)											//M3508减速比
#define M2006_REDUCTION_RATIO 	(36.0f/1.0f)											//M2006减速比
#define TURNING_REDUCTION_RATIO (78.0f/17.0f)											//转向齿轮减速比
#define WHEEL_TURNING_REDUCTION_RATIO (M2006_REDUCTION_RATIO * TURNING_REDUCTION_RATIO)	//航向轮减速比


void OutputVel2Wheel(float vel, float direction, float omega);
wheelVel_t CalcWheelSpeed(float vel , float direction , float omega , float angleN, float postureAngle);
void OrientationClear(void);

void WheelVelControl(wheel_t wheelVel);
void Transform2CasterCoordinate(wheel_t * wheelVel);
void Transform2WheelCoordinate(wheel_t * wheelVel);
void SendCmd2Driver(float lfVel , float lfDir , float rfVel , float rfDir,
					float lrVel , float lrDir , float rrVel , float rrDir);


int WheelAngle2PositionTransform(float angle , int32_t loopShift);
int Vel2RotateVel(float vel);
float WheelAngle2PositionInverseTransform(int position , int32_t loopShift);
float RotateVel2Vel(int pulse);

void CalcSteerLoopShift(void);
void CalcWheelActAngle(void);

void AngleLimit(float *angle);
float ReturnLimitAngle(float angle);
void JudgeVelDirection(wheelVel_t *targetVel , float actualAngle);
float TurnInferiorArc(float targetAngle , float actualAngle);

void OutputZeroVel2Wheel(void);
#endif
