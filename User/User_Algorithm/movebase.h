#ifndef _MOVEBASE_H
#define _MOVEBASE_H

#include "stm32f4xx.h"

/* ���ֵ��̻������ٶȽṹ�� */
//���������ٶ�
typedef struct
{
	float vel;			//�ٶȴ�С
	float direction;	//�ٶȷ���
	float omega;		//���ٶȴ�С
}CasterVel_t;

/* ��������״̬�ṹ�� */
typedef struct
{
	float vel;			//��ϵ�ٶȴ�С��������ת�٣�
	float direction;	//��ϵ�ٶȷ��򣨺�����ƫ���Ƕȣ�
}wheelVel_t;


/* �ĸ���ϵ״̬�ṹ�� */
typedef struct
{
	wheelVel_t rightFront;	//��ǰ��
	wheelVel_t leftFront;	//��ǰ��
	wheelVel_t leftRear;	//�����
	wheelVel_t rightRear;	//�Һ���
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

/* �ĸ���ϵ״̬�ṹ�� */
/*�Ƿ�����滻Ϊ
typedef struct
{
	wheel_t Target;
	wheel_t Act;
	uint8_t disenableFlag; 	//����ʧ�ܱ�־λ
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
	
	uint8_t disenableFlag; 	//����ʧ�ܱ�־λ
}wheelState_t;

//�����תһ�ܵĻ�еת�ӽǶ�
#define COUNTS_PER_ROUND (8191)
#define SECOUND_PER_MIN  (60)

/* ���̻������� */
#define WHEEL_DIAMETER  (140.0f)		//����ֱ������λ��mm��
#define DISX_OPS2CENTER (0.0f)			//��λϵͳX�᷽�����ľ���
#define DISY_OPS2CENTER (-253.6f)		//��λϵͳY�᷽�����ľ���
#define MOVEBASE_RADIUS (430.63f)	//������ת�뾶								

/* �Ƕ��ƻ����ƻ��� */
#define ANGLE2RAD(x) (x / 180.0f * PI)			//�Ƕ���ת��Ϊ������
#define RAD2ANGLE(x) (x /PI * 180.0f)			//������ת��Ϊ�Ƕ���

/* ����ƽ������ϵ���������� */
#define RIGHT_FRONT_TURNING_NUM 	(1)				//��ǰ��ת��ID��  	ID_C610_01	0x201
#define LEFT_FRONT_TURNING_NUM 		(2)				//��ǰ��ת��ID��	ID_C610_02	0x202
#define LEFT_REAR_TURNING_NUM		(3)				//�����ת��ID��	ID_C610_03	0x203
#define RIGHT_REAR_TURNING_NUM 		(4)				//�Һ���ת��ID��	ID_C610_04	0x204

#define RIGHT_FRONT_NUM 			(5)				//��ǰ��ID��		ID_C620_01	0x205
#define LEFT_FRONT_NUM 				(6)				//��ǰ��ID��		ID_C620_02	0x206
#define LEFT_REAR_NUM				  (7)				//�����ID��		ID_C620_03	0x207
#define RIGHT_REAR_NUM 				(8)				//�Һ���ID��		ID_C620_04	0x208

/* ����λ�� 
 * ��λ������Y��Ϊ��ʼ�ᣬ��ʱ��Ϊ�� 	    //��ʼ�᷽��������޸ĺ����޸���Ӧ�Ƕ�
 */
#define LEFT_FRONT_VERTICAL_ANG  (-135.0f)	//��ǰ���������������߷���
#define RIGHT_FRONT_VERTICAL_ANG (135.0f)	//��ǰ���������������߷���
#define LEFT_REAR_VERTICAL_ANG   (-45.0f)	//������������������߷���
#define RIGHT_REAR_VERTICAL_ANG  (45.0f)	//�Һ����������������߷���

/* ���ٱ� */
#define M3508_REDUCTION_RATIO	(3591.f/187.0f)											//M3508���ٱ�
#define M2006_REDUCTION_RATIO 	(36.0f/1.0f)											//M2006���ٱ�
#define TURNING_REDUCTION_RATIO (78.0f/17.0f)											//ת����ּ��ٱ�
#define WHEEL_TURNING_REDUCTION_RATIO (M2006_REDUCTION_RATIO * TURNING_REDUCTION_RATIO)	//�����ּ��ٱ�


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
