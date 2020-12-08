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
	Pose_t actPos;		//��ǰ����λ�õ�
	Pose_t nextPos;		//��һ��Ҫ�ܵ���λ�õ�
	
	/* ��λϵͳ��ȡ��Ϣ */
	float tempX;				//��ǰ����Xֵ
	float tempY;				//��ǰ����Yֵ
	float tempAngle;			//��ǰ��̬��
	float tempSpeedX;			//��ǰX�᷽�����ٶ�
	float tempSpeedY;			//��ǰY�᷽�����ٶ�
	float tempWZ;				//��ǰ���ٶ�
	
	float posAngle;				//������̬��
	float giveOmega;			//Ԥ����ٶ�
	float maxVel;				//Ԥ������ٶ�
	
	float outputVel;			//�ٶȻ�������ٶȴ�С
	float outputDirection;		//�ٶȻ�������ٶȷ���
	
	/* �˶�ѧ�����ٶ������Ϣ */
	CasterVel_t	CasterVel;		//���ֵ����ٶ���Ϣ �ٶȡ����򡢽��ٶ�  ����ûɶ��
	float casterVel;			//�����ٶȴ�С	
	float casterVelDir;			//����ƽ���ٶȷ���
//	float casterOmega;			//���ٶȴ�С����/�룩
			
	/* �������ٶ���Ϣ */
	wheel_t wheelVel;			//�ֽ⵽�������ӵ��ٶȴ�С�ͷ���
	
	float posAngleShift;		//����ʱ��̬��ƫ����
	float gasPressure;			//����ʱ��ѹ	

	//������жϵ����յ���������

}debugInfo_t;

typedef enum
{
	waitForStart,
	go2Kick1stBall,
	Kick1stBall,

	//wait for adding other status
}kickStatus_t;

//��������ṹ��
typedef struct
{
	//�����־λ
	uint8_t kickFlag;
	//����׼����־
	uint8_t ready;
	//��ѹֵ
	float gasPressure;
	
	uint8_t ballNum;
}kicker_t;

typedef struct
{

	wheelState_t 		wheelState;				//����״̬��Ϣ
	debugInfo_t 		debugInformation;	//��������
	kickStatus_t 	  kickStatus;		  	//����״̬
	kicker_t				kicker;						//������Ϣ
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
