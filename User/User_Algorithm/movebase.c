#include "movebase.h"

#include "algorithm.h"
#include "motion.h"

#include "pps.h"
#include "esc.h"
#include "robot.h"
#include "task.h"

//������motion.h ������task.c
extern vector_t finalVel;
extern gRobot_t gRobot;
/*
 *OutputVel2Wheel ����������ϵ���ٶȿ���ʸ������������ϵ�ٵ���ϵ����ϵ��ת��
 *
 *���ȵ���CalcWheelSpeed����������ת��Ϊ���۸���ϵ���ٶ�finalVel�������еĿ���ֵ��
 *	��������������ʱ����outputVel�У���ouputVel��ֵ��ȫ�ֱ���gRobot.debugInformation.wheelVel��¼�����ӿ���ֵ
 *
 *��ͨ��WheelVelControl�����������ó�����ֵת��Ϊ��ϵʵ�ʿ���ֵ
 *	����gRobot.debugInformation.wheelVel��¼�ĸ����ӿ���ֵת��Ϊ��ϵ����ϵ��ʵ�ʿ���ֵ
 *		������ֵ��ͬ���ڣ���ϵ����ת������ı䡢����ƫת�Ƕ��нǶȻ����ϵ
 *			WheelVelControl����ĽǶȴ����ں�������ʱ����outputVel�У�����ֵ���ⲿ��̬����Ang
 *	 			ͨ���ⲿ��̬������¼ʵ�ʿ���ֵ���������ʱȫ�ֱ���gRobot.debugInformation.wheelState�ᱻ��ֵ����¼�ÿ���ֵ
 *
 *����ʵ�ʿ���ֵ�����͸�Vel��PosControl��������PID����
 *���յķ��ͽ���esc.c�е�VelControl���� �ú��������¼finalvel�Ŀ���ֵ������can�ķ���
 *�ֶ�ģʽ�£�ң�����޸ĵ�ֵΪfinalVel
 */
void OutputVel2Wheel(float vel, float direction, float omega)
{
//	finalVel.module = vel;
//	finalVel.direction = direction;
// �ⲿ�Ѿ���¼��ҡ��ֵ��
	
	wheel_t outputVel = {0.0f};
	
	outputVel.leftFront  = CalcWheelSpeed(vel, direction, omega, LEFT_FRONT_VERTICAL_ANG, GetAngle());
	outputVel.rightFront = CalcWheelSpeed(vel, direction, omega, RIGHT_FRONT_VERTICAL_ANG, GetAngle());
	outputVel.leftRear   = CalcWheelSpeed(vel, direction, omega, LEFT_REAR_VERTICAL_ANG, GetAngle());
	outputVel.rightRear  = CalcWheelSpeed(vel, direction, omega, RIGHT_REAR_VERTICAL_ANG, GetAngle());
	
	gRobot.debugInformation.wheelVel = outputVel;		//��¼�����ٶ�
	
	WheelVelControl(outputVel);
}

wheelVel_t CalcWheelSpeed(float vel , float direction , float omega , float angleN, float postureAngle)
{
	wheelVel_t sumVel = {0.0f};
	float velX, velY = 0.0f;
	float velN, velNDirection = 0.0f;
	float sumVelX, sumVelY = 0.0f;
	
	velX = vel * arm_cos_f32(ANGLE2RAD(direction));
	velY = vel * arm_sin_f32(ANGLE2RAD(direction));

	velN = ANGLE2RAD(omega) * MOVEBASE_RADIUS;
	velNDirection = angleN + postureAngle;
	AngleLimit(&velNDirection);
	
	sumVelX = velX + velN * arm_cos_f32(ANGLE2RAD(velNDirection));
	sumVelY = velY + velN * arm_sin_f32(ANGLE2RAD(velNDirection));
	
	arm_sqrt_f32(sumVelX * sumVelX + sumVelY * sumVelY, &sumVel.vel);

	//����ϳ��ٶȷ���ʱ��0������������
	if(sumVel.vel>0.01f)
	{
		sumVel.direction = RAD2ANGLE(atan2f(sumVelY, sumVelX));
	}
	else
	{
		sumVel.direction = direction;
	}
	
	return sumVel;
}


/* ��¼�������ӳ�������������ж��Ƿ���Ҫ���� */
static float leftFrontAng = 0.0f, rightFrontAng = 0.0f, leftRearAng = 0.0f, rightRearAng = 0.0f;

/* �������㺯�� */
void OrientationClear(void)
{
	leftFrontAng  = 0.0f;
	rightFrontAng = 0.0f;
	leftRearAng   = 0.0f;
	rightRearAng  = 0.0f;
	
	gRobot.wheelState.steerLoopShift.rf = 0;
	gRobot.wheelState.steerLoopShift.lf = 0;
	gRobot.wheelState.steerLoopShift.lr = 0;
  gRobot.wheelState.steerLoopShift.rr = 0;
	
	gRobot.wheelState.rightFrontAct.direction = 0;
	gRobot.wheelState.leftFrontAct.direction  = 0;
	gRobot.wheelState.leftRearAct.direction   = 0;
	gRobot.wheelState.rightRearAct.direction  = 0;
}

void WheelVelControl(wheel_t wheelVel)
{
	Transform2CasterCoordinate(&wheelVel);		//����������ϵ�½Ƕ�ת��Ϊ����������ϵ��ƫ�ƽǶ�
	Transform2WheelCoordinate(&wheelVel);		  //������������ϵ��ƫ�ƽǶ�ת��Ϊ����ϵ��ƫ�ƽǶ�
	
	//�ж��Ƿ���Ҫ���ٶȷ���
	JudgeVelDirection(&wheelVel.leftFront, leftFrontAng);
	JudgeVelDirection(&wheelVel.rightFront, rightFrontAng);
	JudgeVelDirection(&wheelVel.leftRear, leftRearAng);
	JudgeVelDirection(&wheelVel.rightRear, rightRearAng);

	//��֤��תΪ�ӻ�
	leftFrontAng  = TurnInferiorArc(wheelVel.leftFront.direction, leftFrontAng);
	rightFrontAng = TurnInferiorArc(wheelVel.rightFront.direction, rightFrontAng);
	leftRearAng   = TurnInferiorArc(wheelVel.leftRear.direction, leftRearAng);
	rightRearAng  = TurnInferiorArc(wheelVel.rightRear.direction, rightRearAng);
	
	//���������õĿ���ֵ������PID����
	SendCmd2Driver(wheelVel.leftFront.vel, leftFrontAng, wheelVel.rightFront.vel, rightFrontAng,
				   wheelVel.leftRear.vel, leftRearAng, wheelVel.rightRear.vel, rightRearAng);	
}

void OutputZeroVel2Wheel(void)
{
	wheel_t outputVel = {0.0f};
	gRobot.debugInformation.wheelVel = outputVel;
	
	ResetAngle_t ResetAngle = {0.0f};
	
	ResetAngle.lf = FlexibelValue(ResetAngle.lf, gRobot.wheelState.rightFrontAct.direction, 30.0f);
	ResetAngle.rf = FlexibelValue(ResetAngle.rf, gRobot.wheelState.leftFrontAct.direction , 30.0f);
	ResetAngle.lr = FlexibelValue(ResetAngle.lr, gRobot.wheelState.leftRearAct.direction  , 30.0f);
	ResetAngle.rr = FlexibelValue(ResetAngle.rr, gRobot.wheelState.rightRearAct.direction , 30.0f);
	
	SendCmd2Driver(0, ResetAngle.lf, 0, ResetAngle.rf, 0, ResetAngle.lr, 0, ResetAngle.rr);	
}

void SendCmd2Driver(float lfVel , float lfDir , float rfVel , float rfDir,
					float lrVel , float lrDir , float rrVel , float rrDir)
{
	//��¼��������ʵ�ʸ����Ŀ�����
	gRobot.wheelState.leftFrontTarget.vel = lfVel;
	gRobot.wheelState.leftFrontTarget.direction = lfDir;
	
	//��������ϵ�ʶԳư�װʱ��������ת�������෴
	gRobot.wheelState.rightFrontTarget.vel = -rfVel;
	gRobot.wheelState.rightFrontTarget.direction = rfDir;
	
	gRobot.wheelState.leftRearTarget.vel = lrVel;
	gRobot.wheelState.leftRearTarget.direction = lrDir;
	
	gRobot.wheelState.rightRearTarget.vel = -rrVel;
	gRobot.wheelState.rightRearTarget.direction = rrDir;
	

//	VelCrl(RIGHT_FRONT_ID, -Vel2RotateVel(rfVel));								//Caster.wheelState.rightFrontTarget.vel
//	PosCrl(RIGHT_FRONT_TURNING_ID, WheelAngle2PositionTransform(rfDir, 0));		//Caster.wheelState.rightFrontTarget.direction
//	
//	VelCrl(LEFT_FRONT_ID, Vel2RotateVel(lfVel));								//Caster.wheelState.leftFrontTarget.vel
//	PosCrl(LEFT_FRONT_TURNING_ID, WheelAngle2PositionTransform(lfDir, 0));		//Caster.wheelState.leftFrontTarget.direction

//	VelCrl(LEFT_REAR_ID, Vel2RotateVel(lrVel));									//Caster.wheelState.leftRearTarget.vel
//	PosCrl(LEFT_REAR_TURNING_ID, WheelAngle2PositionTransform(lrDir, 0));		//Caster.wheelState.leftRearTarget.direction
//	
//	VelCrl(RIGHT_REAR_ID, -Vel2RotateVel(rrVel));								//Caster.wheelState.rightRearTarget.vel
//	PosCrl(RIGHT_REAR_TURNING_ID, WheelAngle2PositionTransform(rrDir, 0));		//Caster.wheelState.rightRearTarget.direction

	VelCrl(RIGHT_FRONT_ID, -Vel2RotateVel(rfVel));								//Caster.wheelState.rightFrontTarget.vel
	PosCrl(RIGHT_FRONT_TURNING_ID, WheelAngle2PositionTransform(rfDir, gRobot.wheelState.steerLoopShift.rf));		//Caster.wheelState.rightFrontTarget.direction
	
	VelCrl(LEFT_FRONT_ID, Vel2RotateVel(lfVel));								//Caster.wheelState.leftFrontTarget.vel
	PosCrl(LEFT_FRONT_TURNING_ID, WheelAngle2PositionTransform(lfDir, gRobot.wheelState.steerLoopShift.lf));		//Caster.wheelState.leftFrontTarget.direction

	VelCrl(LEFT_REAR_ID, Vel2RotateVel(lrVel));									//Caster.wheelState.leftRearTarget.vel
	PosCrl(LEFT_REAR_TURNING_ID, WheelAngle2PositionTransform(lrDir, gRobot.wheelState.steerLoopShift.lr));		//Caster.wheelState.leftRearTarget.direction
	
	VelCrl(RIGHT_REAR_ID, -Vel2RotateVel(rrVel));								//Caster.wheelState.rightRearTarget.vel
	PosCrl(RIGHT_REAR_TURNING_ID, WheelAngle2PositionTransform(rrDir, gRobot.wheelState.steerLoopShift.rr));		//Caster.wheelState.rightRearTarget.direction
}

void Transform2CasterCoordinate(wheel_t * wheelVel)
{
	//����λϵͳ����ϵ�½Ƕ�ת��Ϊ����������ϵ�½Ƕ�
	wheelVel->leftFront.direction	-= GetAngle();
	wheelVel->rightFront.direction 	-= GetAngle();
	wheelVel->leftRear.direction	-= GetAngle();
	wheelVel->rightRear.direction	-= GetAngle();

	//���Ƕ�������180�ȵ�-180�ȷ�Χ��
	AngleLimit(&wheelVel->leftFront.direction);
	AngleLimit(&wheelVel->rightFront.direction);
	AngleLimit(&wheelVel->leftRear.direction);
	AngleLimit(&wheelVel->rightRear.direction);
}

void Transform2WheelCoordinate(wheel_t * wheelVel)
{
	//����������ϵ�����ӳ���ת��Ϊ��������ϵ�½Ƕ�
//	wheelVel->leftFront.direction  = 90.0f - wheelVel->leftFront.direction;
//	wheelVel->rightFront.direction = 90.0f - wheelVel->rightFront.direction;
//	wheelVel->leftRear.direction   = 90.0f - wheelVel->leftRear.direction;
//	wheelVel->rightRear.direction  = 90.0f - wheelVel->rightRear.direction;

	
	wheelVel->leftFront.direction  -= 90.0f;
	wheelVel->rightFront.direction -= 90.0f;
	wheelVel->leftRear.direction   -= 90.0f;
	wheelVel->rightRear.direction  -= 90.0f;
	
	//���Ƕ�������-180�㵽180��
	AngleLimit(&wheelVel->leftFront.direction);
	AngleLimit(&wheelVel->rightFront.direction);
	AngleLimit(&wheelVel->leftRear.direction);
	AngleLimit(&wheelVel->rightRear.direction);

}

void JudgeVelDirection(wheelVel_t *targetVel , float actualAngle)
{	
	int n = 0;
	float angleErr = 0.0f;
	
	//��Ŀ��ǶȺ͵�ǰʵ�ʽǶ�ת����һ��360��������
	n = (int)(actualAngle / 180.0f) - (int)(actualAngle / 360.0f);
	targetVel->direction = n * 360.0f + targetVel->direction;
	
	//����Ŀ��ǶȺ�ʵ�ʽǶȵ����
	angleErr = targetVel->direction - actualAngle;
	
	//�����������-180�ȵ�180��
	AngleLimit(&angleErr);
	
	//����Ƕ�������90�����ٶȷ��򲢽�Ŀ��Ƕȼ�180��
	if(fabs(angleErr) > 90.0f)
	{
		targetVel->vel = -(targetVel->vel);
		targetVel->direction = targetVel->direction + 180.0f;
		
		//��֤������Ŀ��ǶȺ͵�ǰʵ�ʽǶ���һ��������
		if(targetVel->direction > (n * 360.0f + 180.0f))
		{
			targetVel->direction -= 360.0f;
		}
		else if(targetVel->direction < (n * 360.0f - 180.0f))
		{
			targetVel->direction += 360.0f;
		}
	}
}

int Vel2RotateVel(float vel)
{
	return (int)(vel /(PI * WHEEL_DIAMETER) * SECOUND_PER_MIN * M3508_REDUCTION_RATIO);
}

float RotateVel2Vel(int rpm)
{
	return ((float)rpm / SECOUND_PER_MIN) / M3508_REDUCTION_RATIO * PI * WHEEL_DIAMETER;
}

int WheelAngle2PositionTransform(float angle , int32_t loopShift)
{
	return (int)(((angle / 360.0f) * WHEEL_TURNING_REDUCTION_RATIO + loopShift) * COUNTS_PER_ROUND );
//	return (int)(((angle / 360.0f) * WHEEL_TURNING_REDUCTION_RATIO) * COUNTS_PER_ROUND );
}

float WheelAngle2PositionInverseTransform(int position , int32_t loopShift)
{
	return (float)(((float)position / COUNTS_PER_ROUND - loopShift) / WHEEL_TURNING_REDUCTION_RATIO * 360.0f);
//	return (float)(((float)position / COUNTS_PER_ROUND) / WHEEL_TURNING_REDUCTION_RATIO * 360.0f);
}

int f2int(float floatData)
{
	floatData+= floatData>0?0.5:-0.5;
	
	return (int)floatData;
}

void CalcSteerLoopShift(void)
{
	gRobot.wheelState.steerLoopShift.rf = f2int(C610[0].Position / (WHEEL_TURNING_REDUCTION_RATIO * COUNTS_PER_ROUND));
	gRobot.wheelState.steerLoopShift.lf = f2int(C610[1].Position / (WHEEL_TURNING_REDUCTION_RATIO * COUNTS_PER_ROUND));
	gRobot.wheelState.steerLoopShift.lr = f2int(C610[2].Position / (WHEEL_TURNING_REDUCTION_RATIO * COUNTS_PER_ROUND));
	gRobot.wheelState.steerLoopShift.rr = f2int(C610[3].Position / (WHEEL_TURNING_REDUCTION_RATIO * COUNTS_PER_ROUND));
}

void CalcWheelActAngle(void)
{
	gRobot.wheelState.rightFrontAct.direction = WheelAngle2PositionInverseTransform(C610[0].Position , gRobot.wheelState.steerLoopShift.rf);
	gRobot.wheelState.leftFrontAct.direction  = WheelAngle2PositionInverseTransform(C610[1].Position , gRobot.wheelState.steerLoopShift.lf);
	gRobot.wheelState.leftRearAct.direction   = WheelAngle2PositionInverseTransform(C610[2].Position , gRobot.wheelState.steerLoopShift.lr);
	gRobot.wheelState.rightRearAct.direction  = WheelAngle2PositionInverseTransform(C610[3].Position , gRobot.wheelState.steerLoopShift.rr);
}

float TurnInferiorArc(float targetAngle , float actualAngle)
{
	if(targetAngle - actualAngle > 180.0f)
	{
		return (targetAngle - 360.0f);
	}
	else if(targetAngle - actualAngle < -180.0f)
	{
		return (targetAngle + 360.0f);
	}
	else
	{
		return targetAngle;
	}	
}

void AngleLimit(float *angle)
{
	static uint8_t recursiveTimes = 0;
	recursiveTimes++;
	
	if(recursiveTimes < 100)
	{
		if(*angle > 180.0f)
		{
			*angle -= 360.0f;
			AngleLimit(angle);
		}
		else if(*angle < -180.0f)
		{
			*angle += 360.0f;
			AngleLimit(angle);
		}
	}
	
	recursiveTimes--;
}

float ReturnLimitAngle(float angle)
{
	static uint8_t recursiveTimes = 0;
	recursiveTimes++;

	if(recursiveTimes<100)
	{
		if(angle > 180.0f)
		{
			angle = ReturnLimitAngle(angle - 360.0f);
		}
		else if(angle < -180.0f)
		{
			angle = ReturnLimitAngle(angle + 360.0f);
		}
	}
	
	recursiveTimes--;
	
	return angle;	
}
