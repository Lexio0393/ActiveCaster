#include "checkpos.h"


void CheckPos(void)
{
	static uint8_t judgeStopFlag = 0, judgeStopDoneFlag = 0;
	
	switch(gRobot.kickStatus)
	{	
		
		//等待触发0
		case waitForStart:
		{
			//按下遥控器，开始任务
			if(gRobot.teleCommand.nextFlag == TELENEXT)
			{				
				gRobot.teleCommand.nextFlag = TELENOCMD;
				
				gRobot.kickStatus = go2Kick1stBall;
			}
			break;
		}
		case go2Kick1stBall:
		{	
			Point_t presentPoint;
			Point_t finalPoint;
			
			presentPoint.x = GetX();
			presentPoint.y = GetY();
//		finalPoint.x = 
//		finalPoint.y = 
			
			float dis2FinalX = presentPoint.x - finalPoint.x;
			float dis2FinalY = presentPoint.y - finalPoint.y;
					
			if((sqrtf(dis2FinalX*dis2FinalX + dis2FinalY*dis2FinalY)<30.0f && JudgeSpeedLessEqual(300.0f))\
							|| (fabs(GetX())<2500.0f	&& JudgeSpeedLessEqual(300.0f)))
			{
				if(gRobot.kicker.ballNum % 2 == 1)
				{
					OutputVel2Wheel(700.0f,180.0f,0.0f);    //速度、方向待修改
				}
				else if(gRobot.kicker.ballNum % 2 == 0)
				{
					OutputVel2Wheel(700.0f,180.0f,0.0f);    //速度、方向待修改
				}
				
//				gRobot.teleCommand.nextFlag = TELENOCMD;
				
				judgeStopFlag = 0;
				judgeStopDoneFlag = 0;
				
				gRobot.kickStatus = Kick1stBall;
				return;
			}
			break;
		}
		
		case Kick1stBall：
		{
			static uint8_t posAchieve = 0, posTimeOut = 0;
			
//			float posX , posY = 0.0f;	
//			posX = gRobot.kicker.targetPos.point.x;
//			posY = gRobot.kicker.targetPos.point.y;
			
			if(judgeStopDoneFlag == 0)	//说明底盘未停止
			{
				if(gRobot.kicker.ballNum % 2 == 1)
				{
					OutputVel2Wheel(700.0f,180.0f,0.0f);	//速度、方向待修改
				}
				else if(gRobot.kicker.ballNum % 2 == 0)
				{
					OutputVel2Wheel(700.0f,0.0f,0.0f);
				}	
			}
			
			//通过X坐标判断是否停止，距离浮动3mm，持续实际5ms以上
			if(JudgeStop2(3.f,5) && judgeStopFlag==0)
			{
				judgeStopFlag = 1;
				//处理函数
			}
			
			if(judgeStopFlag && judgeStopDoneFlag == 0)
			{
				if(fabs(GetY()) > 1500.0f)					//坐标待修改 if(fabs(GetY()) > fabs(posY - gRobot.kicker.targetPos.distanceLimit) && fabs(GetX()) < fabs(posX))
				{
					posAchieve	=	1;
				}
				else
				{
					posTimeOut++;
					posTimeOut = posTimeOut > 100 ? 100 : posTimeOut;
				}
			}
			
			if((posAchieve || posTimeOut > 20) && judgeStopDoneFlag == 0)
			{		
				judgeStopDoneFlag = 1;

				//刹车、踢球
			}
			
			if(judgeStopDoneFlag == 1)// && gRobot.teleCommand.nextFlag == TELENEXT)
			{
				posAchieve = 0;
				posTimeOut = 0;
				judgeStopFlag = 0;
				judgeStopDoneFlag= 0;

//				gRobot.teleCommand.nextFlag = TELENOCMD;

				gRobot.walkStatus = go2Try1stBall;
			}
			break;
		}
	}
}



uint8_t JudgeStop(float disChange,uint8_t countTime)
{
	static uint8_t counter = 0;
	float disX , disY = 0.0f;
	static float posXRecord , posYRecord = 0.0f;
	
	disX = GetX() - posXRecord;
	disY = GetY() - posYRecord;
	
	if(sqrtf(disX * disX + disY * disY)<=disChange)
	{
		counter++;
	}
	else
	{
		counter = 0;
	}
	
	posXRecord = GetX();
	posYRecord = GetY();
	
	if(counter<=countTime)
	{
		return 0;
	}
	else
	{
		counter = 0;
		return 1;
	}
}
uint8_t JudgeStop2(float disChange,uint8_t countTime)
{
	static uint8_t counter = 0;
	float disX = 0.0f;
	static float posXRecord = 0.0f;
	
	disX = GetX() - posXRecord;
//	disY = GetY() - posYRecord;
	

	if(fabs(disX) < disChange)
	{
		counter++;
	}
	else
	{
		counter = 0;
	}

	posXRecord = GetX();
//	posYRecord = GetY();
	
	if(counter<=countTime)
	{
		return 0;
	}
	else
	{
		counter = 0;
		return 1;
	}
}




uint8_t JudgeSpeedLessEqual(float speedCompared)
{
	PosVel_t actSpeed = GetSpeedWithoutOmega();
	float speedX = actSpeed.x;
	float speedY = actSpeed.y;
	
	float speed = sqrtf(speedX * speedX + speedY * speedY);
	
	if(speed>speedCompared)
	{
		return 0;
	}
	else
	{
		return 1;
	}
}


Pose_t GetPosPresent(void)
{
	Pose_t pos;
	pos.point.x = GetX();
	pos.point.y = GetY();
	pos.direction   = GetAngle();
	pos.vel = 0.0f;
	return pos;
}


float CalculateLineAngle(Point_t actualPos, Point_t targetPos)
{
	float xErr, yErr;
	float chordLength;
	float direction;
	float cos_theta;
	
	Point_t actPos = actualPos;
	Point_t tarPos = targetPos;
	
	xErr = tarPos.x - actPos.x;
	yErr = tarPos.y - actPos.y;
	chordLength = sqrtf(xErr * xErr + yErr * yErr);
	
	cos_theta = xErr / chordLength;
	
	direction = acos(cos_theta);
	
	return direction;
}