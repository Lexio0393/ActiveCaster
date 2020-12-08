#ifndef _MANUL_H
#define _MANUL_H

#include "stm32f4xx.h"

#include "motion.h"

typedef enum
{
		RemoteStatus_VelLoop_Kick = (uint8_t)0x00,			
    RemoteStatus_Reset  			= (uint8_t)0x01,	
		RemoteStatus_VelLoop_Rec	= (uint8_t)0x02,
}RemoteStatus_t;


void Task_RemoteAnalyse(void);

extern vector_t finalVel;

#endif
