#ifndef _BSP_RTC_H
#define _BSP_RTC_H

#include "stm32f4xx.h"

void SetDate(uint16_t Year, uint16_t Month, uint16_t Day);
void SetTime(uint16_t Hour, uint16_t Minute, uint8_t Second);

void SetStartTime(void);
void SetEndTime(void);
float GetTimeInterval(void);

uint8_t RTC_Time_Init(void);
	
#endif
