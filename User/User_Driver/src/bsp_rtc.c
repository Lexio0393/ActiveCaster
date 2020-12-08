#include "bsp_rtc.h"

RTC_TimeTypeDef RTC_TimeStruct_Start, RTC_TimeStruct_End;

float SubSecond_Start, SubSecond_End;

void SetDate(uint16_t Year, uint16_t Month, uint16_t Day)
{
	RTC_DateTypeDef RTC_DateStruct;
	RTC_DateStruct.RTC_Year  = Year - 2000;
	RTC_DateStruct.RTC_Month = Month;
	RTC_DateStruct.RTC_Date  = Day;
	RTC_DateStruct.RTC_WeekDay = (Day + 1 + 2 * Month + 3 * (Month + 1) / 5 + Year + \
										Year / 4 - Year / 100 + Year / 400) % 7;
	RTC_SetDate(RTC_Format_BIN, &RTC_DateStruct);
}

void SetTime(uint16_t Hour, uint16_t Minute, uint8_t Second)
{
	RTC_TimeTypeDef RTC_TimeStruct;
	RTC_TimeStruct.RTC_Hours = Hour;
	RTC_TimeStruct.RTC_H12 = Hour > 12 ? RTC_H12_AM : RTC_H12_PM;
	RTC_TimeStruct.RTC_Minutes = Minute;
	RTC_TimeStruct.RTC_Seconds = Second;
	RTC_SetTime(RTC_Format_BIN, &RTC_TimeStruct);
}

void SetStartTime(void)
{
	SubSecond_Start = RTC_GetSubSecond();
	RTC_GetTime(RTC_Format_BIN, &RTC_TimeStruct_Start);
}	

void SetEndTime(void)
{
	SubSecond_End = RTC_GetSubSecond();
	RTC_GetTime(RTC_Format_BIN, &RTC_TimeStruct_End);	
}

float GetTimeInterval(void)
{
	return 
		(RTC_TimeStruct_End.RTC_Hours - RTC_TimeStruct_Start.RTC_Hours) * 3600.0f
		+	(RTC_TimeStruct_End.RTC_Minutes - RTC_TimeStruct_Start.RTC_Minutes) * 60.0f
		+	(RTC_TimeStruct_End.RTC_Seconds - RTC_TimeStruct_Start.RTC_Seconds)
		+	(SubSecond_Start - SubSecond_End);		//RTC亚秒定时器是倒计时方式，所以应倒着减
}
	
uint8_t RTC_Time_Init(void)
{
	RTC_InitTypeDef RTC_InitStructure;
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_PWR, ENABLE);
	PWR_BackupAccessCmd(ENABLE);
	
	{
		RCC_LSEConfig(RCC_LSE_ON);
		while(RCC_GetFlagStatus(RCC_FLAG_LSERDY) == RESET);
		
		RCC_RTCCLKConfig(RCC_RTCCLKSource_LSE);
		RCC_RTCCLKCmd(ENABLE);
		
		RTC_InitStructure.RTC_AsynchPrediv = 0x7F;
		RTC_InitStructure.RTC_SynchPrediv  = 0xFF;
		RTC_InitStructure.RTC_HourFormat   = RTC_HourFormat_24;
		
		RTC_Init(&RTC_InitStructure);
	}
	
	return 0;
}
