#ifndef _CYLINDER_H
#define _CYLINDER_H

#include "bsp_io.h"
#include "bsp_systick.h"


#ifndef KICK_GAS_PRESSURE
#define KICK_GAS_PRESSURE (0.423f)
#endif

//踢球机构正在调整
#define KICKER_ADJUSTING (0)

//踢球机构就绪
#define KICKER_ADJ_READY (1)

#define KICKER_UNFINISH	(1)
#define KICKER_FINISH		(0)

void KickerInit(void);
void BSP_CYLINDER_Init(void);

//void Task_GetKickerStatus(void);
//void Task_KickConotrol(void);
void Task_JudgeKickerFlag(void);
void KickControl(void);

//float GetGasPressureADCValue(void);
//float GetGasPressure(void);
void GasPressureControl(float gasPressure);

//踢球机构气缸推出
void KickerPush(void);

//踢球机构气缸收回
void KickerPull(void);

//发射机构发射
void KickerKick(void);

//踢球机构复位
void KickerReset(void);

//void KickerPosCmdReset(void);

void KickerCmdReset(void);

uint8_t KickerCheckState(void);
uint8_t KickerFinishState(void);
#endif
