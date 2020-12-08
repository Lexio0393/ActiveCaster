#ifndef _BSP_DAC_H
#define _BSP_DAC_H

#include "stm32f4xx.h"

#define Periph_GPIO_DAC1	RCC_AHB1Periph_GPIOA
#define GPIO_Port_DAC1		GPIOA
#define GPIO_Pin_DAC1			GPIO_Pin_4

void BSP_DAC1_Init(void);

void DAC_Set_Vol(uint16_t Vol);

#endif
