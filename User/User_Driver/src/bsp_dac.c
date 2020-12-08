#include "bsp_dac.h"

void BSP_DAC1_Init(void)
{
	GPIO_InitTypeDef  GPIO_InitStructure;
	DAC_InitTypeDef	  DAC_InitType;
	
	RCC_AHB1PeriphClockCmd(Periph_GPIO_DAC1, ENABLE);			//使能GPIOA时钟
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_DAC, ENABLE);		//使能DAC时钟
	   
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_DAC1;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AN;					//模拟输入
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_DOWN;				//下拉
	GPIO_Init(GPIO_Port_DAC1, &GPIO_InitStructure);				//初始化

	DAC_InitType.DAC_Trigger=DAC_Trigger_None;														//不使用触发功能 TEN1=0
	DAC_InitType.DAC_WaveGeneration=DAC_WaveGeneration_None;							//不使用波形发生
	DAC_InitType.DAC_LFSRUnmask_TriangleAmplitude=DAC_LFSRUnmask_Bit0;		//屏蔽、幅值设置
	DAC_InitType.DAC_OutputBuffer=DAC_OutputBuffer_Disable ;							//DAC1输出缓存关闭 BOFF1=1
	DAC_Init(DAC_Channel_1,&DAC_InitType);	 															//初始化DAC通道1

	DAC_Cmd(DAC_Channel_1, ENABLE);  							//使能DAC通道1
  
	DAC_SetChannel1Data(DAC_Align_12b_R, 0);  		//12位右对齐数据格式设置DAC值	
	
}

//设置通道1输出电压
//Vol:0~3300,代表0~3.3V
void DAC_Set_Vol(uint16_t Vol)
{
	double temp = Vol;
	
	temp /= 1000;
	temp = temp * 4096 / 3.3;
	
	DAC_SetChannel1Data(DAC_Align_12b_R, temp);
}

void DAC_SetValue(uint16_t Value)
{
	DAC_SetChannel1Data(DAC_Align_12b_R, Value);
}

uint16_t DAC_GetValue(void)
{
	return DAC_GetDataOutputValue(DAC_Channel_1);
}
