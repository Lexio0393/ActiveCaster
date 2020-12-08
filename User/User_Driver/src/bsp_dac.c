#include "bsp_dac.h"

void BSP_DAC1_Init(void)
{
	GPIO_InitTypeDef  GPIO_InitStructure;
	DAC_InitTypeDef	  DAC_InitType;
	
	RCC_AHB1PeriphClockCmd(Periph_GPIO_DAC1, ENABLE);			//ʹ��GPIOAʱ��
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_DAC, ENABLE);		//ʹ��DACʱ��
	   
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_DAC1;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AN;					//ģ������
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_DOWN;				//����
	GPIO_Init(GPIO_Port_DAC1, &GPIO_InitStructure);				//��ʼ��

	DAC_InitType.DAC_Trigger=DAC_Trigger_None;														//��ʹ�ô������� TEN1=0
	DAC_InitType.DAC_WaveGeneration=DAC_WaveGeneration_None;							//��ʹ�ò��η���
	DAC_InitType.DAC_LFSRUnmask_TriangleAmplitude=DAC_LFSRUnmask_Bit0;		//���Ρ���ֵ����
	DAC_InitType.DAC_OutputBuffer=DAC_OutputBuffer_Disable ;							//DAC1�������ر� BOFF1=1
	DAC_Init(DAC_Channel_1,&DAC_InitType);	 															//��ʼ��DACͨ��1

	DAC_Cmd(DAC_Channel_1, ENABLE);  							//ʹ��DACͨ��1
  
	DAC_SetChannel1Data(DAC_Align_12b_R, 0);  		//12λ�Ҷ������ݸ�ʽ����DACֵ	
	
}

//����ͨ��1�����ѹ
//Vol:0~3300,����0~3.3V
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
