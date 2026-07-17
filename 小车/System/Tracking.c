#include "stm32f10x.h"                  // Device header
#include "CAR.h"



void Tracking(void)
{
	if (GPIO_ReadInputDataBit(GPIOB,GPIO_Pin_14)==1&&
			GPIO_ReadInputDataBit(GPIOB,GPIO_Pin_15)==1)
		{
			Go_Ahead();
		}
	else if(GPIO_ReadInputDataBit(GPIOB,GPIO_Pin_14)==0&&
			GPIO_ReadInputDataBit(GPIOB,GPIO_Pin_15)==0)
		{
			Car_Stop();
		}
	else if(GPIO_ReadInputDataBit(GPIOB,GPIO_Pin_14)==1&&
			GPIO_ReadInputDataBit(GPIOB,GPIO_Pin_15)==0)
		{
			Turn_Right();
		}
	else if(GPIO_ReadInputDataBit(GPIOB,GPIO_Pin_14)==0&&
			GPIO_ReadInputDataBit(GPIOB,GPIO_Pin_15)==1)
		{
			Turn_Left();
		}
}
