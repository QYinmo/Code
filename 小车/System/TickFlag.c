#include "stm32f10x.h"

volatile uint32_t TickFlag = 0;										// 定时器滴答标志，每次定时器溢出时更新

// 初始化定时器1，产生1ms滴答
void Timer_Init(void)
{
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM1, ENABLE); 			// 使能定时器1时钟
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);  			// 使能GPIOA时钟
    
    // 配置定时器
    TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStructure;
    TIM_TimeBaseInitStructure.TIM_Period = 999; 					 // 设置计数周期为999，周期1ms
    TIM_TimeBaseInitStructure.TIM_Prescaler = 71; 					 // 72MHz / (71 + 1) = 1MHz，1微秒计数
    TIM_TimeBaseInitStructure.TIM_ClockDivision = TIM_CKD_DIV1;
    TIM_TimeBaseInitStructure.TIM_CounterMode = TIM_CounterMode_Up;
    TIM_TimeBaseInit(TIM1, &TIM_TimeBaseInitStructure);
    
    // 配置中断
    TIM_ITConfig(TIM1, TIM_IT_Update, ENABLE);	 					 // 使能更新中断
    NVIC_EnableIRQ(TIM1_UP_IRQn);  									 // 使能定时器1中断
    
    TIM_Cmd(TIM1, ENABLE);  										 // 启动定时器
}

// 定时器1中断服务程序（假设使用定时器1，周期1ms）
void TIM1_UP_IRQHandler(void) 
{
    if (TIM_GetITStatus(TIM1, TIM_IT_Update) != RESET) 
	{
        TickFlag++;  												// 每次定时器溢出，TickFlag 自增
        TIM_ClearITPendingBit(TIM1, TIM_IT_Update);  				// 清除中断标志
    }
}

// 非阻塞延时函数，依赖TickFlag
void NonBlockingDelay_ms(uint32_t ms)
{
    uint32_t startTick = TickFlag;
    while (TickFlag - startTick < ms)
	{
        /* 在这里可以执行其他任务，等待时间过去 */
    }
}
