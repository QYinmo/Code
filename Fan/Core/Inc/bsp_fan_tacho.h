/**
 * ============================================================================
 *  4线风扇测速驱动 (TACH 信号, TIM3_CH2 输入捕获, PA7)
 *
 *  4线风扇 TACH 引脚每转输出 2 个下降沿脉冲 (与3线完全一致)
 *  通过 TIM3 输入捕获测量脉冲周期, 计算 RPM
 * ============================================================================
 */
#ifndef __BSP_FAN_TACHO_H
#define __BSP_FAN_TACHO_H

#include "bsp_config.h"

/**
 * @brief 初始化风扇测速 (启动 TIM3 CH2 输入捕获中断)
 */
void FanTacho_Init(void);

/**
 * @brief 获取当前风扇转速 (RPM)
 * @return RPM, 若超时无脉冲返回 0
 */
uint16_t FanTacho_GetRPM(void);

/**
 * @brief TIM3 输入捕获回调 (在 stm32f1xx_it.c 中调用)
 *        由 HAL_TIM_IC_CaptureCallback 转发
 */
void FanTacho_CaptureCallback(TIM_HandleTypeDef *htim);

#endif /* __BSP_FAN_TACHO_H */
