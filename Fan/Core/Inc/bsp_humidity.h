/**
 * ============================================================================
 *  绝对湿度计算模块
 *  使用 Magnus-Tetens 经验公式, 将 (温度, 相对湿度) 转换为绝对湿度 (g/m³)
 * ============================================================================
 */
#ifndef __BSP_HUMIDITY_H
#define __BSP_HUMIDITY_H

#include "bsp_config.h"

/**
 * @brief 计算绝对湿度 (g/m³)
 * @param temp_c  温度 (℃)
 * @param rh_pct  相对湿度 (%, 0~100)
 * @return 绝对湿度 (g/m³)
 *
 * 原理:
 *   饱和蒸气压 Es = 6.112 × exp(17.67 × T / (T + 243.5))  [hPa]
 *   实际蒸气压 Ea = Es × RH / 100
 *   绝对湿度   AH = 216.7 × Ea / (T + 273.15)              [g/m³]
 */
float Humidity_CalcAbsolute(float temp_c, float rh_pct);

/**
 * @brief 判断室内湿度是否已收敛到室外水平
 * @param indoor_abs_humi  室内绝对湿度 g/m³
 * @param outdoor_abs_humi 室外绝对湿度 g/m³
 * @return true = 已收敛
 */
bool Humidity_IsConverged(float indoor_abs_humi, float outdoor_abs_humi);

#endif /* __BSP_HUMIDITY_H */
