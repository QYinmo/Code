/**
 * ============================================================================
 *  绝对湿度计算模块实现
 *  Magnus-Tetens 公式 + 室内外对比收敛判断
 * ============================================================================
 */
#include "bsp_humidity.h"
#include <math.h>

/**
 * Magnus-Tetens 公式常数
 *   a = 17.67
 *   b = 243.5  (℃)
 *   c = 6.112  (hPa, 0℃ 时饱和蒸气压)
 *   Mw / R = 216.7  (g·K / J)
 */
#define MAGNUS_A  17.67f
#define MAGNUS_B  243.5f
#define MAGNUS_C  6.112f
#define AH_CONST  216.7f

float Humidity_CalcAbsolute(float temp_c, float rh_pct)
{
    /* 饱和蒸气压 Es (hPa) */
    float es = MAGNUS_C * expf((MAGNUS_A * temp_c) / (MAGNUS_B + temp_c));

    /* 实际蒸气压 Ea (hPa) */
    float ea = es * rh_pct / 100.0f;

    /* 绝对湿度 AH (g/m³) = 216.7 × Ea / (T + 273.15) */
    float abs_humi = AH_CONST * ea / (temp_c + 273.15f);

    return abs_humi;
}

bool Humidity_IsConverged(float indoor_abs_humi, float outdoor_abs_humi)
{
    float diff = indoor_abs_humi - outdoor_abs_humi;
    if (diff < 0.0f) diff = -diff;
    return (diff < ABS_HUMI_CONVERGE_THRESHOLD);
}
