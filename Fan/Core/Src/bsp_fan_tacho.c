/**
 * ============================================================================
 *  4线风扇测速驱动实现
 *  TIM3_CH2 (PA7) 输入捕获, 下降沿触发
 * ============================================================================
 */
#include "bsp_fan_tacho.h"
#include "tim.h"

/* 内部变量 */
static volatile uint32_t s_capture_val   = 0;   /* 本次捕获值 */
static volatile uint32_t s_capture_diff  = 0;   /* 两次捕获差值 (周期) */
static volatile uint32_t s_last_capture  = 0;   /* 上一次捕获值 */
static volatile uint32_t s_last_tick     = 0;   /* 最后一次捕获的系统 Tick */
static volatile bool     s_first_capture = true;

/* ======================== 公开接口 ======================== */

void FanTacho_Init(void)
{
    s_first_capture = true;
    s_capture_diff  = 0;
    s_last_capture  = 0;
    s_last_tick     = HAL_GetTick();

    /* 启动 TIM3 CH2 输入捕获中断 */
    HAL_TIM_IC_Start_IT(&htim3, TIM_CHANNEL_2);
}

uint16_t FanTacho_GetRPM(void)
{
    /* 超时判断: 若太久没有脉冲, 认为风扇已停转 */
    if ((HAL_GetTick() - s_last_tick) > FAN_TACH_TIMEOUT_MS) {
        return 0;
    }

    uint32_t diff = s_capture_diff;
    if (diff == 0) return 0;

    /*
     * RPM = 60 × TIM_FREQ / (diff × 每转脉冲数)
     *
     * 例: TIM_FREQ=1MHz, diff=25000, 每转2脉冲
     *     RPM = 60×1000000 / (25000×2) = 1200
     */
    uint32_t rpm = (60UL * FAN_TACH_TIM_FREQ_HZ) / (diff * FAN_TACH_PULSES_PER_REV);

    /* 合理性过滤: 排风扇一般 500~3000 RPM */
    if (rpm > 10000) rpm = 0;

    return (uint16_t)rpm;
}

/* ======================== 中断回调 ======================== */

void FanTacho_CaptureCallback(TIM_HandleTypeDef *htim)
{
    if (htim->Instance == TIM3 && htim->Channel == HAL_TIM_ACTIVE_CHANNEL_2) {
        s_capture_val = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_2);
        s_last_tick = HAL_GetTick();

        if (s_first_capture) {
            s_first_capture = false;
        } else {
            /* 处理定时器溢出: 差值自动环回 (16位计数器) */
            if (s_capture_val >= s_last_capture) {
                s_capture_diff = s_capture_val - s_last_capture;
            } else {
                s_capture_diff = (0xFFFF - s_last_capture) + s_capture_val + 1;
            }
        }
        s_last_capture = s_capture_val;
    }
}
