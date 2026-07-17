/**
 * ============================================================================
 *  PID 控制器模块实现
 *  位置式 PID, 带积分限幅和输出钳位, 防积分饱和 (Anti-Windup)
 *  反向作用: error = measured - setpoint (测量值越高, 输出越大)
 *  引入真实 dt (秒), 消除主循环时间波动对积分/微分的影响
 * ============================================================================
 */
#include "bsp_pid.h"
#include "stm32f1xx_hal.h"

/* dt 限制常量 */
#define PID_DT_MIN  0.01f   /* 10ms, 防止除零 */
#define PID_DT_MAX  2.0f    /* 2s, 防止长时间挂起后积分暴增 */

/* ======================== 公开接口 ======================== */

void PID_Init(PID_Controller_t *pid, float kp, float ki, float kd,
              float out_min, float out_max)
{
    pid->Kp = kp;
    pid->Ki = ki;
    pid->Kd = kd;
    pid->setpoint   = 0.0f;
    pid->integral    = 0.0f;
    pid->prev_error  = 0.0f;
    pid->output_min  = out_min;
    pid->output_max  = out_max;
    pid->integral_max = (out_max - out_min) * 0.6f;
    pid->last_tick   = HAL_GetTick();
}

float PID_Compute(PID_Controller_t *pid, float measured)
{
    uint32_t now = HAL_GetTick();
    float dt = (float)(now - pid->last_tick) / 1000.0f;
    pid->last_tick = now;

    if (dt < PID_DT_MIN) dt = PID_DT_MIN;
    if (dt > PID_DT_MAX) dt = PID_DT_MAX;

    /* 反向作用: 室内湿度越高于室外, 偏差越大, 风扇越快 */
    float error = measured - pid->setpoint;

    /* 积分项 (乘以 dt) */
    pid->integral += error * dt;

    /* Anti-Windup */
    if (pid->integral >  pid->integral_max) pid->integral =  pid->integral_max;
    if (pid->integral < -pid->integral_max) pid->integral = -pid->integral_max;

    /* 微分项 (除以 dt) */
    float derivative = (error - pid->prev_error) / dt;
    pid->prev_error = error;

    /* 位置式 PID 输出 */
    float output = pid->Kp * error
                 + pid->Ki * pid->integral
                 + pid->Kd * derivative;

    /* 输出钳位 */
    if (output > pid->output_max) output = pid->output_max;
    if (output < pid->output_min) output = pid->output_min;

    return output;
}

void PID_Reset(PID_Controller_t *pid)
{
    pid->integral   = 0.0f;
    pid->prev_error = 0.0f;
    pid->last_tick  = HAL_GetTick();
}

void PID_SetTunings(PID_Controller_t *pid, float kp, float ki, float kd)
{
    pid->Kp = kp;
    pid->Ki = ki;
    pid->Kd = kd;
}

void PID_SetSetpoint(PID_Controller_t *pid, float sp)
{
    pid->setpoint = sp;
}
