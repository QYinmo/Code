/**
 * ============================================================================
 *  PID 控制器模块
 *  支持位置式 PID, 带积分限幅和输出钳位
 *  排风扇为反向作用系统: 测量值 > 设定值时输出增大
 * ============================================================================
 */
#ifndef __BSP_PID_H
#define __BSP_PID_H

#include "bsp_config.h"

/* PID 控制器结构体 */
typedef struct {
    float Kp;
    float Ki;
    float Kd;
    float setpoint;     /* 目标值 (SP) */
    float integral;     /* 积分累积 */
    float prev_error;   /* 上一次偏差 */
    float output_min;   /* 输出下限 */
    float output_max;   /* 输出上限 */
    float integral_max; /* 积分限幅 (防积分饱和) */
    uint32_t last_tick; /* 上次计算时刻 (ms) */
} PID_Controller_t;

/**
 * @brief 初始化 PID 控制器
 * @param pid       控制器结构体指针
 * @param kp,ki,kd  PID 增益
 * @param out_min   输出下限
 * @param out_max   输出上限
 */
void PID_Init(PID_Controller_t *pid, float kp, float ki, float kd,
              float out_min, float out_max);

/**
 * @brief PID 单步计算 (自动获取 dt)
 * @param pid       控制器结构体指针
 * @param measured  当前测量值 (PV)
 * @return 控制输出 (CV)
 *
 * 偏差方向: error = measured - setpoint (反向作用, 适用于排风扇)
 * 测量值越高于设定值, 输出越大
 */
float PID_Compute(PID_Controller_t *pid, float measured);

/**
 * @brief 重置 PID 积分与历史
 */
void PID_Reset(PID_Controller_t *pid);

/**
 * @brief 更新 PID 参数 (运行时热调参)
 */
void PID_SetTunings(PID_Controller_t *pid, float kp, float ki, float kd);

/**
 * @brief 设置目标值
 */
void PID_SetSetpoint(PID_Controller_t *pid, float sp);

#endif /* __BSP_PID_H */
