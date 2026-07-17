/**
 * ============================================================================
 *  AHT20 温湿度传感器驱动 (I2C)
 *  支持阻塞和非阻塞两种读取模式
 * ============================================================================
 */
#ifndef __BSP_AHT20_H
#define __BSP_AHT20_H

#include "bsp_config.h"

#define AHT20_I2C_ADDR      (0x38 << 1)  /* 7-bit 地址左移 1 位 = 0x70 */

/* 非阻塞状态机状态 */
typedef enum {
    AHT20_STATE_IDLE = 0,
    AHT20_STATE_TRIGGERED,
    AHT20_STATE_READY,
    AHT20_STATE_ERROR,
} AHT20_State_t;

/* 非阻塞状态机句柄 */
typedef struct {
    I2C_HandleTypeDef *hi2c;
    AHT20_State_t state;
    uint32_t trigger_tick;    /* 发送测量命令的时刻 */
    float    temperature;     /* 最近一次成功读取的温度 */
    float    humidity;        /* 最近一次成功读取的湿度 */
    uint8_t  fail_count;      /* 连续失败次数 */
    bool     data_fresh;      /* 本轮是否有新数据 */
} AHT20_Handle_t;

#define AHT20_MEASURE_TIME_MS  80   /* 测量完成所需时间 */
#define AHT20_BUSY_EXTRA_MS    50   /* Busy 时额外等待 */
#define AHT20_MAX_FAIL_COUNT   20   /* 连续失败次数上限 */

/**
 * @brief 初始化 AHT20 (发送校准命令, 阻塞式)
 */
HAL_StatusTypeDef AHT20_Init(I2C_HandleTypeDef *hi2c);

/**
 * @brief 初始化非阻塞句柄
 */
void AHT20_Handle_Init(AHT20_Handle_t *h, I2C_HandleTypeDef *hi2c);

/**
 * @brief 非阻塞状态机轮询 (在主循环中调用)
 *        自动管理 触发→等待→读取 的完整流程
 *        读取完成后 h->data_fresh = true, 数据在 h->temperature/humidity
 */
void AHT20_Poll(AHT20_Handle_t *h);

/**
 * @brief 阻塞式读取 (兼容旧接口, 不建议在主循环中使用)
 */
HAL_StatusTypeDef AHT20_Read(I2C_HandleTypeDef *hi2c, float *temperature, float *humidity);

#endif /* __BSP_AHT20_H */
