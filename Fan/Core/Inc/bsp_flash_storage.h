/**
 * ============================================================================
 *  内部 Flash 模拟 EEPROM 存储模块
 *  利用 STM32F103C8T6 最后一页 (Page63, 0x0800FC00) 存储用户参数
 *  掉电不丢失, 无需外部 EEPROM 芯片, 成本 0 元
 * ============================================================================
 */
#ifndef __BSP_FLASH_STORAGE_H
#define __BSP_FLASH_STORAGE_H

#include "bsp_config.h"

/* Flash 存储数据结构 (对齐到 4 字节) */
typedef struct {
    uint32_t magic;         /* 魔数校验, 若不匹配说明首次使用或数据损坏 */
    float    pid_kp;        /* PID Kp */
    float    pid_ki;        /* PID Ki */
    float    pid_kd;        /* PID Kd */
    uint32_t fan_delay_sec; /* 延时停机时间 */
    uint16_t acs_zero_adc;  /* ACS712 校准零点 (可选保存) */
    uint16_t reserved;      /* 保留对齐 */
} FlashStorage_t;

/**
 * @brief 从 Flash 加载参数
 * @param data 输出数据指针
 * @return true = 成功且魔数匹配; false = 首次使用或数据损坏
 */
bool Flash_Load(FlashStorage_t *data);

/**
 * @brief 保存参数到 Flash
 * @param data 要保存的数据指针
 * @return true = 成功
 *
 * 注意: 此函数会擦除整页后再写入, 调用频率不宜过高 (Flash 寿命 ~10K 次)
 *       建议仅在按键修改参数后调用
 */
bool Flash_Save(const FlashStorage_t *data);

#endif /* __BSP_FLASH_STORAGE_H */
