/**
 * ============================================================================
 *  MLX90614 红外测温驱动 (I2C / SMBus)
 * ============================================================================
 */
#ifndef __BSP_MLX90614_H
#define __BSP_MLX90614_H

#include "bsp_config.h"

#define MLX90614_I2C_ADDR       (0x5A << 1)  /* 默认地址 0xB4 */

/* RAM 寄存器地址 */
#define MLX90614_REG_TA         0x06    /* 环境温度 */
#define MLX90614_REG_TOBJ1      0x07    /* 目标温度 1 */

/**
 * @brief 读取环境温度 Ta
 */
HAL_StatusTypeDef MLX90614_ReadAmbient(I2C_HandleTypeDef *hi2c, float *temp);

/**
 * @brief 读取目标(物体)温度 Tobj
 */
HAL_StatusTypeDef MLX90614_ReadObject(I2C_HandleTypeDef *hi2c, float *temp);

/**
 * @brief 同时读取环境温度和目标温度
 */
HAL_StatusTypeDef MLX90614_ReadAll(I2C_HandleTypeDef *hi2c, float *ta, float *tobj);

#endif /* __BSP_MLX90614_H */
