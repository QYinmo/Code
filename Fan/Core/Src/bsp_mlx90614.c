/**
 * ============================================================================
 *  MLX90614 红外测温驱动实现
 *  通信协议: SMBus (兼容 I2C)
 *  供电: 3.3V
 *  PEC: CRC-8, 多项式 x^8+x^2+x+1 (0x07)
 * ============================================================================
 */
#include "bsp_mlx90614.h"

/* SMBus PEC (CRC-8) 校验 */
static uint8_t MLX90614_CRC8(const uint8_t *data, uint8_t len)
{
    uint8_t crc = 0;
    for (uint8_t i = 0; i < len; i++) {
        crc ^= data[i];
        for (uint8_t bit = 0; bit < 8; bit++) {
            if (crc & 0x80)
                crc = (crc << 1) ^ 0x07;
            else
                crc <<= 1;
        }
    }
    return crc;
}

/**
 * @brief 从 MLX90614 指定寄存器读取温度 (带 PEC 校验)
 *
 * SMBus 帧:
 *   写: [SA_W] [CMD]
 *   读: [SA_R] [DATA_LOW] [DATA_HIGH] [PEC]
 * PEC 计算覆盖: SA_W, CMD, SA_R, DATA_LOW, DATA_HIGH
 */
static HAL_StatusTypeDef MLX90614_ReadReg(I2C_HandleTypeDef *hi2c,
                                           uint8_t reg, float *temp)
{
    uint8_t data[3];  /* LOW, HIGH, PEC */
    HAL_StatusTypeDef ret;

    ret = HAL_I2C_Mem_Read(hi2c, MLX90614_I2C_ADDR, reg,
                           I2C_MEMADD_SIZE_8BIT, data, 3, 50);
    if (ret != HAL_OK) return ret;

    /* PEC 校验 */
    uint8_t pec_buf[5];
    pec_buf[0] = MLX90614_I2C_ADDR;       /* SA + W */
    pec_buf[1] = reg;                      /* CMD */
    pec_buf[2] = MLX90614_I2C_ADDR | 1;   /* SA + R */
    pec_buf[3] = data[0];                  /* DATA_LOW */
    pec_buf[4] = data[1];                  /* DATA_HIGH */
    if (MLX90614_CRC8(pec_buf, 5) != data[2]) {
        return HAL_ERROR;
    }

    uint16_t raw = ((uint16_t)data[1] << 8) | data[0];

    if (raw & 0x8000) {
        *temp = -999.0f;
        return HAL_ERROR;
    }

    *temp = (float)raw * 0.02f - 273.15f;

    return HAL_OK;
}

HAL_StatusTypeDef MLX90614_ReadAmbient(I2C_HandleTypeDef *hi2c, float *temp)
{
    return MLX90614_ReadReg(hi2c, MLX90614_REG_TA, temp);
}

HAL_StatusTypeDef MLX90614_ReadObject(I2C_HandleTypeDef *hi2c, float *temp)
{
    return MLX90614_ReadReg(hi2c, MLX90614_REG_TOBJ1, temp);
}

HAL_StatusTypeDef MLX90614_ReadAll(I2C_HandleTypeDef *hi2c, float *ta, float *tobj)
{
    HAL_StatusTypeDef ret;

    ret = MLX90614_ReadAmbient(hi2c, ta);
    if (ret != HAL_OK) return ret;

    ret = MLX90614_ReadObject(hi2c, tobj);
    return ret;
}
