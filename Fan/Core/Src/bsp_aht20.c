/**
 * ============================================================================
 *  AHT20 温湿度传感器驱动实现
 *  通信协议: I2C, 地址 0x38
 *  供电: 3.3V
 *  支持非阻塞状态机读取 (AHT20_Poll) 和阻塞读取 (AHT20_Read)
 * ============================================================================
 */
#include "bsp_aht20.h"

/* AHT20 命令 */
#define AHT20_CMD_INIT          0xBE
#define AHT20_CMD_TRIGGER       0xAC
#define AHT20_CMD_SOFTRESET     0xBA

/* 内部: 解析 6 字节原始数据为温湿度 */
static bool AHT20_ParseData(const uint8_t data[6], float *temp, float *humi)
{
    if (data[0] & 0x80) return false; /* Busy */

    uint32_t raw_humi = ((uint32_t)data[1] << 12) |
                        ((uint32_t)data[2] << 4)  |
                        ((uint32_t)data[3] >> 4);

    uint32_t raw_temp = (((uint32_t)data[3] & 0x0F) << 16) |
                        ((uint32_t)data[4] << 8) |
                        ((uint32_t)data[5]);

    *humi = (float)raw_humi / 1048576.0f * 100.0f;
    *temp = (float)raw_temp / 1048576.0f * 200.0f - 50.0f;
    return true;
}

/* 内部: 发送触发测量命令 */
static HAL_StatusTypeDef AHT20_SendTrigger(I2C_HandleTypeDef *hi2c)
{
    uint8_t cmd[3] = {AHT20_CMD_TRIGGER, 0x33, 0x00};
    return HAL_I2C_Master_Transmit(hi2c, AHT20_I2C_ADDR, cmd, 3, 50);
}

/* ======================== 公开接口 ======================== */

HAL_StatusTypeDef AHT20_Init(I2C_HandleTypeDef *hi2c)
{
    uint8_t cmd[3];
    HAL_StatusTypeDef ret;

    HAL_Delay(40);

    cmd[0] = AHT20_CMD_INIT;
    cmd[1] = 0x08;
    cmd[2] = 0x00;
    ret = HAL_I2C_Master_Transmit(hi2c, AHT20_I2C_ADDR, cmd, 3, 100);
    if (ret != HAL_OK) return ret;

    HAL_Delay(10);
    return HAL_OK;
}

void AHT20_Handle_Init(AHT20_Handle_t *h, I2C_HandleTypeDef *hi2c)
{
    h->hi2c = hi2c;
    h->state = AHT20_STATE_IDLE;
    h->trigger_tick = 0;
    h->temperature = 0.0f;
    h->humidity = 0.0f;
    h->fail_count = 0;
    h->data_fresh = false;
}

void AHT20_Poll(AHT20_Handle_t *h)
{
    h->data_fresh = false;

    switch (h->state) {
    case AHT20_STATE_IDLE:
        if (AHT20_SendTrigger(h->hi2c) == HAL_OK) {
            h->state = AHT20_STATE_TRIGGERED;
            h->trigger_tick = HAL_GetTick();
        } else {
            h->state = AHT20_STATE_ERROR;
            if (h->fail_count < AHT20_MAX_FAIL_COUNT) h->fail_count++;
        }
        break;

    case AHT20_STATE_TRIGGERED: {
        uint32_t elapsed = HAL_GetTick() - h->trigger_tick;
        if (elapsed < AHT20_MEASURE_TIME_MS) break;

        uint8_t data[6];
        if (HAL_I2C_Master_Receive(h->hi2c, AHT20_I2C_ADDR, data, 6, 50) == HAL_OK) {
            float t, rh;
            if (AHT20_ParseData(data, &t, &rh)) {
                h->temperature = t;
                h->humidity = rh;
                h->data_fresh = true;
                h->fail_count = 0;
                h->state = AHT20_STATE_READY;
            } else if (elapsed < (AHT20_MEASURE_TIME_MS + AHT20_BUSY_EXTRA_MS)) {
                break; /* 还在 Busy, 继续等 */
            } else {
                h->state = AHT20_STATE_ERROR;
                if (h->fail_count < AHT20_MAX_FAIL_COUNT) h->fail_count++;
            }
        } else {
            h->state = AHT20_STATE_ERROR;
            if (h->fail_count < AHT20_MAX_FAIL_COUNT) h->fail_count++;
        }
        break;
    }

    case AHT20_STATE_READY:
    case AHT20_STATE_ERROR:
        h->state = AHT20_STATE_IDLE;
        break;
    }
}

/* 阻塞式读取 (保留兼容, 初始化阶段可用) */
HAL_StatusTypeDef AHT20_Read(I2C_HandleTypeDef *hi2c, float *temperature, float *humidity)
{
    uint8_t data[6];
    HAL_StatusTypeDef ret;

    ret = AHT20_SendTrigger(hi2c);
    if (ret != HAL_OK) return ret;

    HAL_Delay(80);

    ret = HAL_I2C_Master_Receive(hi2c, AHT20_I2C_ADDR, data, 6, 100);
    if (ret != HAL_OK) return ret;

    if (data[0] & 0x80) {
        HAL_Delay(50);
        ret = HAL_I2C_Master_Receive(hi2c, AHT20_I2C_ADDR, data, 6, 100);
        if (ret != HAL_OK) return ret;
    }

    if (!AHT20_ParseData(data, temperature, humidity)) {
        return HAL_ERROR;
    }
    return HAL_OK;
}
