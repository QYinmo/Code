/**
 * ============================================================================
 *  OLED SSD1306 显示驱动 (I2C, 128x64)
 * ============================================================================
 */
#ifndef __BSP_OLED_H
#define __BSP_OLED_H

#include "bsp_config.h"

#define OLED_I2C_ADDR       (0x3C << 1)  /* 0x78 */
#define OLED_WIDTH          128
#define OLED_HEIGHT         64

/**
 * @brief 初始化 OLED 显示屏
 */
void OLED_Init(I2C_HandleTypeDef *hi2c);

/**
 * @brief 清屏
 */
void OLED_Clear(void);

/**
 * @brief 在指定位置显示字符串 (6x8 字体)
 * @param x 列 (0~127)
 * @param y 页 (0~7, 每页 8 像素)
 * @param str 字符串
 */
void OLED_ShowString(uint8_t x, uint8_t y, const char *str);

/**
 * @brief 刷新显示缓存到屏幕
 */
void OLED_Refresh(void);

/**
 * @brief 在指定位置画一个像素点
 */
void OLED_DrawPoint(uint8_t x, uint8_t y, uint8_t color);

/**
 * @brief 反显/正显切换
 */
void OLED_SetInverse(bool inverse);

#endif /* __BSP_OLED_H */
