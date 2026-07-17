#ifndef __ST7735_H
#define __ST7735_H

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"

#define ST7735_WIDTH   160U
#define ST7735_HEIGHT  128U

void ST7735_Init(void);
void ST7735_TestPattern(void);
void ST7735_FillScreen(uint16_t color);
void ST7735_BeginWritePixels(uint16_t x, uint16_t y, uint16_t w, uint16_t h);
HAL_StatusTypeDef ST7735_WritePixelsDMA(const uint8_t *data, uint16_t len);
void ST7735_EndWritePixels(void);
void ST7735_WaitTransfer(void);
void ST7735_TxCpltCallback(SPI_HandleTypeDef *hspi);
uint16_t ST7735_RGB565(uint8_t r, uint8_t g, uint8_t b);

#ifdef __cplusplus
}
#endif

#endif
