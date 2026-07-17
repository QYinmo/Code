#ifndef __GY_MCU90640_H
#define __GY_MCU90640_H

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"

#define GY_MCU90640_WIDTH        32U
#define GY_MCU90640_HEIGHT       24U
#define GY_MCU90640_PIXEL_COUNT  (GY_MCU90640_WIDTH * GY_MCU90640_HEIGHT)

typedef struct {
  uint32_t rx_events;
  uint32_t valid_frames;
  uint32_t parse_errors;
  uint32_t length_errors;
  uint32_t checksum_errors;
  uint32_t uart_errors;
  uint32_t stream_overflows;
  uint32_t dropped_bytes;
  uint32_t last_valid_tick;
  int16_t sensor_centi;
  uint16_t last_rx_len;
} GY_MCU90640_Status;

void GY_MCU90640_Init(void);
uint8_t GY_MCU90640_PopFrame(int16_t *temp_centi);
void GY_MCU90640_RxEventCallback(UART_HandleTypeDef *huart, uint16_t size);
void GY_MCU90640_ErrorCallback(UART_HandleTypeDef *huart);
const GY_MCU90640_Status *GY_MCU90640_GetStatus(void);

#ifdef __cplusplus
}
#endif

#endif
