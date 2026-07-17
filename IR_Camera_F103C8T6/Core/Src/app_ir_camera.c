#include "app_ir_camera.h"

#include <string.h>
#include "gy_mcu90640.h"
#include "st7735.h"
#include "thermal_view.h"
#include "usart.h"

#define APP_NO_FRAME_TIMEOUT_MS  3000U

static int16_t temp_frame[GY_MCU90640_PIXEL_COUNT];
static ThermalView_Stats view_stats;
static uint32_t last_frame_tick;
static uint8_t no_frame_shown;

static void debug_write(const char *text)
{
  (void)HAL_UART_Transmit(&huart1, (uint8_t *)text, (uint16_t)strlen(text), 100U);
}

void App_IRCamera_Init(void)
{
  debug_write("IR camera boot\r\n");
  ST7735_Init();
  ST7735_TestPattern();
  ThermalView_ShowWaiting();
  GY_MCU90640_Init();
  last_frame_tick = HAL_GetTick();
  no_frame_shown = 0U;
  debug_write("GY-MCU90640 receive started\r\n");
}

void App_IRCamera_Task(void)
{
  if (GY_MCU90640_PopFrame(temp_frame) != 0U) {
    ThermalView_RenderFrame(temp_frame, &view_stats);
    last_frame_tick = HAL_GetTick();
    no_frame_shown = 0U;
  } else if ((no_frame_shown == 0U) && ((HAL_GetTick() - last_frame_tick) > APP_NO_FRAME_TIMEOUT_MS)) {
    ThermalView_ShowNoFrame();
    no_frame_shown = 1U;
  }
}

void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
{
  GY_MCU90640_RxEventCallback(huart, Size);
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
  GY_MCU90640_ErrorCallback(huart);
}

void HAL_SPI_TxCpltCallback(SPI_HandleTypeDef *hspi)
{
  ST7735_TxCpltCallback(hspi);
}
