#include "st7735.h"

#include "spi.h"

#define ST7735_X_OFFSET  1U
#define ST7735_Y_OFFSET  2U

#define ST7735_SWRESET   0x01U
#define ST7735_SLPOUT    0x11U
#define ST7735_DISPON    0x29U
#define ST7735_CASET     0x2AU
#define ST7735_RASET     0x2BU
#define ST7735_RAMWR     0x2CU
#define ST7735_MADCTL    0x36U
#define ST7735_COLMOD    0x3AU
#define ST7735_FRMCTR1   0xB1U
#define ST7735_FRMCTR2   0xB2U
#define ST7735_FRMCTR3   0xB3U
#define ST7735_INVCTR    0xB4U
#define ST7735_PWCTR1    0xC0U
#define ST7735_PWCTR2    0xC1U
#define ST7735_PWCTR3    0xC2U
#define ST7735_PWCTR4    0xC3U
#define ST7735_PWCTR5    0xC4U
#define ST7735_VMCTR1    0xC5U
#define ST7735_GMCTRP1   0xE0U
#define ST7735_GMCTRN1   0xE1U

#define MADCTL_MY        0x80U
#define MADCTL_MX        0x40U
#define MADCTL_MV        0x20U
#define MADCTL_BGR       0x08U

static volatile uint8_t st7735_tx_busy = 0U;
static uint8_t fill_line[ST7735_WIDTH * 2U];

static void st7735_select(void)
{
  HAL_GPIO_WritePin(LCD_CS_GPIO_Port, LCD_CS_Pin, GPIO_PIN_RESET);
}

static void st7735_unselect(void)
{
  HAL_GPIO_WritePin(LCD_CS_GPIO_Port, LCD_CS_Pin, GPIO_PIN_SET);
}

static void st7735_command_mode(void)
{
  HAL_GPIO_WritePin(LCD_DC_GPIO_Port, LCD_DC_Pin, GPIO_PIN_RESET);
}

static void st7735_data_mode(void)
{
  HAL_GPIO_WritePin(LCD_DC_GPIO_Port, LCD_DC_Pin, GPIO_PIN_SET);
}

static void st7735_reset(void)
{
  HAL_GPIO_WritePin(LCD_RES_GPIO_Port, LCD_RES_Pin, GPIO_PIN_SET);
  HAL_Delay(10);
  HAL_GPIO_WritePin(LCD_RES_GPIO_Port, LCD_RES_Pin, GPIO_PIN_RESET);
  HAL_Delay(20);
  HAL_GPIO_WritePin(LCD_RES_GPIO_Port, LCD_RES_Pin, GPIO_PIN_SET);
  HAL_Delay(120);
}

static void st7735_write_command(uint8_t cmd)
{
  st7735_select();
  st7735_command_mode();
  (void)HAL_SPI_Transmit(&hspi1, &cmd, 1U, HAL_MAX_DELAY);
  st7735_unselect();
}

static void st7735_write_data(const uint8_t *data, uint16_t len)
{
  if (len == 0U) {
    return;
  }

  st7735_select();
  st7735_data_mode();
  (void)HAL_SPI_Transmit(&hspi1, (uint8_t *)data, len, HAL_MAX_DELAY);
  st7735_unselect();
}

static void st7735_write_reg(uint8_t cmd, const uint8_t *data, uint16_t len)
{
  st7735_write_command(cmd);
  st7735_write_data(data, len);
}

static void st7735_set_window(uint16_t x, uint16_t y, uint16_t w, uint16_t h)
{
  uint16_t x0;
  uint16_t x1;
  uint16_t y0;
  uint16_t y1;
  uint8_t data[4];

  if ((x >= ST7735_WIDTH) || (y >= ST7735_HEIGHT) || (w == 0U) || (h == 0U)) {
    return;
  }
  if ((x + w) > ST7735_WIDTH) {
    w = ST7735_WIDTH - x;
  }
  if ((y + h) > ST7735_HEIGHT) {
    h = ST7735_HEIGHT - y;
  }

  x0 = x + ST7735_X_OFFSET;
  x1 = x0 + w - 1U;
  y0 = y + ST7735_Y_OFFSET;
  y1 = y0 + h - 1U;

  data[0] = (uint8_t)(x0 >> 8);
  data[1] = (uint8_t)x0;
  data[2] = (uint8_t)(x1 >> 8);
  data[3] = (uint8_t)x1;
  st7735_write_reg(ST7735_CASET, data, 4U);

  data[0] = (uint8_t)(y0 >> 8);
  data[1] = (uint8_t)y0;
  data[2] = (uint8_t)(y1 >> 8);
  data[3] = (uint8_t)y1;
  st7735_write_reg(ST7735_RASET, data, 4U);
}

uint16_t ST7735_RGB565(uint8_t r, uint8_t g, uint8_t b)
{
  return (uint16_t)((uint16_t)(r & 0xF8U) << 8) |
         (uint16_t)((uint16_t)(g & 0xFCU) << 3) |
         (uint16_t)(b >> 3);
}

void ST7735_Init(void)
{
  uint8_t data[16];

  st7735_unselect();
  st7735_data_mode();
  HAL_GPIO_WritePin(LCD_BL_GPIO_Port, LCD_BL_Pin, GPIO_PIN_SET);
  st7735_reset();

  st7735_write_command(ST7735_SWRESET);
  HAL_Delay(150);
  st7735_write_command(ST7735_SLPOUT);
  HAL_Delay(150);

  data[0] = 0x01U;
  data[1] = 0x2CU;
  data[2] = 0x2DU;
  st7735_write_reg(ST7735_FRMCTR1, data, 3U);
  st7735_write_reg(ST7735_FRMCTR2, data, 3U);

  data[0] = 0x01U;
  data[1] = 0x2CU;
  data[2] = 0x2DU;
  data[3] = 0x01U;
  data[4] = 0x2CU;
  data[5] = 0x2DU;
  st7735_write_reg(ST7735_FRMCTR3, data, 6U);

  data[0] = 0x07U;
  st7735_write_reg(ST7735_INVCTR, data, 1U);

  data[0] = 0xA2U;
  data[1] = 0x02U;
  data[2] = 0x84U;
  st7735_write_reg(ST7735_PWCTR1, data, 3U);
  data[0] = 0xC5U;
  st7735_write_reg(ST7735_PWCTR2, data, 1U);
  data[0] = 0x0AU;
  data[1] = 0x00U;
  st7735_write_reg(ST7735_PWCTR3, data, 2U);
  data[0] = 0x8AU;
  data[1] = 0x2AU;
  st7735_write_reg(ST7735_PWCTR4, data, 2U);
  data[0] = 0x8AU;
  data[1] = 0xEEU;
  st7735_write_reg(ST7735_PWCTR5, data, 2U);
  data[0] = 0x0EU;
  st7735_write_reg(ST7735_VMCTR1, data, 1U);

  data[0] = (uint8_t)(MADCTL_MV | MADCTL_MX);
  st7735_write_reg(ST7735_MADCTL, data, 1U);
  data[0] = 0x05U;
  st7735_write_reg(ST7735_COLMOD, data, 1U);

  {
    static const uint8_t gamma_pos[16] = {
      0x02U, 0x1CU, 0x07U, 0x12U, 0x37U, 0x32U, 0x29U, 0x2DU,
      0x29U, 0x25U, 0x2BU, 0x39U, 0x00U, 0x01U, 0x03U, 0x10U
    };
    static const uint8_t gamma_neg[16] = {
      0x03U, 0x1DU, 0x07U, 0x06U, 0x2EU, 0x2CU, 0x29U, 0x2DU,
      0x2EU, 0x2EU, 0x37U, 0x3FU, 0x00U, 0x00U, 0x02U, 0x10U
    };
    st7735_write_reg(ST7735_GMCTRP1, gamma_pos, 16U);
    st7735_write_reg(ST7735_GMCTRN1, gamma_neg, 16U);
  }

  st7735_write_command(ST7735_DISPON);
  HAL_Delay(100);
  ST7735_FillScreen(0x0000U);
}

void ST7735_BeginWritePixels(uint16_t x, uint16_t y, uint16_t w, uint16_t h)
{
  ST7735_WaitTransfer();
  st7735_set_window(x, y, w, h);
  st7735_write_command(ST7735_RAMWR);
  st7735_select();
  st7735_data_mode();
}

HAL_StatusTypeDef ST7735_WritePixelsDMA(const uint8_t *data, uint16_t len)
{
  HAL_StatusTypeDef status;

  if ((data == 0) || (len == 0U)) {
    return HAL_ERROR;
  }
  ST7735_WaitTransfer();
  st7735_tx_busy = 1U;
  status = HAL_SPI_Transmit_DMA(&hspi1, (uint8_t *)data, len);
  if (status != HAL_OK) {
    st7735_tx_busy = 0U;
  }
  return status;
}

void ST7735_WaitTransfer(void)
{
  while (st7735_tx_busy != 0U) {
  }
}

void ST7735_EndWritePixels(void)
{
  ST7735_WaitTransfer();
  st7735_unselect();
}

void ST7735_TxCpltCallback(SPI_HandleTypeDef *hspi)
{
  if (hspi == &hspi1) {
    st7735_tx_busy = 0U;
  }
}

void ST7735_FillScreen(uint16_t color)
{
  uint16_t x;
  uint16_t y;

  for (x = 0U; x < ST7735_WIDTH; x++) {
    fill_line[(x * 2U)] = (uint8_t)(color >> 8);
    fill_line[(x * 2U) + 1U] = (uint8_t)color;
  }

  ST7735_BeginWritePixels(0U, 0U, ST7735_WIDTH, ST7735_HEIGHT);
  for (y = 0U; y < ST7735_HEIGHT; y++) {
    (void)ST7735_WritePixelsDMA(fill_line, sizeof(fill_line));
  }
  ST7735_EndWritePixels();
}

void ST7735_TestPattern(void)
{
  ST7735_FillScreen(ST7735_RGB565(255U, 0U, 0U));
  HAL_Delay(180);
  ST7735_FillScreen(ST7735_RGB565(0U, 255U, 0U));
  HAL_Delay(180);
  ST7735_FillScreen(ST7735_RGB565(0U, 0U, 255U));
  HAL_Delay(180);
  ST7735_FillScreen(0x0000U);
}
