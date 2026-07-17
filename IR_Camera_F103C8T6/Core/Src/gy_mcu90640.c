#include "gy_mcu90640.h"

#include <string.h>
#include "usart.h"

#define GY_RX_BUFFER_SIZE       1600U
#define GY_FRAME_HEADER_SIZE    4U
#define GY_TEMP_DATA_SIZE       (GY_MCU90640_PIXEL_COUNT * 2U)
#define GY_SENSOR_TEMP_SIZE     2U
#define GY_CHECKSUM_SIZE        2U
#define GY_PAYLOAD_LENGTH       0x0602U
#define GY_FULL_FRAME_SIZE      (GY_FRAME_HEADER_SIZE + GY_PAYLOAD_LENGTH + GY_CHECKSUM_SIZE)
#define GY_STREAM_BUFFER_SIZE   (GY_FULL_FRAME_SIZE * 2U)
#define GY_HEADER_BYTE          0x5AU
#define GY_UART_BAUDRATE        115200U

static uint8_t rx_dma_buf[GY_RX_BUFFER_SIZE];
static uint8_t stream_buf[GY_STREAM_BUFFER_SIZE];
static uint8_t parse_buf[GY_FULL_FRAME_SIZE];
static volatile uint16_t stream_len = 0U;
static volatile uint8_t frame_ready = 0U;
static GY_MCU90640_Status gy_status = {0};

static void gy_start_receive(void)
{
  (void)HAL_UARTEx_ReceiveToIdle_DMA(&huart2, rx_dma_buf, GY_RX_BUFFER_SIZE);
  if (huart2.hdmarx != 0) {
    __HAL_DMA_DISABLE_IT(huart2.hdmarx, DMA_IT_HT);
  }
}

static void gy_send_command(const uint8_t *cmd, uint16_t len)
{
  (void)HAL_UART_Transmit(&huart2, (uint8_t *)cmd, len, 100U);
}

static void gy_reset_receiver_state(void)
{
  __disable_irq();
  frame_ready = 0U;
  stream_len = 0U;
  __enable_irq();
}

static void gy_consume_stream(uint16_t len)
{
  __disable_irq();
  if (len >= stream_len) {
    stream_len = 0U;
    frame_ready = 0U;
  } else {
    memmove(stream_buf, &stream_buf[len], (uint16_t)(stream_len - len));
    stream_len = (uint16_t)(stream_len - len);
    frame_ready = (stream_len != 0U) ? 1U : 0U;
  }
  __enable_irq();
}

static void gy_configure_uart(void)
{
  (void)HAL_UART_DMAStop(&huart2);
  huart2.Init.BaudRate = GY_UART_BAUDRATE;
  (void)HAL_UART_Init(&huart2);
}

void GY_MCU90640_Init(void)
{
  static const uint8_t cmd_4hz[4] = {0xA5U, 0x25U, 0x03U, 0xCDU};
  static const uint8_t cmd_auto[4] = {0xA5U, 0x35U, 0x02U, 0xDCU};

  gy_reset_receiver_state();
  memset(&gy_status, 0, sizeof(gy_status));

  gy_configure_uart();
  HAL_Delay(20);
  gy_send_command(cmd_4hz, sizeof(cmd_4hz));
  HAL_Delay(50);
  gy_send_command(cmd_auto, sizeof(cmd_auto));
  HAL_Delay(50);
  gy_start_receive();
}

void GY_MCU90640_RxEventCallback(UART_HandleTypeDef *huart, uint16_t size)
{
  uint16_t drop_len;

  if (huart != &huart2) {
    return;
  }

  gy_status.rx_events++;
  gy_status.last_rx_len = size;

  (void)HAL_UART_DMAStop(&huart2);

  if (size > GY_RX_BUFFER_SIZE) {
    size = GY_RX_BUFFER_SIZE;
  }
  if (size > 0U) {
    if ((uint16_t)(stream_len + size) > GY_STREAM_BUFFER_SIZE) {
      drop_len = (uint16_t)((stream_len + size) - GY_STREAM_BUFFER_SIZE);
      gy_status.stream_overflows++;
      gy_status.dropped_bytes += drop_len;
      if (drop_len >= stream_len) {
        stream_len = 0U;
      } else {
        memmove(stream_buf, &stream_buf[drop_len], (uint16_t)(stream_len - drop_len));
        stream_len = (uint16_t)(stream_len - drop_len);
      }
    }
    memcpy(&stream_buf[stream_len], rx_dma_buf, size);
    stream_len = (uint16_t)(stream_len + size);
    frame_ready = 1U;
  }

  gy_start_receive();
}

void GY_MCU90640_ErrorCallback(UART_HandleTypeDef *huart)
{
  if (huart == &huart2) {
    gy_status.uart_errors++;
    (void)HAL_UART_DMAStop(&huart2);
    gy_start_receive();
  }
}

static int16_t gy_read_i16_le(const uint8_t *p)
{
  return (int16_t)((uint16_t)p[0] | ((uint16_t)p[1] << 8));
}

static uint16_t gy_read_u16_le(const uint8_t *p)
{
  return (uint16_t)p[0] | ((uint16_t)p[1] << 8);
}

static uint16_t gy_find_frame_start(const uint8_t *buf, uint16_t len)
{
  uint16_t i;

  if (len < GY_FULL_FRAME_SIZE) {
    return len;
  }
  for (i = 0U; i <= (uint16_t)(len - GY_FULL_FRAME_SIZE); i++) {
    if ((buf[i] == GY_HEADER_BYTE) && (buf[i + 1U] == GY_HEADER_BYTE)) {
      return i;
    }
  }
  return len;
}

uint8_t GY_MCU90640_PopFrame(int16_t *temp_centi)
{
  uint16_t len;
  uint16_t start;
  uint16_t payload_len;
  uint16_t i;
  const uint8_t *data;
  uint16_t received_sum;
  uint16_t calc_sum;
  uint8_t last_byte;

  if ((temp_centi == 0) || (frame_ready == 0U)) {
    return 0U;
  }

  __disable_irq();
  len = stream_len;
  if (len > GY_STREAM_BUFFER_SIZE) {
    len = GY_STREAM_BUFFER_SIZE;
  }
  last_byte = (len > 0U) ? stream_buf[len - 1U] : 0U;
  if (len >= GY_FULL_FRAME_SIZE) {
    start = gy_find_frame_start(stream_buf, len);
    if ((start < len) && ((uint16_t)(len - start) >= GY_FULL_FRAME_SIZE)) {
      memcpy(parse_buf, &stream_buf[start], GY_FULL_FRAME_SIZE);
    }
  } else {
    start = len;
  }
  __enable_irq();

  if (len < GY_FULL_FRAME_SIZE) {
    return 0U;
  }

  if (start >= len) {
    gy_status.parse_errors++;
    if (len > 1U) {
      if (last_byte == GY_HEADER_BYTE) {
        gy_consume_stream((uint16_t)(len - 1U));
      } else {
        gy_consume_stream(len);
      }
    }
    return 0U;
  }
  if ((uint16_t)(len - start) < GY_FULL_FRAME_SIZE) {
    if (start > 0U) {
      gy_consume_stream(start);
    }
    return 0U;
  }

  payload_len = gy_read_u16_le(&parse_buf[2U]);
  if (payload_len != GY_PAYLOAD_LENGTH) {
    gy_status.length_errors++;
    gy_consume_stream((uint16_t)(start + 2U));
    return 0U;
  }

  calc_sum = 0U;
  for (i = 0U; i < (uint16_t)(GY_FRAME_HEADER_SIZE + GY_PAYLOAD_LENGTH); i += 2U) {
    calc_sum = (uint16_t)(calc_sum + gy_read_u16_le(&parse_buf[i]));
  }
  received_sum = gy_read_u16_le(&parse_buf[GY_FRAME_HEADER_SIZE + GY_PAYLOAD_LENGTH]);
  if (calc_sum != received_sum) {
    gy_status.checksum_errors++;
    gy_consume_stream((uint16_t)(start + 2U));
    return 0U;
  }

  data = &parse_buf[GY_FRAME_HEADER_SIZE];
  for (i = 0U; i < GY_MCU90640_PIXEL_COUNT; i++) {
    temp_centi[i] = gy_read_i16_le(&data[i * 2U]);
  }

  gy_status.sensor_centi = gy_read_i16_le(&parse_buf[GY_FRAME_HEADER_SIZE + GY_TEMP_DATA_SIZE]);
  gy_status.last_valid_tick = HAL_GetTick();
  gy_status.valid_frames++;
  gy_consume_stream((uint16_t)(start + GY_FULL_FRAME_SIZE));
  return 1U;
}

const GY_MCU90640_Status *GY_MCU90640_GetStatus(void)
{
  return &gy_status;
}
