#include "thermal_view.h"

#include <string.h>
#include "st7735.h"

#define HEATMAP_W       160U
#define HEATMAP_H       120U
#define SCALE           5U
#define STATUS_H        8U
#define FONT_W          5U
#define FONT_H          7U
#define THERMAL_MIRROR_X  1U
#define THERMAL_FILTER_ENABLE  1U
#define STATUS_REFRESH_MS      1000U
#define STATUS_TEMP_STEP       25

static uint8_t heat_block[HEATMAP_W * SCALE * 2U];
static uint8_t status_block[ST7735_WIDTH * STATUS_H * 2U];
static int16_t filtered_frame[GY_MCU90640_PIXEL_COUNT];
static uint16_t color_lut[256];
static ThermalView_Stats last_status_stats;
static uint32_t last_status_tick = 0U;
static int16_t display_min_centi = 0;
static int16_t display_max_centi = 0;
static uint8_t filter_ready = 0U;
static uint8_t status_ready = 0U;
static uint8_t display_range_ready = 0U;

static uint16_t thermal_color_from_index(uint8_t index)
{
  uint8_t r;
  uint8_t g;
  uint8_t b;
  uint8_t phase;
  uint8_t t;
  uint16_t pos;

  pos = (uint16_t)index * 4U;
  phase = (uint8_t)(pos / 256U);
  t = (uint8_t)(pos & 0xFFU);
  r = 0U;
  g = 0U;
  b = 0U;

  switch (phase) {
    case 0U:
      r = 0U;
      g = t;
      b = 255U;
      break;
    case 1U:
      r = 0U;
      g = 255U;
      b = (uint8_t)(255U - t);
      break;
    case 2U:
      r = t;
      g = 255U;
      b = 0U;
      break;
    default:
      r = 255U;
      g = (uint8_t)(255U - t);
      b = 0U;
      break;
  }

  return ST7735_RGB565(r, g, b);
}

static void build_color_lut(void)
{
  uint16_t i;

  for (i = 0U; i < 256U; i++) {
    color_lut[i] = thermal_color_from_index((uint8_t)i);
  }
}

static void put_pixel(uint8_t *buf, uint16_t width, uint16_t x, uint16_t y, uint16_t color)
{
  uint32_t idx;

  idx = ((uint32_t)y * width + x) * 2UL;
  buf[idx] = (uint8_t)(color >> 8);
  buf[idx + 1UL] = (uint8_t)color;
}

static void fill_status_bg(uint16_t color)
{
  uint16_t i;

  for (i = 0U; i < (ST7735_WIDTH * STATUS_H); i++) {
    status_block[i * 2U] = (uint8_t)(color >> 8);
    status_block[i * 2U + 1U] = (uint8_t)color;
  }
}

static uint8_t font5x7(char c, uint8_t col)
{
  static const uint8_t digits[10][5] = {
    {0x3EU, 0x51U, 0x49U, 0x45U, 0x3EU},
    {0x00U, 0x42U, 0x7FU, 0x40U, 0x00U},
    {0x42U, 0x61U, 0x51U, 0x49U, 0x46U},
    {0x21U, 0x41U, 0x45U, 0x4BU, 0x31U},
    {0x18U, 0x14U, 0x12U, 0x7FU, 0x10U},
    {0x27U, 0x45U, 0x45U, 0x45U, 0x39U},
    {0x3CU, 0x4AU, 0x49U, 0x49U, 0x30U},
    {0x01U, 0x71U, 0x09U, 0x05U, 0x03U},
    {0x36U, 0x49U, 0x49U, 0x49U, 0x36U},
    {0x06U, 0x49U, 0x49U, 0x29U, 0x1EU}
  };
  static const uint8_t letter_c[5] = {0x3EU, 0x41U, 0x41U, 0x41U, 0x22U};
  static const uint8_t letter_a[5] = {0x7EU, 0x11U, 0x11U, 0x11U, 0x7EU};
  static const uint8_t letter_e[5] = {0x7FU, 0x49U, 0x49U, 0x49U, 0x41U};
  static const uint8_t letter_f[5] = {0x7FU, 0x09U, 0x09U, 0x09U, 0x01U};
  static const uint8_t letter_l[5] = {0x7FU, 0x40U, 0x40U, 0x40U, 0x40U};
  static const uint8_t letter_h[5] = {0x7FU, 0x08U, 0x08U, 0x08U, 0x7FU};
  static const uint8_t letter_m[5] = {0x7FU, 0x02U, 0x0CU, 0x02U, 0x7FU};
  static const uint8_t letter_n[5] = {0x7FU, 0x04U, 0x08U, 0x10U, 0x7FU};
  static const uint8_t letter_o[5] = {0x3EU, 0x41U, 0x41U, 0x41U, 0x3EU};
  static const uint8_t letter_r[5] = {0x7FU, 0x09U, 0x19U, 0x29U, 0x46U};
  static const uint8_t colon[5] = {0x00U, 0x36U, 0x36U, 0x00U, 0x00U};
  static const uint8_t dot[5] = {0x00U, 0x60U, 0x60U, 0x00U, 0x00U};
  static const uint8_t minus[5] = {0x08U, 0x08U, 0x08U, 0x08U, 0x08U};
  const uint8_t *p;

  p = 0;
  if ((c >= '0') && (c <= '9')) {
    p = digits[(uint8_t)(c - '0')];
  } else if (c == 'A') {
    p = letter_a;
  } else if (c == 'C') {
    p = letter_c;
  } else if (c == 'E') {
    p = letter_e;
  } else if (c == 'F') {
    p = letter_f;
  } else if (c == 'H') {
    p = letter_h;
  } else if (c == 'L') {
    p = letter_l;
  } else if (c == 'M') {
    p = letter_m;
  } else if (c == 'N') {
    p = letter_n;
  } else if (c == 'O') {
    p = letter_o;
  } else if (c == 'R') {
    p = letter_r;
  } else if (c == ':') {
    p = colon;
  } else if (c == '.') {
    p = dot;
  } else if (c == '-') {
    p = minus;
  }

  if (p == 0) {
    return 0U;
  }
  return p[col];
}

static void draw_char(uint16_t x, uint16_t y, char c, uint16_t fg)
{
  uint8_t col;
  uint8_t row;
  uint8_t bits;

  for (col = 0U; col < FONT_W; col++) {
    bits = font5x7(c, col);
    for (row = 0U; row < FONT_H; row++) {
      if ((bits & (1U << row)) != 0U) {
        put_pixel(status_block, ST7735_WIDTH, (uint16_t)(x + col), (uint16_t)(y + row), fg);
      }
    }
  }
}

static uint16_t draw_text(uint16_t x, const char *text, uint16_t fg)
{
  while ((*text != '\0') && ((x + FONT_W) < ST7735_WIDTH)) {
    draw_char(x, 0U, *text, fg);
    x = (uint16_t)(x + 6U);
    text++;
  }
  return x;
}

static void append_char(char *s, uint8_t *pos, char c)
{
  if (*pos < 63U) {
    s[*pos] = c;
    (*pos)++;
    s[*pos] = '\0';
  }
}

static void append_uint(char *s, uint8_t *pos, uint16_t value)
{
  char tmp[5];
  uint8_t n;

  n = 0U;
  do {
    tmp[n] = (char)('0' + (value % 10U));
    value /= 10U;
    n++;
  } while ((value != 0U) && (n < sizeof(tmp)));

  while (n > 0U) {
    n--;
    append_char(s, pos, tmp[n]);
  }
}

static void append_temp(char *s, uint8_t *pos, int16_t centi)
{
  uint16_t abs_v;

  if (centi < 0) {
    append_char(s, pos, '-');
    abs_v = (uint16_t)(-centi);
  } else {
    abs_v = (uint16_t)centi;
  }
  append_uint(s, pos, (uint16_t)(abs_v / 100U));
  append_char(s, pos, '.');
  append_uint(s, pos, (uint16_t)((abs_v / 10U) % 10U));
}

static uint16_t abs_diff_i16(int16_t a, int16_t b)
{
  int32_t diff;

  diff = (int32_t)a - (int32_t)b;
  if (diff < 0L) {
    diff = -diff;
  }
  return (uint16_t)diff;
}

static uint8_t status_changed(const ThermalView_Stats *stats)
{
  if (status_ready == 0U) {
    return 1U;
  }
  if (abs_diff_i16(stats->center_centi, last_status_stats.center_centi) >= STATUS_TEMP_STEP) {
    return 1U;
  }
  if (abs_diff_i16(stats->min_centi, last_status_stats.min_centi) >= STATUS_TEMP_STEP) {
    return 1U;
  }
  if (abs_diff_i16(stats->max_centi, last_status_stats.max_centi) >= STATUS_TEMP_STEP) {
    return 1U;
  }
  return 0U;
}

static void render_status_now(const ThermalView_Stats *stats)
{
  char text[64];
  uint8_t pos;

  pos = 0U;
  text[0] = '\0';
  append_char(text, &pos, 'C');
  append_char(text, &pos, ':');
  append_temp(text, &pos, stats->center_centi);
  append_char(text, &pos, ' ');
  append_char(text, &pos, 'L');
  append_char(text, &pos, ':');
  append_temp(text, &pos, stats->min_centi);
  append_char(text, &pos, ' ');
  append_char(text, &pos, 'H');
  append_char(text, &pos, ':');
  append_temp(text, &pos, stats->max_centi);

  fill_status_bg(0x0000U);
  (void)draw_text(1U, text, 0xFFFFU);
  ST7735_BeginWritePixels(0U, HEATMAP_H, ST7735_WIDTH, STATUS_H);
  (void)ST7735_WritePixelsDMA(status_block, sizeof(status_block));
  ST7735_EndWritePixels();
  last_status_stats = *stats;
  last_status_tick = HAL_GetTick();
  status_ready = 1U;
}

static void render_status_if_needed(const ThermalView_Stats *stats, uint8_t force)
{
  if ((force != 0U) ||
      (status_changed(stats) != 0U) ||
      ((HAL_GetTick() - last_status_tick) >= STATUS_REFRESH_MS)) {
    render_status_now(stats);
  }
}

static void calc_stats(const int16_t *temp_centi, ThermalView_Stats *stats)
{
  uint16_t i;
  int32_t center_sum;

  stats->min_centi = temp_centi[0];
  stats->max_centi = temp_centi[0];
  for (i = 1U; i < GY_MCU90640_PIXEL_COUNT; i++) {
    if (temp_centi[i] < stats->min_centi) {
      stats->min_centi = temp_centi[i];
    }
    if (temp_centi[i] > stats->max_centi) {
      stats->max_centi = temp_centi[i];
    }
  }

  center_sum = (int32_t)temp_centi[(11U * 32U) + 15U] +
               (int32_t)temp_centi[(11U * 32U) + 16U] +
               (int32_t)temp_centi[(12U * 32U) + 15U] +
               (int32_t)temp_centi[(12U * 32U) + 16U];
  stats->center_centi = (int16_t)(center_sum / 4L);

  if ((int16_t)(stats->max_centi - stats->min_centi) < 200) {
    stats->min_centi = (int16_t)(stats->center_centi - 100);
    stats->max_centi = (int16_t)(stats->center_centi + 100);
  }
}

static const int16_t *prepare_filtered_frame(const int16_t *temp_centi)
{
  uint16_t i;
  int32_t value;

#if THERMAL_FILTER_ENABLE
  if (filter_ready == 0U) {
    memcpy(filtered_frame, temp_centi, sizeof(filtered_frame));
    filter_ready = 1U;
  } else {
    for (i = 0U; i < GY_MCU90640_PIXEL_COUNT; i++) {
      value = ((int32_t)filtered_frame[i] * 3L) + (int32_t)temp_centi[i];
      filtered_frame[i] = (int16_t)(value / 4L);
    }
  }
  return filtered_frame;
#else
  (void)i;
  (void)value;
  return temp_centi;
#endif
}

static void smooth_display_range(ThermalView_Stats *stats)
{
  int32_t min_v;
  int32_t max_v;

  if (display_range_ready == 0U) {
    display_min_centi = stats->min_centi;
    display_max_centi = stats->max_centi;
    display_range_ready = 1U;
  } else {
    min_v = ((int32_t)display_min_centi * 3L) + (int32_t)stats->min_centi;
    max_v = ((int32_t)display_max_centi * 3L) + (int32_t)stats->max_centi;
    display_min_centi = (int16_t)(min_v / 4L);
    display_max_centi = (int16_t)(max_v / 4L);
  }

  if ((int16_t)(display_max_centi - display_min_centi) < 200) {
    display_min_centi = (int16_t)(stats->center_centi - 100);
    display_max_centi = (int16_t)(stats->center_centi + 100);
  }
  stats->min_centi = display_min_centi;
  stats->max_centi = display_max_centi;
}

static uint8_t temp_to_color_index(int16_t value, int16_t min_v, int16_t max_v)
{
  int32_t span;
  int32_t pos;

  span = (int32_t)max_v - (int32_t)min_v;
  if (span < 1L) {
    span = 1L;
  }

  pos = (((int32_t)value - (int32_t)min_v) * 255L) / span;
  if (pos < 0L) {
    return 0U;
  }
  if (pos > 255L) {
    return 255U;
  }
  return (uint8_t)pos;
}

void ThermalView_ShowWaiting(void)
{
  ThermalView_Stats stats;

  filter_ready = 0U;
  status_ready = 0U;
  display_range_ready = 0U;
  ST7735_FillScreen(0x0000U);
  stats.min_centi = 0;
  stats.max_centi = 0;
  stats.center_centi = 0;
  render_status_if_needed(&stats, 1U);
}

void ThermalView_ShowNoFrame(void)
{
  fill_status_bg(0x0000U);
  (void)draw_text(1U, "NO FRAME", 0xF800U);
  ST7735_FillScreen(0x0000U);
  ST7735_BeginWritePixels(0U, HEATMAP_H, ST7735_WIDTH, STATUS_H);
  (void)ST7735_WritePixelsDMA(status_block, sizeof(status_block));
  ST7735_EndWritePixels();
}

void ThermalView_RenderFrame(const int16_t *temp_centi, ThermalView_Stats *stats)
{
  ThermalView_Stats local_stats;
  const int16_t *render_frame;
  uint16_t sy;
  uint16_t sx;
  uint16_t ry;
  uint16_t rx;
  uint16_t dst_x;
  uint16_t src_x;
  uint16_t color;
  uint8_t color_index;
  uint8_t *p;

  if ((temp_centi == 0) || (stats == 0)) {
    return;
  }

  render_frame = prepare_filtered_frame(temp_centi);
  calc_stats(render_frame, &local_stats);
  smooth_display_range(&local_stats);
  build_color_lut();

  ST7735_BeginWritePixels(0U, 0U, HEATMAP_W, HEATMAP_H);
  for (sy = 0U; sy < GY_MCU90640_HEIGHT; sy++) {
    for (sx = 0U; sx < GY_MCU90640_WIDTH; sx++) {
#if THERMAL_MIRROR_X
      src_x = (uint16_t)(GY_MCU90640_WIDTH - 1U - sx);
#else
      src_x = sx;
#endif
      color_index = temp_to_color_index(render_frame[(sy * GY_MCU90640_WIDTH) + src_x],
                                        local_stats.min_centi,
                                        local_stats.max_centi);
      color = color_lut[color_index];
      for (rx = 0U; rx < SCALE; rx++) {
        dst_x = (uint16_t)((sx * SCALE) + rx);
        p = &heat_block[dst_x * 2U];
        p[0] = (uint8_t)(color >> 8);
        p[1] = (uint8_t)color;
      }
    }
    for (ry = 1U; ry < SCALE; ry++) {
      memcpy(&heat_block[(uint16_t)(ry * HEATMAP_W * 2U)], heat_block, HEATMAP_W * 2U);
    }
    (void)ST7735_WritePixelsDMA(heat_block, sizeof(heat_block));
    ST7735_WaitTransfer();
  }
  ST7735_EndWritePixels();

  *stats = local_stats;
  render_status_if_needed(&local_stats, 0U);
}
