#ifndef __THERMAL_VIEW_H
#define __THERMAL_VIEW_H

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"
#include "gy_mcu90640.h"

typedef struct {
  int16_t min_centi;
  int16_t max_centi;
  int16_t center_centi;
} ThermalView_Stats;

void ThermalView_ShowWaiting(void);
void ThermalView_ShowNoFrame(void);
void ThermalView_RenderFrame(const int16_t *temp_centi, ThermalView_Stats *stats);

#ifdef __cplusplus
}
#endif

#endif
