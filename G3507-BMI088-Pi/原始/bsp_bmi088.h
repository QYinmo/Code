#ifndef BSP_BMI088_H
#define BSP_BMI088_H

#include "ti_msp_dl_config.h"
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Fixed configuration used by this board driver. */
#define BMI088_ACCEL_RANGE_G       (6.0f)
#define BMI088_GYRO_RANGE_DPS      (500.0f)

typedef enum {
    BMI088_OK = 0,
    BMI088_ERR_ARG = 1,
    BMI088_ERR_SPI_TIMEOUT = 2,
    BMI088_ERR_ACCEL_ID = 3,
    BMI088_ERR_GYRO_ID = 4,
    BMI088_ERR_VERIFY = 5,
    BMI088_ERR_NOT_INITIALIZED = 6,
    BMI088_ERR_INVALID_DATA = 7
} BMI088_Status_t;

typedef struct {
    int16_t acc_x;
    int16_t acc_y;
    int16_t acc_z;
    int16_t gyro_x;
    int16_t gyro_y;
    int16_t gyro_z;
} BMI088_Data_t;

typedef struct {
    float ax; /* g */
    float ay;
    float az;
    float gx; /* degree/s */
    float gy;
    float gz;
    float temperature_c;
} BMI088_RealData_t;

BMI088_Status_t BMI088_Init(void);
BMI088_Status_t BMI088_Read_All_Raw(BMI088_Data_t *data);
BMI088_Status_t BMI088_Get_RealData(BMI088_RealData_t *data);
BMI088_Status_t BMI088_Read_Temperature(float *temperature_c);
void BMI088_Get_Chip_ID(uint8_t *accel_id, uint8_t *gyro_id);
const char *BMI088_Status_String(BMI088_Status_t status);

#ifdef __cplusplus
}
#endif

#endif /* BSP_BMI088_H */
