#include "bsp_bmi088.h"

#include <stdbool.h>
#include <stddef.h>

#define BMI088_SPI_READ_BIT          (0x80U)
#define BMI088_SPI_ADDR_MASK         (0x7FU)
#define BMI088_SPI_TIMEOUT_LOOPS     (100000UL)

#define BMI088_ACC_CHIP_ID_REG       (0x00U)
#define BMI088_ACC_DATA_REG          (0x12U)
#define BMI088_ACC_TEMP_REG          (0x22U)
#define BMI088_ACC_CONF_REG          (0x40U)
#define BMI088_ACC_RANGE_REG         (0x41U)
#define BMI088_ACC_PWR_CONF_REG      (0x7CU)
#define BMI088_ACC_PWR_CTRL_REG      (0x7DU)
#define BMI088_ACC_SOFTRESET_REG     (0x7EU)

#define BMI088_GYRO_CHIP_ID_REG      (0x00U)
#define BMI088_GYRO_DATA_REG         (0x02U)
#define BMI088_GYRO_RANGE_REG        (0x0FU)
#define BMI088_GYRO_BANDWIDTH_REG    (0x10U)
#define BMI088_GYRO_LPM1_REG         (0x11U)
#define BMI088_GYRO_SOFTRESET_REG    (0x14U)

#define BMI088_ACC_CHIP_ID_VALUE     (0x1EU)
#define BMI088_GYRO_CHIP_ID_VALUE    (0x0FU)
#define BMI088_SOFTRESET_COMMAND     (0xB6U)

/* Accelerometer: normal filter, 100 Hz ODR, +/-6 g. */
#define BMI088_ACC_CONF_VALUE        (0xA8U)
#define BMI088_ACC_RANGE_VALUE       (0x01U)
/* Gyroscope: 500 dps, 100 Hz ODR / 12 Hz bandwidth. */
#define BMI088_GYRO_RANGE_VALUE      (0x02U)
#define BMI088_GYRO_BW_VALUE         (0x05U)

#define ACC_CS_LOW()  DL_GPIO_clearPins(BMI088_PORT, BMI088_CS_1_PIN)
#define ACC_CS_HIGH() DL_GPIO_setPins(BMI088_PORT, BMI088_CS_1_PIN)
#define GYR_CS_LOW()  DL_GPIO_clearPins(BMI088_PORT, BMI088_CS_2_PIN)
#define GYR_CS_HIGH() DL_GPIO_setPins(BMI088_PORT, BMI088_CS_2_PIN)

typedef enum {
    BMI088_TARGET_ACCEL = 0,
    BMI088_TARGET_GYRO = 1
} BMI088_Target_t;

static bool s_initialized = false;
static uint8_t s_accel_id = 0U;
static uint8_t s_gyro_id = 0U;

static void bmi088_delay_us(uint32_t us)
{
    delay_cycles((CPUCLK_FREQ / 1000000UL) * us);
}

static void bmi088_select(BMI088_Target_t target, bool selected)
{
    if (target == BMI088_TARGET_ACCEL) {
        if (selected) {
            ACC_CS_LOW();
        } else {
            ACC_CS_HIGH();
        }
    } else {
        if (selected) {
            GYR_CS_LOW();
        } else {
            GYR_CS_HIGH();
        }
    }
}

static void bmi088_flush_rx(void)
{
    while (!DL_SPI_isRXFIFOEmpty(SPI_BMI088_INST)) {
        (void)DL_SPI_receiveData8(SPI_BMI088_INST);
    }
}

static BMI088_Status_t bmi088_spi_byte(uint8_t tx, uint8_t *rx)
{
    uint32_t timeout = BMI088_SPI_TIMEOUT_LOOPS;

    while (DL_SPI_isTXFIFOFull(SPI_BMI088_INST)) {
        if (--timeout == 0U) {
            return BMI088_ERR_SPI_TIMEOUT;
        }
    }
    DL_SPI_transmitData8(SPI_BMI088_INST, tx);

    timeout = BMI088_SPI_TIMEOUT_LOOPS;
    while (DL_SPI_isRXFIFOEmpty(SPI_BMI088_INST)) {
        if (--timeout == 0U) {
            return BMI088_ERR_SPI_TIMEOUT;
        }
    }
    *rx = DL_SPI_receiveData8(SPI_BMI088_INST);
    return BMI088_OK;
}

static BMI088_Status_t bmi088_read_regs(BMI088_Target_t target,
                                         uint8_t reg,
                                         uint8_t *data,
                                         size_t length)
{
    BMI088_Status_t status;
    uint8_t discarded;
    size_t i;

    if ((data == NULL) || (length == 0U)) {
        return BMI088_ERR_ARG;
    }

    bmi088_flush_rx();
    bmi088_select(target, true);
    status = bmi088_spi_byte((uint8_t)(reg | BMI088_SPI_READ_BIT), &discarded);

    /* The accelerometer returns one extra dummy byte on every SPI read. */
    if ((status == BMI088_OK) && (target == BMI088_TARGET_ACCEL)) {
        status = bmi088_spi_byte(0x00U, &discarded);
    }

    for (i = 0U; (i < length) && (status == BMI088_OK); ++i) {
        status = bmi088_spi_byte(0x00U, &data[i]);
    }
    bmi088_select(target, false);
    return status;
}

static BMI088_Status_t bmi088_write_reg(BMI088_Target_t target,
                                         uint8_t reg,
                                         uint8_t value)
{
    BMI088_Status_t status;
    uint8_t discarded;

    bmi088_flush_rx();
    bmi088_select(target, true);
    status = bmi088_spi_byte((uint8_t)(reg & BMI088_SPI_ADDR_MASK), &discarded);
    if (status == BMI088_OK) {
        status = bmi088_spi_byte(value, &discarded);
    }
    bmi088_select(target, false);

    /* Register writes need up to 1 ms while the accelerometer is suspended. */
    bmi088_delay_us(1000U);
    return status;
}

static BMI088_Status_t bmi088_write_verify(BMI088_Target_t target,
                                            uint8_t reg,
                                            uint8_t value,
                                            uint8_t mask)
{
    BMI088_Status_t status;
    uint8_t actual = 0U;

    status = bmi088_write_reg(target, reg, value);
    if (status != BMI088_OK) {
        return status;
    }
    status = bmi088_read_regs(target, reg, &actual, 1U);
    if (status != BMI088_OK) {
        return status;
    }
    return ((actual & mask) == (value & mask)) ? BMI088_OK : BMI088_ERR_VERIFY;
}

static int16_t bmi088_i16_le(const uint8_t *bytes)
{
    return (int16_t)((uint16_t)bytes[0] | ((uint16_t)bytes[1] << 8U));
}

BMI088_Status_t BMI088_Init(void)
{
    BMI088_Status_t status;
    uint8_t ignored = 0U;

    s_initialized = false;
    s_accel_id = 0U;
    s_gyro_id = 0U;
    ACC_CS_HIGH();
    GYR_CS_HIGH();
    bmi088_delay_us(1000U);

    /* The first accelerometer read after power-up selects its SPI interface. */
    status = bmi088_read_regs(BMI088_TARGET_ACCEL, BMI088_ACC_CHIP_ID_REG,
                              &ignored, 1U);
    if (status != BMI088_OK) {
        return status;
    }

    status = bmi088_write_reg(BMI088_TARGET_ACCEL, BMI088_ACC_SOFTRESET_REG,
                              BMI088_SOFTRESET_COMMAND);
    if (status != BMI088_OK) {
        return status;
    }
    bmi088_delay_us(2000U);

    /* Soft reset returns the accelerometer interface to its power-on state. */
    status = bmi088_read_regs(BMI088_TARGET_ACCEL, BMI088_ACC_CHIP_ID_REG,
                              &ignored, 1U);
    if (status != BMI088_OK) {
        return status;
    }

    status = bmi088_write_reg(BMI088_TARGET_GYRO, BMI088_GYRO_SOFTRESET_REG,
                              BMI088_SOFTRESET_COMMAND);
    if (status != BMI088_OK) {
        return status;
    }
    bmi088_delay_us(30000U);

    status = bmi088_read_regs(BMI088_TARGET_ACCEL, BMI088_ACC_CHIP_ID_REG,
                              &s_accel_id, 1U);
    if (status != BMI088_OK) {
        return status;
    }
    if (s_accel_id != BMI088_ACC_CHIP_ID_VALUE) {
        return BMI088_ERR_ACCEL_ID;
    }

    status = bmi088_read_regs(BMI088_TARGET_GYRO, BMI088_GYRO_CHIP_ID_REG,
                              &s_gyro_id, 1U);
    if (status != BMI088_OK) {
        return status;
    }
    if (s_gyro_id != BMI088_GYRO_CHIP_ID_VALUE) {
        return BMI088_ERR_GYRO_ID;
    }

    /* Bosch sequence: PWR_CONF first, then PWR_CTRL after 5 ms. */
    status = bmi088_write_verify(BMI088_TARGET_ACCEL, BMI088_ACC_PWR_CONF_REG,
                                 0x00U, 0xFFU);
    if (status != BMI088_OK) {
        return status;
    }
    bmi088_delay_us(5000U);
    status = bmi088_write_verify(BMI088_TARGET_ACCEL, BMI088_ACC_PWR_CTRL_REG,
                                 0x04U, 0xFFU);
    if (status != BMI088_OK) {
        return status;
    }
    bmi088_delay_us(50000U);

    status = bmi088_write_verify(BMI088_TARGET_GYRO, BMI088_GYRO_LPM1_REG,
                                 0x00U, 0xE0U);
    if (status != BMI088_OK) {
        return status;
    }
    bmi088_delay_us(30000U);

    status = bmi088_write_verify(BMI088_TARGET_ACCEL, BMI088_ACC_CONF_REG,
                                 BMI088_ACC_CONF_VALUE, 0xFFU);
    if (status != BMI088_OK) {
        return status;
    }
    status = bmi088_write_verify(BMI088_TARGET_ACCEL, BMI088_ACC_RANGE_REG,
                                 BMI088_ACC_RANGE_VALUE, 0x03U);
    if (status != BMI088_OK) {
        return status;
    }
    status = bmi088_write_verify(BMI088_TARGET_GYRO, BMI088_GYRO_RANGE_REG,
                                 BMI088_GYRO_RANGE_VALUE, 0x07U);
    if (status != BMI088_OK) {
        return status;
    }
    status = bmi088_write_verify(BMI088_TARGET_GYRO, BMI088_GYRO_BANDWIDTH_REG,
                                 BMI088_GYRO_BW_VALUE, 0x07U);
    if (status != BMI088_OK) {
        return status;
    }

    s_initialized = true;
    return BMI088_OK;
}

BMI088_Status_t BMI088_Read_All_Raw(BMI088_Data_t *data)
{
    BMI088_Status_t status;
    uint8_t bytes[6];

    if (data == NULL) {
        return BMI088_ERR_ARG;
    }
    if (!s_initialized) {
        return BMI088_ERR_NOT_INITIALIZED;
    }

    status = bmi088_read_regs(BMI088_TARGET_ACCEL, BMI088_ACC_DATA_REG,
                              bytes, sizeof(bytes));
    if (status != BMI088_OK) {
        return status;
    }
    data->acc_x = bmi088_i16_le(&bytes[0]);
    data->acc_y = bmi088_i16_le(&bytes[2]);
    data->acc_z = bmi088_i16_le(&bytes[4]);

    status = bmi088_read_regs(BMI088_TARGET_GYRO, BMI088_GYRO_DATA_REG,
                              bytes, sizeof(bytes));
    if (status != BMI088_OK) {
        return status;
    }
    data->gyro_x = bmi088_i16_le(&bytes[0]);
    data->gyro_y = bmi088_i16_le(&bytes[2]);
    data->gyro_z = bmi088_i16_le(&bytes[4]);
    return BMI088_OK;
}

BMI088_Status_t BMI088_Read_Temperature(float *temperature_c)
{
    BMI088_Status_t status;
    uint8_t bytes[2];
    uint16_t raw;
    int16_t signed_raw;

    if (temperature_c == NULL) {
        return BMI088_ERR_ARG;
    }
    if (!s_initialized) {
        return BMI088_ERR_NOT_INITIALIZED;
    }

    status = bmi088_read_regs(BMI088_TARGET_ACCEL, BMI088_ACC_TEMP_REG,
                              bytes, sizeof(bytes));
    if (status != BMI088_OK) {
        return status;
    }
    if (bytes[0] == 0x80U) {
        return BMI088_ERR_INVALID_DATA;
    }

    raw = (uint16_t)(((uint16_t)bytes[0] << 3U) |
                     ((uint16_t)bytes[1] >> 5U));
    signed_raw = (raw > 1023U) ? (int16_t)(raw - 2048U) : (int16_t)raw;
    *temperature_c = (float)signed_raw * 0.125f + 23.0f;
    return BMI088_OK;
}

BMI088_Status_t BMI088_Get_RealData(BMI088_RealData_t *data)
{
    BMI088_Data_t raw;
    BMI088_Status_t status;
    const float acc_scale = BMI088_ACCEL_RANGE_G / 32768.0f;
    const float gyro_scale = BMI088_GYRO_RANGE_DPS / 32768.0f;

    if (data == NULL) {
        return BMI088_ERR_ARG;
    }

    status = BMI088_Read_All_Raw(&raw);
    if (status != BMI088_OK) {
        return status;
    }
    data->ax = (float)raw.acc_x * acc_scale;
    data->ay = (float)raw.acc_y * acc_scale;
    data->az = (float)raw.acc_z * acc_scale;
    data->gx = (float)raw.gyro_x * gyro_scale;
    data->gy = (float)raw.gyro_y * gyro_scale;
    data->gz = (float)raw.gyro_z * gyro_scale;

    return BMI088_Read_Temperature(&data->temperature_c);
}

void BMI088_Get_Chip_ID(uint8_t *accel_id, uint8_t *gyro_id)
{
    if (accel_id != NULL) {
        *accel_id = s_accel_id;
    }
    if (gyro_id != NULL) {
        *gyro_id = s_gyro_id;
    }
}

const char *BMI088_Status_String(BMI088_Status_t status)
{
    switch (status) {
    case BMI088_OK: return "OK";
    case BMI088_ERR_ARG: return "ARG";
    case BMI088_ERR_SPI_TIMEOUT: return "SPI TIMEOUT";
    case BMI088_ERR_ACCEL_ID: return "ACC ID";
    case BMI088_ERR_GYRO_ID: return "GYRO ID";
    case BMI088_ERR_VERIFY: return "REG VERIFY";
    case BMI088_ERR_NOT_INITIALIZED: return "NOT INIT";
    case BMI088_ERR_INVALID_DATA: return "BAD DATA";
    default: return "UNKNOWN";
    }
}
