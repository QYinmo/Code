#ifndef GIMBAL_PROTOCOL_H
#define GIMBAL_PROTOCOL_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define GIMBAL_PACKET_LENGTH (10U)
#define GIMBAL_PACKET_HEADER (0xA55AU)

typedef enum {
    GIMBAL_MODE_STOP = 0U,
    GIMBAL_MODE_RATE_CONTROL = 1U,
    GIMBAL_MODE_ATTITUDE = 2U,
    GIMBAL_MODE_FAULT = 3U
} GimbalMode;

typedef struct {
    uint8_t sequence;
    GimbalMode mode;
    /* 中性原始字段；必须按 mode 使用安全 accessor 解释。 */
    int16_t yaw_value;
    int16_t pitch_value;
} GimbalDecodedPacket;

/* 保持原 GimbalProtocol_Parse() 调用形式兼容。 */
typedef GimbalDecodedPacket GimbalDecodedCommand;

typedef struct {
    int16_t yaw_rate_cdeg_s;
    int16_t pitch_rate_cdeg_s;
} GimbalRateControlValues;

typedef struct {
    int16_t yaw_cdeg;
    int16_t pitch_cdeg;
} GimbalAttitudeValues;

/* CRC-16/MODBUS: poly 0xA001, init 0xFFFF, reflected, xorout 0x0000. */
uint16_t GimbalProtocol_Crc16Modbus(const uint8_t *data, size_t length);
uint16_t GimbalProtocol_ReadU16Le(const uint8_t *bytes);
int16_t GimbalProtocol_ReadI16Le(const uint8_t *bytes);
bool GimbalProtocol_IsValid(const uint8_t packet[GIMBAL_PACKET_LENGTH]);
bool GimbalProtocol_Parse(const uint8_t packet[GIMBAL_PACKET_LENGTH],
                          GimbalDecodedCommand *command);
bool GimbalProtocol_GetRateControl(const GimbalDecodedPacket *packet,
                                   GimbalRateControlValues *values);
bool GimbalProtocol_GetAttitude(const GimbalDecodedPacket *packet,
                                GimbalAttitudeValues *values);

#ifdef __cplusplus
}
#endif

#endif /* GIMBAL_PROTOCOL_H */
