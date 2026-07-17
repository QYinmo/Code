#include "gimbal_protocol.h"

uint16_t GimbalProtocol_Crc16Modbus(const uint8_t *data, size_t length)
{
    uint16_t crc = 0xFFFFU;
    size_t index;
    uint8_t bit;
    if (data == NULL) {
        return crc;
    }
    for (index = 0U; index < length; ++index) {
        crc ^= data[index];
        for (bit = 0U; bit < 8U; ++bit) {
            if ((crc & 1U) != 0U) {
                crc = (uint16_t)((crc >> 1U) ^ 0xA001U);
            } else {
                crc = (uint16_t)(crc >> 1U);
            }
        }
    }
    return crc;
}

uint16_t GimbalProtocol_ReadU16Le(const uint8_t *bytes)
{
    if (bytes == NULL) {
        return 0U;
    }
    return (uint16_t)((uint16_t)bytes[0] | ((uint16_t)bytes[1] << 8U));
}

int16_t GimbalProtocol_ReadI16Le(const uint8_t *bytes)
{
    return (int16_t)GimbalProtocol_ReadU16Le(bytes);
}

bool GimbalProtocol_IsValid(const uint8_t packet[GIMBAL_PACKET_LENGTH])
{
    uint16_t received_crc;
    if (packet == NULL || GimbalProtocol_ReadU16Le(&packet[0]) != GIMBAL_PACKET_HEADER ||
        packet[3] > (uint8_t)GIMBAL_MODE_FAULT) {
        return false;
    }
    received_crc = GimbalProtocol_ReadU16Le(&packet[8]);
    return received_crc == GimbalProtocol_Crc16Modbus(packet, 8U);
}

bool GimbalProtocol_Parse(const uint8_t packet[GIMBAL_PACKET_LENGTH],
                          GimbalDecodedCommand *command)
{
    if (command == NULL || !GimbalProtocol_IsValid(packet)) {
        return false;
    }
    command->sequence = packet[2];
    command->mode = (GimbalMode)packet[3];
    command->yaw_value = GimbalProtocol_ReadI16Le(&packet[4]);
    command->pitch_value = GimbalProtocol_ReadI16Le(&packet[6]);
    return true;
}

bool GimbalProtocol_GetRateControl(const GimbalDecodedPacket *packet,
                                   GimbalRateControlValues *values)
{
    if (packet == NULL || values == NULL ||
        packet->mode != GIMBAL_MODE_RATE_CONTROL) {
        return false;
    }
    values->yaw_rate_cdeg_s = packet->yaw_value;
    values->pitch_rate_cdeg_s = packet->pitch_value;
    return true;
}

bool GimbalProtocol_GetAttitude(const GimbalDecodedPacket *packet,
                                GimbalAttitudeValues *values)
{
    if (packet == NULL || values == NULL || packet->mode != GIMBAL_MODE_ATTITUDE) {
        return false;
    }
    values->yaw_cdeg = packet->yaw_value;
    values->pitch_cdeg = packet->pitch_value;
    return true;
}
