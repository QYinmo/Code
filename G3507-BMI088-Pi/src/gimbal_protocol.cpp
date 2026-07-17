#include "gimbal_protocol.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

namespace gimbal {

std::uint16_t crc16Modbus(const std::uint8_t* data, std::size_t length) noexcept {
    std::uint16_t crc = 0xFFFFU;
    if (data == nullptr) return crc;
    for (std::size_t index = 0U; index < length; ++index) {
        crc ^= data[index];
        for (unsigned bit = 0U; bit < 8U; ++bit) {
            crc = (crc & 1U) != 0U ? static_cast<std::uint16_t>((crc >> 1U) ^ 0xA001U)
                                   : static_cast<std::uint16_t>(crc >> 1U);
        }
    }
    return crc;
}

std::int16_t encodeRate(float rate_dps) noexcept {
    if (!std::isfinite(rate_dps)) return 0;
    constexpr float minimum = static_cast<float>(std::numeric_limits<std::int16_t>::min()) / 100.0F;
    constexpr float maximum = static_cast<float>(std::numeric_limits<std::int16_t>::max()) / 100.0F;
    const float limited = std::clamp(rate_dps, minimum, maximum);
    return static_cast<std::int16_t>(std::lround(limited * 100.0F));
}

std::int16_t encodeAngle(float angle_deg) noexcept {
    if (!std::isfinite(angle_deg)) return 0;
    // 姿态帧仍使用 int16/0.01°；将连续 yaw 折回 [-180°, 180°]，避免长时间运行后饱和。
    const float wrapped = std::remainder(angle_deg, 360.0F);
    return static_cast<std::int16_t>(std::lround(wrapped * 100.0F));
}

GimbalPacketPayload makeStopPayload(std::uint8_t sequence) noexcept {
    return {sequence, PacketMode::stop, 0.0F, 0.0F};
}

GimbalPacketPayload makeFaultPayload(std::uint8_t sequence) noexcept {
    return {sequence, PacketMode::fault, 0.0F, 0.0F};
}

GimbalPacketPayload makeRateControlPayload(std::uint8_t sequence, float yaw_rate_dps,
                                           float pitch_rate_dps) noexcept {
    return {sequence, PacketMode::rate_control, yaw_rate_dps, pitch_rate_dps};
}

GimbalPacketPayload makeAttitudePayload(std::uint8_t sequence, float yaw_deg,
                                        float pitch_deg) noexcept {
    return {sequence, PacketMode::attitude, yaw_deg, pitch_deg};
}

bool getRateControlValues(const GimbalPacketPayload& payload,
                          RateControlValues& values) noexcept {
    if (payload.mode != PacketMode::rate_control) return false;
    values = {payload.yaw_value, payload.pitch_value};
    return true;
}

bool getAttitudeValues(const GimbalPacketPayload& payload,
                       AttitudeValues& values) noexcept {
    if (payload.mode != PacketMode::attitude) return false;
    values = {payload.yaw_value, payload.pitch_value};
    return true;
}

std::array<std::uint8_t, kPacketLength>
serializePacket(const GimbalPacketPayload& payload) noexcept {
    std::array<std::uint8_t, kPacketLength> packet{};
    std::int16_t yaw = 0;
    std::int16_t pitch = 0;
    if (payload.mode == PacketMode::rate_control) {
        yaw = encodeRate(payload.yaw_value);
        pitch = encodeRate(payload.pitch_value);
    } else if (payload.mode == PacketMode::attitude) {
        yaw = encodeAngle(payload.yaw_value);
        pitch = encodeAngle(payload.pitch_value);
    }
    packet[0] = static_cast<std::uint8_t>(kPacketHeader & 0xFFU);
    packet[1] = static_cast<std::uint8_t>((kPacketHeader >> 8U) & 0xFFU);
    packet[2] = payload.sequence;
    packet[3] = static_cast<std::uint8_t>(payload.mode);
    packet[4] = static_cast<std::uint8_t>(static_cast<std::uint16_t>(yaw) & 0xFFU);
    packet[5] = static_cast<std::uint8_t>((static_cast<std::uint16_t>(yaw) >> 8U) & 0xFFU);
    packet[6] = static_cast<std::uint8_t>(static_cast<std::uint16_t>(pitch) & 0xFFU);
    packet[7] = static_cast<std::uint8_t>((static_cast<std::uint16_t>(pitch) >> 8U) & 0xFFU);
    const std::uint16_t crc = crc16Modbus(packet.data(), 8U);
    packet[8] = static_cast<std::uint8_t>(crc & 0xFFU);
    packet[9] = static_cast<std::uint8_t>((crc >> 8U) & 0xFFU);
    return packet;
}

} // namespace gimbal
