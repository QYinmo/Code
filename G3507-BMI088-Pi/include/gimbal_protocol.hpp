#pragma once

#include <array>
#include <cstddef>
#include <cstdint>

namespace gimbal {

constexpr std::uint16_t kPacketHeader = 0xA55AU;
constexpr std::size_t kPacketLength = 10U;

enum class PacketMode : std::uint8_t {
    stop = 0U,
    rate_control = 1U,
    attitude = 2U,
    fault = 3U
};

struct GimbalPacketPayload {
    std::uint8_t sequence{0U};
    PacketMode mode{PacketMode::stop};
    // 中性字段：rate_control 时单位为 degree/s，attitude 时单位为 degree。
    // 调用者必须先检查 mode，或使用下面的安全 accessor。
    float yaw_value{0.0F};
    float pitch_value{0.0F};
};

struct RateControlValues {
    float yaw_rate_dps{0.0F};
    float pitch_rate_dps{0.0F};
};

struct AttitudeValues {
    float yaw_deg{0.0F};
    float pitch_deg{0.0F};
};

std::uint16_t crc16Modbus(const std::uint8_t* data, std::size_t length) noexcept;
std::int16_t encodeRate(float rate_dps) noexcept;
std::int16_t encodeAngle(float angle_deg) noexcept;
GimbalPacketPayload makeStopPayload(std::uint8_t sequence = 0U) noexcept;
GimbalPacketPayload makeFaultPayload(std::uint8_t sequence = 0U) noexcept;
GimbalPacketPayload makeRateControlPayload(std::uint8_t sequence, float yaw_rate_dps,
                                           float pitch_rate_dps) noexcept;
GimbalPacketPayload makeAttitudePayload(std::uint8_t sequence, float yaw_deg,
                                        float pitch_deg) noexcept;
bool getRateControlValues(const GimbalPacketPayload& payload,
                          RateControlValues& values) noexcept;
bool getAttitudeValues(const GimbalPacketPayload& payload,
                       AttitudeValues& values) noexcept;
std::array<std::uint8_t, kPacketLength>
serializePacket(const GimbalPacketPayload& payload) noexcept;

} // namespace gimbal
