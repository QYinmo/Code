#include "gimbal_protocol.hpp"

#include <cmath>
#include <cstdint>
#include <iostream>
#include <limits>

int main() {
    const auto payload =
        gimbal::makeRateControlPayload(0x12U, 25.36F, -8.50F);
    const auto packet = gimbal::serializePacket(payload);
    if (packet[0] != 0x5AU || packet[1] != 0xA5U || packet[2] != 0x12U || packet[3] != 1U ||
        packet[4] != 0xE8U || packet[5] != 0x09U || packet[6] != 0xAEU || packet[7] != 0xFCU ||
        packet[8] != 0x97U || packet[9] != 0x73U) {
        std::cerr << "完整 10 字节包或正负 int16 小端序列化失败\n";
        return 1;
    }
    const std::uint16_t packet_crc = static_cast<std::uint16_t>(packet[8]) |
                                     (static_cast<std::uint16_t>(packet[9]) << 8U);
    if (packet_crc != gimbal::crc16Modbus(packet.data(), 8U)) {
        std::cerr << "数据包 CRC 字段失败\n";
        return 1;
    }
    if (gimbal::encodeRate(1000.0F) != std::numeric_limits<std::int16_t>::max() ||
        gimbal::encodeRate(-1000.0F) != std::numeric_limits<std::int16_t>::min()) {
        std::cerr << "角速度超范围限幅失败\n";
        return 1;
    }
    if (gimbal::encodeRate(std::numeric_limits<float>::quiet_NaN()) != 0 ||
        gimbal::encodeRate(std::numeric_limits<float>::infinity()) != 0) {
        std::cerr << "NaN/Inf 安全处理失败\n";
        return 1;
    }
    gimbal::RateControlValues rate{};
    gimbal::AttitudeValues attitude{};
    if (!gimbal::getRateControlValues(payload, rate) ||
        gimbal::getAttitudeValues(payload, attitude) ||
        std::fabs(rate.yaw_rate_dps - 25.36F) > 1.0e-6F ||
        std::fabs(rate.pitch_rate_dps + 8.50F) > 1.0e-6F) {
        std::cerr << "按 mode 读取角速度的安全 accessor 失败\n";
        return 1;
    }
    const auto stop = gimbal::serializePacket(
        {0U, gimbal::PacketMode::stop, 99.0F, -99.0F});
    if (stop[4] != 0U || stop[5] != 0U || stop[6] != 0U || stop[7] != 0U) {
        std::cerr << "STOP 非零 payload 未被强制清零\n";
        return 1;
    }
    return 0;
}
