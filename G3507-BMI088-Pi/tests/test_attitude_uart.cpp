#include "exit_status.hpp"
#include "run_mode_policy.hpp"

#include <chrono>
#include <cstdint>
#include <iostream>
#include <limits>

namespace {

gimbal::AttitudePublicationInput validInput() {
    gimbal::AttitudePublicationInput input{};
    input.attitude_uart_mode = true;
    input.calibration_complete = true;
    input.attitude_degraded = false;
    input.safety_state = gimbal::SafetyState::stop_ready;
    input.attitude.timestamp_s = 10.0;
    input.attitude.yaw_deg = 12.34F;
    input.attitude.pitch_deg = -5.67F;
    input.attitude.roll_deg = 88.0F;
    input.attitude.valid = true;
    input.now_s = 10.01;
    input.timeout_s = 0.1;
    return input;
}

} // namespace

int main() {
    auto input = validInput();
    input.calibration_complete = false;
    if (gimbal::decideAttitudePublication(input).should_send) {
        std::cerr << "校准完成前发送了姿态帧\n";
        return 1;
    }

    input = validInput();
    input.attitude.valid = false;
    if (gimbal::decideAttitudePublication(input).should_send) {
        std::cerr << "无效姿态仍被发送\n";
        return 1;
    }

    input = validInput();
    const gimbal::PacketDecision first =
        gimbal::decideAttitudePublication(input);
    if (!first.should_send ||
        first.payload.mode != gimbal::PacketMode::attitude) {
        std::cerr << "首次有效姿态未生成 mode 2\n";
        return 1;
    }
    const auto packet = gimbal::serializePacket(first.payload);
    if (packet[0] != 0x5AU || packet[1] != 0xA5U ||
        packet[3] != 0x02U || packet[4] != 0xD2U ||
        packet[5] != 0x04U || packet[6] != 0xC9U ||
        packet[7] != 0xFDU) {
        std::cerr << "姿态 mode 2 的 yaw/pitch 0.01° 编码错误\n";
        return 1;
    }
    const std::uint16_t crc = static_cast<std::uint16_t>(packet[8]) |
                              (static_cast<std::uint16_t>(packet[9]) << 8U);
    if (crc != gimbal::crc16Modbus(packet.data(), 8U)) {
        std::cerr << "姿态模式 CRC 错误\n";
        return 1;
    }
    if (gimbal::encodeAngle(370.0F) != 1000 ||
        gimbal::encodeAngle(-190.0F) != 17000 ||
        gimbal::encodeAngle(540.0F) != -18000) {
        std::cerr << "连续 yaw 未正确折回 [-180°, 180°]\n";
        return 1;
    }

    const auto now = std::chrono::steady_clock::time_point{} +
                     std::chrono::seconds(2);
    const auto stale = gimbal::decideUartTransmission(
        true, first.payload, now - std::chrono::milliseconds(101),
        now, 0.1);
    if (stale.should_send) {
        std::cerr << "姿态数据超时后仍发送旧姿态\n";
        return 1;
    }

    input = validInput();
    input.attitude_degraded = true;
    if (gimbal::decideAttitudePublication(input).should_send) {
        std::cerr << "姿态降级后仍发送旧姿态\n";
        return 1;
    }
    input = validInput();
    input.attitude.yaw_deg = std::numeric_limits<float>::quiet_NaN();
    if (gimbal::decideAttitudePublication(input).should_send) {
        std::cerr << "非有限姿态仍被发送\n";
        return 1;
    }

    if (gimbal::modeCanEnterRunning(true) ||
        first.payload.mode == gimbal::PacketMode::rate_control) {
        std::cerr << "姿态模式进入 RUNNING 或生成 RATE_CONTROL\n";
        return 1;
    }

    using gimbal::OperatorCommand;
    if (gimbal::operatorCommandAllowed(true, OperatorCommand::run) ||
        gimbal::operatorCommandAllowed(true, OperatorCommand::stop) ||
        gimbal::operatorCommandAllowed(true, OperatorCommand::target) ||
        gimbal::operatorCommandAllowed(true, OperatorCommand::reset_fault) ||
        !gimbal::operatorCommandAllowed(true, OperatorCommand::status) ||
        !gimbal::operatorCommandAllowed(true, OperatorCommand::quit) ||
        gimbal::classifyOperatorCommand("reset-fault") !=
            OperatorCommand::reset_fault ||
        gimbal::classifyOperatorCommand("exit") != OperatorCommand::quit) {
        std::cerr << "姿态模式终端命令权限错误\n";
        return 1;
    }

    const gimbal::UartFailurePolicy failure =
        gimbal::uartFailurePolicy(true);
    gimbal::ExitStatus exit_status;
    if (failure.record_failure_exit) {
        exit_status.record(gimbal::ExitReason::runtime_failure);
    }
    if (!failure.request_process_stop || failure.publish_fault_packet ||
        exit_status.code() != 2) {
        std::cerr << "姿态模式 UART 故障未记录失败并请求安全退出\n";
        return 1;
    }

    const auto shutdown = gimbal::makeShutdownPayloads();
    if (shutdown.size() != gimbal::kShutdownPacketCount) return 1;
    for (std::size_t index = 0U; index < shutdown.size(); ++index) {
        const auto stop = gimbal::serializePacket(shutdown[index]);
        if (shutdown[index].sequence != index ||
            shutdown[index].mode != gimbal::PacketMode::stop ||
            stop[3] != 0U || stop[4] != 0U || stop[5] != 0U ||
            stop[6] != 0U || stop[7] != 0U) {
            std::cerr << "姿态模式退出阶段不是 5 个零值 STOP\n";
            return 1;
        }
    }

    gimbal::RateControlValues rate{};
    gimbal::AttitudeValues angles{};
    if (gimbal::getRateControlValues(first.payload, rate) ||
        !gimbal::getAttitudeValues(first.payload, angles)) {
        std::cerr << "mode 不匹配时安全 accessor 未拒绝\n";
        return 1;
    }
    return 0;
}
