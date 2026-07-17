#pragma once

#include "gimbal_protocol.hpp"
#include "imu_types.hpp"
#include "safety_state.hpp"

#include <array>
#include <chrono>
#include <string_view>

namespace gimbal {

enum class OperatorCommand {
    target,
    run,
    stop,
    reset_fault,
    status,
    quit,
    unknown
};

OperatorCommand classifyOperatorCommand(std::string_view name) noexcept;
bool operatorCommandAllowed(bool attitude_uart_mode, OperatorCommand command) noexcept;
bool modeCanEnterRunning(bool attitude_uart_mode) noexcept;

struct PacketDecision {
    bool should_send{false};
    GimbalPacketPayload payload{};
};

struct AttitudePublicationInput {
    bool attitude_uart_mode{false};
    bool calibration_complete{false};
    bool attitude_degraded{false};
    SafetyState safety_state{SafetyState::boot};
    Attitude attitude{};
    double now_s{0.0};
    double timeout_s{0.0};
};

PacketDecision decideAttitudePublication(
    const AttitudePublicationInput& input) noexcept;

PacketDecision decideUartTransmission(
    bool attitude_uart_mode, const GimbalPacketPayload& latest,
    std::chrono::steady_clock::time_point updated,
    std::chrono::steady_clock::time_point now, double timeout_s) noexcept;

struct UartFailurePolicy {
    bool record_failure_exit{true};
    bool request_process_stop{false};
    bool publish_fault_packet{true};
};

UartFailurePolicy uartFailurePolicy(bool attitude_uart_mode) noexcept;

constexpr std::size_t kShutdownPacketCount = 5U;
std::array<GimbalPacketPayload, kShutdownPacketCount>
makeShutdownPayloads() noexcept;

} // namespace gimbal
