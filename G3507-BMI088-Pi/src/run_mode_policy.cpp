#include "run_mode_policy.hpp"

#include <cmath>

namespace gimbal {

OperatorCommand classifyOperatorCommand(std::string_view name) noexcept {
    if (name == "target") return OperatorCommand::target;
    if (name == "run") return OperatorCommand::run;
    if (name == "stop") return OperatorCommand::stop;
    if (name == "reset-fault") return OperatorCommand::reset_fault;
    if (name == "status") return OperatorCommand::status;
    if (name == "quit" || name == "exit") return OperatorCommand::quit;
    return OperatorCommand::unknown;
}

bool operatorCommandAllowed(bool attitude_uart_mode,
                            OperatorCommand command) noexcept {
    if (!attitude_uart_mode) return command != OperatorCommand::unknown;
    return command == OperatorCommand::status || command == OperatorCommand::quit;
}

bool modeCanEnterRunning(bool attitude_uart_mode) noexcept {
    return !attitude_uart_mode;
}

PacketDecision decideAttitudePublication(
    const AttitudePublicationInput& input) noexcept {
    if (!input.attitude_uart_mode || !input.calibration_complete ||
        input.attitude_degraded || input.safety_state != SafetyState::stop_ready ||
        !input.attitude.valid || !std::isfinite(input.attitude.yaw_deg) ||
        !std::isfinite(input.attitude.pitch_deg) ||
        !std::isfinite(input.attitude.timestamp_s) ||
        !std::isfinite(input.now_s) || !std::isfinite(input.timeout_s) ||
        input.timeout_s <= 0.0) {
        return {};
    }
    const double age_s = input.now_s - input.attitude.timestamp_s;
    if (age_s < 0.0 || age_s > input.timeout_s) return {};
    return {true, makeAttitudePayload(0U, input.attitude.yaw_deg,
                                      input.attitude.pitch_deg)};
}

PacketDecision decideUartTransmission(
    bool attitude_uart_mode, const GimbalPacketPayload& latest,
    std::chrono::steady_clock::time_point updated,
    std::chrono::steady_clock::time_point now, double timeout_s) noexcept {
    if (!std::isfinite(timeout_s) || timeout_s <= 0.0 || now < updated) {
        return attitude_uart_mode ? PacketDecision{}
                                  : PacketDecision{true, makeStopPayload()};
    }
    const bool stale = now - updated > std::chrono::duration<double>(timeout_s);
    if (attitude_uart_mode) {
        if (stale || latest.mode != PacketMode::attitude) return {};
        return {true, latest};
    }
    if (stale || latest.mode == PacketMode::attitude) {
        return {true, makeStopPayload()};
    }
    if (latest.mode == PacketMode::rate_control) return {true, latest};
    if (latest.mode == PacketMode::fault) {
        return {true, makeFaultPayload()};
    }
    return {true, makeStopPayload()};
}

UartFailurePolicy uartFailurePolicy(bool attitude_uart_mode) noexcept {
    if (attitude_uart_mode) return {true, true, false};
    return {true, false, true};
}

std::array<GimbalPacketPayload, kShutdownPacketCount>
makeShutdownPayloads() noexcept {
    std::array<GimbalPacketPayload, kShutdownPacketCount> payloads{};
    for (std::size_t index = 0U; index < payloads.size(); ++index) {
        payloads[index] = makeStopPayload(static_cast<std::uint8_t>(index));
    }
    return payloads;
}

} // namespace gimbal
