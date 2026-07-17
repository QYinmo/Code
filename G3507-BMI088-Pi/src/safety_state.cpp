#include "safety_state.hpp"

#include <algorithm>

namespace gimbal {

SafetyStateMachine::SafetyStateMachine(std::uint32_t recovery_success_frames)
    : required_successes_(std::max<std::uint32_t>(1U, recovery_success_frames)) {}

void SafetyStateMachine::beginCalibration() noexcept {
    state_ = SafetyState::calibrating;
    last_fault_ = FaultReason::none;
    consecutive_successes_ = 0U;
}

void SafetyStateMachine::calibrationSucceeded() noexcept {
    state_ = SafetyState::stop_ready;
    last_fault_ = FaultReason::none;
    consecutive_successes_ = 0U;
}

void SafetyStateMachine::calibrationFailed() noexcept {
    state_ = SafetyState::fault;
    last_fault_ = FaultReason::calibration_failed;
    consecutive_successes_ = 0U;
}

bool SafetyStateMachine::requestRun(const RunPreconditions& conditions) noexcept {
    const bool healthy = conditions.attitude_valid && conditions.imu_fresh &&
                         conditions.uart_ready && conditions.targets_finite;
    const bool recovered = (state_ != SafetyState::fault && state_ != SafetyState::degraded) ||
                           consecutive_successes_ >= required_successes_;
    if (!healthy || !recovered || state_ == SafetyState::boot ||
        state_ == SafetyState::calibrating) {
        return false;
    }
    state_ = SafetyState::running;
    return true;
}

void SafetyStateMachine::requestStop() noexcept {
    if (state_ != SafetyState::fault && state_ != SafetyState::boot &&
        state_ != SafetyState::calibrating) {
        state_ = SafetyState::stop_ready;
    }
}

void SafetyStateMachine::sampleFailure(FaultReason reason, bool threshold_reached) noexcept {
    last_fault_ = reason;
    consecutive_successes_ = 0U;
    if (threshold_reached) {
        state_ = SafetyState::fault;
    } else if (state_ != SafetyState::fault) {
        state_ = SafetyState::degraded;
    }
}

void SafetyStateMachine::sampleSuccess() noexcept {
    if (consecutive_successes_ < required_successes_) ++consecutive_successes_;
    if (state_ == SafetyState::degraded && consecutive_successes_ >= required_successes_) {
        state_ = SafetyState::stop_ready;
    }
}

void SafetyStateMachine::uartFault(FaultReason reason) noexcept {
    last_fault_ = reason;
    state_ = SafetyState::fault;
    consecutive_successes_ = 0U;
}

void SafetyStateMachine::attitudeDegraded(FaultReason reason) noexcept {
    last_fault_ = reason;
    consecutive_successes_ = 0U;
    if (state_ != SafetyState::fault) state_ = SafetyState::degraded;
}

bool SafetyStateMachine::resetFault() noexcept {
    if ((state_ == SafetyState::fault || state_ == SafetyState::degraded) &&
        consecutive_successes_ >= required_successes_) {
        state_ = SafetyState::stop_ready;
        return true;
    }
    return false;
}

const char* SafetyStateMachine::stateName() const noexcept {
    switch (state_) {
    case SafetyState::boot: return "BOOT";
    case SafetyState::calibrating: return "CALIBRATING";
    case SafetyState::stop_ready: return "STOP_READY";
    case SafetyState::running: return "RUNNING";
    case SafetyState::degraded: return "DEGRADED";
    case SafetyState::fault: return "FAULT";
    }
    return "UNKNOWN";
}

const char* SafetyStateMachine::faultName() const noexcept {
    switch (last_fault_) {
    case FaultReason::none: return "无";
    case FaultReason::calibration_failed: return "校准失败";
    case FaultReason::spi_read: return "SPI 读取失败";
    case FaultReason::sample_timeout: return "IMU 采样超时";
    case FaultReason::attitude_invalid: return "姿态无效";
    case FaultReason::accel_rejected: return "加速度连续不可信";
    case FaultReason::uart_io: return "UART 系统调用错误";
    case FaultReason::uart_deadline: return "UART 帧发送超时";
    }
    return "未知故障";
}

} // namespace gimbal
