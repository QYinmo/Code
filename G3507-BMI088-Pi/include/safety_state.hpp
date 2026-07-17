#pragma once

#include <cstdint>

namespace gimbal {

enum class SafetyState : std::uint8_t {
    boot,
    calibrating,
    stop_ready,
    running,
    degraded,
    fault
};

enum class FaultReason : std::uint8_t {
    none,
    calibration_failed,
    spi_read,
    sample_timeout,
    attitude_invalid,
    accel_rejected,
    uart_io,
    uart_deadline
};

struct RunPreconditions {
    bool attitude_valid{false};
    bool imu_fresh{false};
    bool uart_ready{false};
    bool targets_finite{false};
};

class SafetyStateMachine {
public:
    explicit SafetyStateMachine(std::uint32_t recovery_success_frames);
    void beginCalibration() noexcept;
    void calibrationSucceeded() noexcept;
    void calibrationFailed() noexcept;
    bool requestRun(const RunPreconditions& conditions) noexcept;
    void requestStop() noexcept;
    void sampleFailure(FaultReason reason, bool threshold_reached) noexcept;
    void sampleSuccess() noexcept;
    void uartFault(FaultReason reason) noexcept;
    void attitudeDegraded(FaultReason reason) noexcept;
    bool resetFault() noexcept;

    [[nodiscard]] SafetyState state() const noexcept { return state_; }
    [[nodiscard]] FaultReason lastFault() const noexcept { return last_fault_; }
    [[nodiscard]] std::uint32_t consecutiveSuccesses() const noexcept { return consecutive_successes_; }
    [[nodiscard]] bool isRunning() const noexcept { return state_ == SafetyState::running; }
    [[nodiscard]] const char* stateName() const noexcept;
    [[nodiscard]] const char* faultName() const noexcept;

private:
    std::uint32_t required_successes_;
    std::uint32_t consecutive_successes_{0U};
    SafetyState state_{SafetyState::boot};
    FaultReason last_fault_{FaultReason::none};
};

} // namespace gimbal
