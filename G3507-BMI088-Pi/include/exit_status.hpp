#pragma once

#include <atomic>
#include <cstdint>

namespace gimbal {

enum class ExitReason : std::uint8_t {
    normal_completion,
    user_stop,
    signal_stop,
    bmi_initialization_failure,
    uart_initialization_failure,
    calibration_failure,
    runtime_failure
};

constexpr bool isFailureExit(ExitReason reason) noexcept {
    return reason == ExitReason::bmi_initialization_failure ||
           reason == ExitReason::uart_initialization_failure ||
           reason == ExitReason::calibration_failure ||
           reason == ExitReason::runtime_failure;
}

constexpr int exitCodeFor(ExitReason reason) noexcept {
    return isFailureExit(reason) ? 2 : 0;
}

class ExitStatus {
public:
    void record(ExitReason reason) noexcept {
        if (isFailureExit(reason)) {
            reason_.store(reason);
            return;
        }
        ExitReason current = reason_.load();
        while (!isFailureExit(current) &&
               !reason_.compare_exchange_weak(current, reason)) {}
    }

    [[nodiscard]] ExitReason reason() const noexcept { return reason_.load(); }
    [[nodiscard]] int code() const noexcept { return exitCodeFor(reason()); }

private:
    std::atomic<ExitReason> reason_{ExitReason::normal_completion};
};

} // namespace gimbal
