#pragma once

#include "config.hpp"

namespace gimbal {

class AnglePidController {
public:
    explicit AnglePidController(PidConfig config) : config_(config) {}
    float update(float requested_target_deg, float measurement_deg, double dt_s, bool wrap_angle,
                 float measured_rate_dps = 0.0F) noexcept;
    void reset(float current_target_deg = 0.0F) noexcept;
    static float wrapErrorDeg(float error_deg) noexcept;
    [[nodiscard]] float integralState() const noexcept { return integral_; }

private:
    PidConfig config_;
    float ramped_target_deg_{0.0F};
    float integral_{0.0F};
    float previous_error_deg_{0.0F};
    bool initialized_{false};
};

} // namespace gimbal
