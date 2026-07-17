#include "pid_controller.hpp"

#include <algorithm>
#include <cmath>

namespace gimbal {

float AnglePidController::wrapErrorDeg(float error_deg) noexcept {
    while (error_deg > 180.0F) error_deg -= 360.0F;
    while (error_deg < -180.0F) error_deg += 360.0F;
    return error_deg;
}

void AnglePidController::reset(float current_target_deg) noexcept {
    ramped_target_deg_ = current_target_deg;
    integral_ = 0.0F;
    previous_error_deg_ = 0.0F;
    initialized_ = false;
}

float AnglePidController::update(float requested_target_deg, float measurement_deg,
                                 double dt_s, bool wrap_angle, float measured_rate_dps) noexcept {
    if (!std::isfinite(requested_target_deg) || !std::isfinite(measurement_deg) ||
        !std::isfinite(dt_s) || !std::isfinite(measured_rate_dps) || dt_s <= 0.0) return 0.0F;
    const float dt = static_cast<float>(std::clamp(dt_s, 0.0002, 0.05));
    if (!initialized_) {
        ramped_target_deg_ = measurement_deg;
        previous_error_deg_ = 0.0F;
        initialized_ = true;
    }
    float target_delta = requested_target_deg - ramped_target_deg_;
    if (wrap_angle) target_delta = wrapErrorDeg(target_delta);
    const float maximum_step = config_.target_slew_dps * dt;
    ramped_target_deg_ += std::clamp(target_delta, -maximum_step, maximum_step);

    float error = ramped_target_deg_ - measurement_deg;
    if (wrap_angle) error = wrapErrorDeg(error);
    if (std::fabs(error) < config_.deadband_deg) error = 0.0F;
    const float derivative_term = config_.use_measured_rate
                                      ? -config_.kd * measured_rate_dps
                                      : config_.kd * (error - previous_error_deg_) / dt;
    const float candidate_integral = std::clamp(integral_ + error * dt,
        -config_.integral_limit, config_.integral_limit);
    const float candidate_output = config_.kp * error + config_.ki * candidate_integral +
                                   derivative_term;
    // 条件积分抗饱和：只有未饱和或误差正在解除饱和时才接受新积分。
    if (std::fabs(candidate_output) <= config_.output_limit_dps ||
        (candidate_output > config_.output_limit_dps && error < 0.0F) ||
        (candidate_output < -config_.output_limit_dps && error > 0.0F)) {
        integral_ = candidate_integral;
    }
    previous_error_deg_ = error;
    const float output = config_.kp * error + config_.ki * integral_ + derivative_term;
    return std::clamp(output, -config_.output_limit_dps, config_.output_limit_dps);
}

} // namespace gimbal
