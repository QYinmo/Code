#pragma once

#include "imu_types.hpp"

namespace gimbal {

struct MahonyConfig {
    float kp{1.5F};
    float ki{0.02F};
    float accel_full_trust_error_g{0.08F};
    float accel_reject_error_g{0.30F};
    float integral_limit{0.10F};
    float integral_decay_rate{1.0F};
    double max_accel_reject_s{0.30};
};

class MahonyFilter {
public:
    explicit MahonyFilter(MahonyConfig config) : config_(config) {}
    MahonyFilter(float kp, float ki) : config_{kp, ki} {}
    bool update(const Vector3& accel_g, const Vector3& gyro_dps, double dt_s, Attitude& attitude) noexcept;
    // 为 GM6020 编码器或视觉提供渐进式相对 yaw 修正，不引入具体依赖。
    void applyYawCorrectionDeg(float reference_yaw_deg, float gain) noexcept;
    void reset() noexcept;
    [[nodiscard]] float accelWeight() const noexcept { return accel_weight_; }
    [[nodiscard]] double continuousAccelRejectedS() const noexcept { return accel_rejected_s_; }
    [[nodiscard]] bool attitudeDegraded() const noexcept {
        return accel_rejected_s_ > config_.max_accel_reject_s;
    }
    [[nodiscard]] Vector3 integralState() const noexcept { return integral_; }
    [[nodiscard]] float quaternionNorm() const noexcept;

private:
    MahonyConfig config_;
    float q0_{1.0F};
    float q1_{0.0F};
    float q2_{0.0F};
    float q3_{0.0F};
    Vector3 integral_{};
    float previous_wrapped_yaw_deg_{0.0F};
    float continuous_yaw_deg_{0.0F};
    bool yaw_initialized_{false};
    float accel_weight_{0.0F};
    double accel_rejected_s_{0.0};
};

} // namespace gimbal
