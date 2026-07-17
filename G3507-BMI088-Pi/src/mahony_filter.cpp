#include "mahony_filter.hpp"

#include <algorithm>
#include <cmath>

namespace gimbal {

void MahonyFilter::reset() noexcept {
    q0_ = 1.0F; q1_ = 0.0F; q2_ = 0.0F; q3_ = 0.0F;
    integral_ = {};
    previous_wrapped_yaw_deg_ = 0.0F;
    continuous_yaw_deg_ = 0.0F;
    yaw_initialized_ = false;
    accel_weight_ = 0.0F;
    accel_rejected_s_ = 0.0;
}

float MahonyFilter::quaternionNorm() const noexcept {
    return std::sqrt(q0_ * q0_ + q1_ * q1_ + q2_ * q2_ + q3_ * q3_);
}

void MahonyFilter::applyYawCorrectionDeg(float reference_yaw_deg, float gain) noexcept {
    if (!yaw_initialized_ || !std::isfinite(reference_yaw_deg) || !std::isfinite(gain)) return;
    const float limited_gain = std::clamp(gain, 0.0F, 1.0F);
    float error = reference_yaw_deg - continuous_yaw_deg_;
    while (error > 180.0F) error -= 360.0F;
    while (error < -180.0F) error += 360.0F;
    continuous_yaw_deg_ += limited_gain * error;
}

bool MahonyFilter::update(const Vector3& accel_g, const Vector3& gyro_dps,
                          double dt_s, Attitude& attitude) noexcept {
    if (!std::isfinite(dt_s) || dt_s <= 0.0 ||
        !std::isfinite(accel_g.x) || !std::isfinite(accel_g.y) || !std::isfinite(accel_g.z) ||
        !std::isfinite(gyro_dps.x) || !std::isfinite(gyro_dps.y) || !std::isfinite(gyro_dps.z)) {
        return false;
    }
    // 应用层会对真正的采样超时停机；滤波器仍对轻微调度异常做有界传播。
    const float dt = static_cast<float>(std::clamp(dt_s, 0.0002, 0.02));
    const float accel_norm_sq = accel_g.x * accel_g.x + accel_g.y * accel_g.y +
                                accel_g.z * accel_g.z;
    if (!std::isfinite(accel_norm_sq)) return false;
    const float accel_norm = std::sqrt(std::max(accel_norm_sq, 0.0F));
    const float norm_error = std::fabs(accel_norm - 1.0F);
    if (norm_error <= config_.accel_full_trust_error_g) {
        accel_weight_ = 1.0F;
    } else if (norm_error >= config_.accel_reject_error_g) {
        accel_weight_ = 0.0F;
    } else {
        const float span = config_.accel_reject_error_g - config_.accel_full_trust_error_g;
        const float t = std::clamp((config_.accel_reject_error_g - norm_error) / span, 0.0F, 1.0F);
        // smoothstep 避免门限处突然开关校正量。
        accel_weight_ = t * t * (3.0F - 2.0F * t);
    }
    if (accel_weight_ <= 0.001F) accel_rejected_s_ += static_cast<double>(dt);
    else accel_rejected_s_ = 0.0;

    float ex = 0.0F;
    float ey = 0.0F;
    float ez = 0.0F;
    if (accel_norm > 1.0e-6F && accel_weight_ > 0.0F) {
        const float inv_accel_norm = 1.0F / accel_norm;
        const float ax = accel_g.x * inv_accel_norm;
        const float ay = accel_g.y * inv_accel_norm;
        const float az = accel_g.z * inv_accel_norm;
        const float vx = 2.0F * (q1_ * q3_ - q0_ * q2_);
        const float vy = 2.0F * (q0_ * q1_ + q2_ * q3_);
        const float vz = q0_ * q0_ - q1_ * q1_ - q2_ * q2_ + q3_ * q3_;
        ex = (ay * vz - az * vy) * accel_weight_;
        ey = (az * vx - ax * vz) * accel_weight_;
        ez = (ax * vy - ay * vx) * accel_weight_;
    }

    if (config_.ki > 0.0F && accel_weight_ > 0.0F) {
        integral_.x = std::clamp(integral_.x + config_.ki * ex * dt,
                                 -config_.integral_limit, config_.integral_limit);
        integral_.y = std::clamp(integral_.y + config_.ki * ey * dt,
                                 -config_.integral_limit, config_.integral_limit);
        integral_.z = std::clamp(integral_.z + config_.ki * ez * dt,
                                 -config_.integral_limit, config_.integral_limit);
    } else {
        const float decay = std::clamp(1.0F - config_.integral_decay_rate * dt, 0.0F, 1.0F);
        integral_.x *= decay;
        integral_.y *= decay;
        integral_.z *= decay;
    }

    constexpr float deg_to_rad = 0.01745329251994329577F;
    const float gx = gyro_dps.x * deg_to_rad + config_.kp * ex + integral_.x;
    const float gy = gyro_dps.y * deg_to_rad + config_.kp * ey + integral_.y;
    const float gz = gyro_dps.z * deg_to_rad + config_.kp * ez + integral_.z;
    const float half_dt = 0.5F * dt;
    const float old_q0 = q0_;
    const float old_q1 = q1_;
    const float old_q2 = q2_;
    const float old_q3 = q3_;
    q0_ += (-old_q1 * gx - old_q2 * gy - old_q3 * gz) * half_dt;
    q1_ += ( old_q0 * gx + old_q2 * gz - old_q3 * gy) * half_dt;
    q2_ += ( old_q0 * gy - old_q1 * gz + old_q3 * gx) * half_dt;
    q3_ += ( old_q0 * gz + old_q1 * gy - old_q2 * gx) * half_dt;
    const float q_norm_sq = q0_ * q0_ + q1_ * q1_ + q2_ * q2_ + q3_ * q3_;
    if (!std::isfinite(q_norm_sq) || q_norm_sq < 1.0e-12F) {
        reset();
        return false;
    }
    const float inv_q_norm = 1.0F / std::sqrt(q_norm_sq);
    q0_ *= inv_q_norm; q1_ *= inv_q_norm; q2_ *= inv_q_norm; q3_ *= inv_q_norm;

    constexpr float rad_to_deg = 57.295779513082320876F;
    const float roll = std::atan2(2.0F * (q0_ * q1_ + q2_ * q3_),
                                  1.0F - 2.0F * (q1_ * q1_ + q2_ * q2_)) * rad_to_deg;
    const float pitch_argument = std::clamp(2.0F * (q0_ * q2_ - q3_ * q1_), -1.0F, 1.0F);
    const float pitch = std::asin(pitch_argument) * rad_to_deg;
    const float wrapped_yaw = std::atan2(2.0F * (q0_ * q3_ + q1_ * q2_),
                                         1.0F - 2.0F * (q2_ * q2_ + q3_ * q3_)) * rad_to_deg;
    if (!yaw_initialized_) {
        continuous_yaw_deg_ = wrapped_yaw;
        yaw_initialized_ = true;
    } else {
        float delta = wrapped_yaw - previous_wrapped_yaw_deg_;
        if (delta > 180.0F) delta -= 360.0F;
        if (delta < -180.0F) delta += 360.0F;
        continuous_yaw_deg_ += delta;
    }
    previous_wrapped_yaw_deg_ = wrapped_yaw;
    attitude.roll_deg = roll;
    attitude.pitch_deg = pitch;
    attitude.yaw_deg = continuous_yaw_deg_;
    attitude.valid = std::isfinite(roll) && std::isfinite(pitch) &&
                     std::isfinite(continuous_yaw_deg_);
    return attitude.valid;
}

} // namespace gimbal
