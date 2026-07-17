#pragma once

#include <algorithm>
#include <cmath>

namespace gimbal {

class LowPassFilter {
public:
    explicit LowPassFilter(float cutoff_hz = 30.0F) : cutoff_hz_(cutoff_hz) {}

    float update(float input, double dt_s) noexcept {
        if (!initialized_) {
            state_ = input;
            initialized_ = true;
            return state_;
        }
        if (!(dt_s > 0.0) || !std::isfinite(dt_s) || cutoff_hz_ <= 0.0F) {
            return state_;
        }
        constexpr double two_pi = 6.28318530717958647692;
        const double rc_s = 1.0 / (two_pi * static_cast<double>(cutoff_hz_));
        const float alpha = static_cast<float>(std::clamp(dt_s / (rc_s + dt_s), 0.0, 1.0));
        state_ += alpha * (input - state_);
        return state_;
    }

    void reset() noexcept { initialized_ = false; state_ = 0.0F; }

private:
    float cutoff_hz_;
    float state_{0.0F};
    bool initialized_{false};
};

} // namespace gimbal
