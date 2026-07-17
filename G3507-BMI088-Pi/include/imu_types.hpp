#pragma once

#include <array>
#include <cmath>

namespace gimbal {

struct Vector3 {
    float x{0.0F};
    float y{0.0F};
    float z{0.0F};
};

struct ImuSample {
    double timestamp_s{0.0};
    float ax{0.0F}; // g
    float ay{0.0F};
    float az{0.0F};
    float gx{0.0F}; // degree/s
    float gy{0.0F};
    float gz{0.0F};
    float temperature_c{0.0F};
};

struct Attitude {
    double timestamp_s{0.0};
    float roll_deg{0.0F};
    float pitch_deg{0.0F};
    float yaw_deg{0.0F}; // 连续展开的相对角度
    bool valid{false};
};

struct CalibrationResult {
    Vector3 gyro_bias_dps{};
    Vector3 accel_mean_g{};
    bool valid{false};
};

class AxisMapping {
public:
    AxisMapping() = default;
    explicit AxisMapping(const std::array<float, 9>& matrix) : matrix_(matrix) {}

    [[nodiscard]] Vector3 apply(const Vector3& input) const noexcept {
        return {
            matrix_[0] * input.x + matrix_[1] * input.y + matrix_[2] * input.z,
            matrix_[3] * input.x + matrix_[4] * input.y + matrix_[5] * input.z,
            matrix_[6] * input.x + matrix_[7] * input.y + matrix_[8] * input.z};
    }

    [[nodiscard]] bool valid() const noexcept {
        constexpr float tolerance = 1.0e-4F;
        for (float value : matrix_) {
            if (!std::isfinite(value)) return false;
        }
        for (int row = 0; row < 3; ++row) {
            float norm = 0.0F;
            for (int col = 0; col < 3; ++col) {
                norm += matrix_[static_cast<std::size_t>(row * 3 + col)] *
                        matrix_[static_cast<std::size_t>(row * 3 + col)];
            }
            if (std::fabs(norm - 1.0F) > tolerance) {
                return false;
            }
        }
        for (int a = 0; a < 3; ++a) {
            for (int b = a + 1; b < 3; ++b) {
                float dot = 0.0F;
                for (int col = 0; col < 3; ++col) {
                    dot += matrix_[static_cast<std::size_t>(a * 3 + col)] *
                           matrix_[static_cast<std::size_t>(b * 3 + col)];
                }
                if (std::fabs(dot) > tolerance) {
                    return false;
                }
            }
        }
        return true;
    }

private:
    std::array<float, 9> matrix_{1.0F, 0.0F, 0.0F,
                                  0.0F, 1.0F, 0.0F,
                                  0.0F, 0.0F, 1.0F};
};

inline bool finite(const ImuSample& sample) noexcept {
    return std::isfinite(sample.timestamp_s) && std::isfinite(sample.ax) &&
           std::isfinite(sample.ay) && std::isfinite(sample.az) &&
           std::isfinite(sample.gx) && std::isfinite(sample.gy) &&
           std::isfinite(sample.gz) && std::isfinite(sample.temperature_c);
}

} // namespace gimbal
