#pragma once

#include "imu_types.hpp"

#include <array>
#include <cstdint>
#include <string>

namespace gimbal {

struct PidConfig {
    float kp{4.0F};
    float ki{0.0F};
    float kd{0.08F};
    float output_limit_dps{120.0F};
    float integral_limit{20.0F};
    float deadband_deg{0.05F};
    float target_slew_dps{90.0F};
    bool use_measured_rate{true};
};

struct AppConfig {
    std::string accel_spi_device{"/dev/spidev0.0"};
    std::string gyro_spi_device{"/dev/spidev0.1"};
    std::uint32_t spi_speed_hz{1'000'000U};
    std::uint8_t accel_conf{0xABU}; // normal filter, 800 Hz ODR
    std::uint8_t gyro_bandwidth{0x02U}; // 1000 Hz ODR / 116 Hz bandwidth
    std::string uart_device{"/dev/ttyAMA0"};
    std::uint32_t uart_baud{460800U};
    double imu_frequency_hz{500.0};
    double uart_frequency_hz{500.0};
    float accel_cutoff_hz{30.0F};
    float gyro_cutoff_hz{45.0F};
    float mahony_kp{1.5F};
    float mahony_ki{0.02F};
    float mahony_accel_full_trust_error_g{0.08F};
    float mahony_accel_reject_error_g{0.30F};
    float mahony_integral_limit{0.10F};
    float mahony_integral_decay_rate{1.0F};
    double mahony_max_accel_reject_s{0.30};
    PidConfig yaw_pid{};
    PidConfig pitch_pid{4.5F, 0.0F, 0.10F, 90.0F, 20.0F, 0.05F, 90.0F, true};
    std::uint32_t yaw_rate_body_axis{2U};
    std::uint32_t pitch_rate_body_axis{1U};
    float yaw_rate_sign{1.0F};
    float pitch_rate_sign{1.0F};
    float default_target_yaw_deg{0.0F};
    float default_target_pitch_deg{0.0F};
    std::array<float, 9> axis_mapping{1.0F, 0.0F, 0.0F,
                                       0.0F, 1.0F, 0.0F,
                                       0.0F, 0.0F, 1.0F};
    double calibration_seconds{2.0};
    float calibration_max_gyro_stddev_dps{0.8F};
    float calibration_max_mean_rate_dps{3.0F};
    float calibration_max_accel_stddev_g{0.05F};
    bool save_calibration{false};
    std::string calibration_output_file{"config/calibration.conf"};
    double log_frequency_hz{1.0};
    std::uint32_t max_consecutive_spi_failures{5U};
    std::uint32_t fault_recovery_success_frames{25U};
    double command_timeout_s{0.1};
    double uart_frame_deadline_s{0.010};
    std::uint32_t uart_poll_timeout_ms{1U};
    bool auto_run{false};
    float uart_test_yaw_rate_dps{5.0F};
    float uart_test_pitch_rate_dps{0.0F};
};

bool loadConfig(const std::string& path, AppConfig& config, std::string& error);
bool validateConfig(const AppConfig& config, std::string& error);

} // namespace gimbal
