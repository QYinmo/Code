#include "config.hpp"

#include <iostream>
#include <limits>

bool invalid(const gimbal::AppConfig& config) {
    std::string error;
    return !gimbal::validateConfig(config, error);
}

int main() {
    gimbal::AppConfig config{};
    std::string error;
    if (!gimbal::validateConfig(config, error)) {
        std::cerr << "默认配置无效：" << error << '\n';
        return 1;
    }
    auto poll_bad = config;
    poll_bad.uart_poll_timeout_ms = 10U;
    poll_bad.uart_frame_deadline_s = 0.010;
    error.clear();
    if (gimbal::validateConfig(poll_bad, error) || error.find("UART poll") == std::string::npos) {
        std::cerr << "poll 超时等于帧截止时间时未被明确拒绝\n";
        return 1;
    }
    poll_bad = config;
    poll_bad.uart_poll_timeout_ms = 10U;
    poll_bad.uart_frame_deadline_s = 0.009;
    error.clear();
    if (gimbal::validateConfig(poll_bad, error) || error.find("UART poll") == std::string::npos) {
        std::cerr << "poll 超时大于帧截止时间时未被明确拒绝\n";
        return 1;
    }
    auto bad = config;
    bad.yaw_pid.kp = std::numeric_limits<float>::quiet_NaN();
    if (!invalid(bad)) return 1;
    bad = config;
    bad.accel_cutoff_hz = 0.5F * static_cast<float>(bad.imu_frequency_hz);
    if (!invalid(bad)) return 1;
    bad = config;
    bad.command_timeout_s = 1.0 / bad.uart_frequency_hz;
    if (!invalid(bad)) return 1;
    bad = config;
    bad.mahony_accel_reject_error_g = bad.mahony_accel_full_trust_error_g;
    if (!invalid(bad)) return 1;
    bad = config;
    bad.fault_recovery_success_frames = 0U;
    if (!invalid(bad)) return 1;
    bad = config;
    bad.uart_baud = 9600U;
    if (!invalid(bad)) return 1;
    bad = config;
    bad.yaw_rate_body_axis = 3U;
    if (!invalid(bad)) return 1;
    bad = config;
    bad.axis_mapping[0] = std::numeric_limits<float>::quiet_NaN();
    if (!invalid(bad)) return 1;
    bad = config;
    bad.uart_test_yaw_rate_dps = std::numeric_limits<float>::infinity();
    if (!invalid(bad)) return 1;
    return 0;
}
