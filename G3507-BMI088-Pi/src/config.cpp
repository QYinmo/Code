#include "config.hpp"
#include "gimbal_protocol.hpp"

#include <algorithm>
#include <cctype>
#include <cmath>
#include <fstream>
#include <sstream>
#include <unordered_map>
#include <vector>

namespace gimbal {
namespace {

std::string trim(std::string value) {
    const auto not_space = [](unsigned char c) { return !std::isspace(c); };
    value.erase(value.begin(), std::find_if(value.begin(), value.end(), not_space));
    value.erase(std::find_if(value.rbegin(), value.rend(), not_space).base(), value.end());
    return value;
}

bool parseFloat(const std::string& text, float& value) {
    try {
        std::size_t used = 0U;
        value = std::stof(text, &used);
        return used == text.size();
    } catch (...) {
        return false;
    }
}

bool parseDouble(const std::string& text, double& value) {
    try {
        std::size_t used = 0U;
        value = std::stod(text, &used);
        return used == text.size();
    } catch (...) {
        return false;
    }
}

bool parseUnsigned(const std::string& text, std::uint32_t& value) {
    try {
        std::size_t used = 0U;
        const auto parsed = std::stoull(text, &used, 0);
        if (used != text.size() || parsed > 0xFFFFFFFFULL) {
            return false;
        }
        value = static_cast<std::uint32_t>(parsed);
        return true;
    } catch (...) {
        return false;
    }
}

bool parseMatrix(const std::string& text, std::array<float, 9>& matrix) {
    std::stringstream stream(text);
    std::string token;
    std::size_t index = 0U;
    while (std::getline(stream, token, ',')) {
        if (index >= matrix.size() || !parseFloat(trim(token), matrix[index])) {
            return false;
        }
        ++index;
    }
    return index == matrix.size();
}

bool parseBool(const std::string& text, bool& value) {
    std::string normalized = text;
    std::transform(normalized.begin(), normalized.end(), normalized.begin(),
                   [](unsigned char c) { return static_cast<char>(std::tolower(c)); });
    if (normalized == "true" || normalized == "1" || normalized == "yes") {
        value = true;
        return true;
    }
    if (normalized == "false" || normalized == "0" || normalized == "no") {
        value = false;
        return true;
    }
    return false;
}

} // namespace

bool loadConfig(const std::string& path, AppConfig& config, std::string& error) {
    std::ifstream file(path);
    if (!file) {
        error = "无法打开配置文件：" + path;
        return false;
    }

    std::unordered_map<std::string, std::string> entries;
    std::string line;
    std::size_t line_number = 0U;
    while (std::getline(file, line)) {
        ++line_number;
        const auto comment = line.find('#');
        if (comment != std::string::npos) {
            line.erase(comment);
        }
        line = trim(line);
        if (line.empty()) {
            continue;
        }
        const auto separator = line.find('=');
        if (separator == std::string::npos) {
            error = "配置文件第 " + std::to_string(line_number) + " 行缺少 '='";
            return false;
        }
        const std::string key = trim(line.substr(0U, separator));
        const std::string value = trim(line.substr(separator + 1U));
        if (key.empty() || value.empty()) {
            error = "配置文件第 " + std::to_string(line_number) + " 行键或值为空";
            return false;
        }
        entries[key] = value;
    }

    const auto set_string = [&entries](const char* key, std::string& output) {
        const auto it = entries.find(key);
        if (it != entries.end()) {
            output = it->second;
        }
    };
    const auto set_float = [&entries, &error](const char* key, float& output) {
        const auto it = entries.find(key);
        if (it == entries.end()) return true;
        if (!parseFloat(it->second, output)) {
            error = std::string("配置项不是有效浮点数：") + key;
            return false;
        }
        return true;
    };
    const auto set_double = [&entries, &error](const char* key, double& output) {
        const auto it = entries.find(key);
        if (it == entries.end()) return true;
        if (!parseDouble(it->second, output)) {
            error = std::string("配置项不是有效浮点数：") + key;
            return false;
        }
        return true;
    };
    const auto set_u32 = [&entries, &error](const char* key, std::uint32_t& output) {
        const auto it = entries.find(key);
        if (it == entries.end()) return true;
        if (!parseUnsigned(it->second, output)) {
            error = std::string("配置项不是有效无符号整数：") + key;
            return false;
        }
        return true;
    };
    const auto set_bool = [&entries, &error](const char* key, bool& output) {
        const auto it = entries.find(key);
        if (it == entries.end()) return true;
        if (!parseBool(it->second, output)) {
            error = std::string("配置项不是有效布尔值：") + key;
            return false;
        }
        return true;
    };

    set_string("accel_spi_device", config.accel_spi_device);
    set_string("gyro_spi_device", config.gyro_spi_device);
    set_string("uart_device", config.uart_device);
    set_string("calibration_output_file", config.calibration_output_file);
    if (!set_u32("spi_speed_hz", config.spi_speed_hz) ||
        !set_u32("uart_baud", config.uart_baud) ||
        !set_double("imu_frequency_hz", config.imu_frequency_hz) ||
        !set_double("uart_frequency_hz", config.uart_frequency_hz) ||
        !set_float("accel_cutoff_hz", config.accel_cutoff_hz) ||
        !set_float("gyro_cutoff_hz", config.gyro_cutoff_hz) ||
        !set_float("mahony_kp", config.mahony_kp) ||
        !set_float("mahony_ki", config.mahony_ki) ||
        !set_float("mahony_accel_full_trust_error_g", config.mahony_accel_full_trust_error_g) ||
        !set_float("mahony_accel_reject_error_g", config.mahony_accel_reject_error_g) ||
        !set_float("mahony_integral_limit", config.mahony_integral_limit) ||
        !set_float("mahony_integral_decay_rate", config.mahony_integral_decay_rate) ||
        !set_double("mahony_max_accel_reject_s", config.mahony_max_accel_reject_s) ||
        !set_float("yaw_kp", config.yaw_pid.kp) ||
        !set_float("yaw_ki", config.yaw_pid.ki) ||
        !set_float("yaw_kd", config.yaw_pid.kd) ||
        !set_float("yaw_max_rate_dps", config.yaw_pid.output_limit_dps) ||
        !set_float("yaw_integral_limit", config.yaw_pid.integral_limit) ||
        !set_float("yaw_deadband_deg", config.yaw_pid.deadband_deg) ||
        !set_float("yaw_target_slew_dps", config.yaw_pid.target_slew_dps) ||
        !set_bool("yaw_use_measured_rate", config.yaw_pid.use_measured_rate) ||
        !set_float("pitch_kp", config.pitch_pid.kp) ||
        !set_float("pitch_ki", config.pitch_pid.ki) ||
        !set_float("pitch_kd", config.pitch_pid.kd) ||
        !set_float("pitch_max_rate_dps", config.pitch_pid.output_limit_dps) ||
        !set_float("pitch_integral_limit", config.pitch_pid.integral_limit) ||
        !set_float("pitch_deadband_deg", config.pitch_pid.deadband_deg) ||
        !set_float("pitch_target_slew_dps", config.pitch_pid.target_slew_dps) ||
        !set_bool("pitch_use_measured_rate", config.pitch_pid.use_measured_rate) ||
        !set_u32("yaw_rate_body_axis", config.yaw_rate_body_axis) ||
        !set_u32("pitch_rate_body_axis", config.pitch_rate_body_axis) ||
        !set_float("yaw_rate_sign", config.yaw_rate_sign) ||
        !set_float("pitch_rate_sign", config.pitch_rate_sign) ||
        !set_float("default_target_yaw_deg", config.default_target_yaw_deg) ||
        !set_float("default_target_pitch_deg", config.default_target_pitch_deg) ||
        !set_double("calibration_seconds", config.calibration_seconds) ||
        !set_float("calibration_max_gyro_stddev_dps", config.calibration_max_gyro_stddev_dps) ||
        !set_float("calibration_max_mean_rate_dps", config.calibration_max_mean_rate_dps) ||
        !set_float("calibration_max_accel_stddev_g", config.calibration_max_accel_stddev_g) ||
        !set_bool("save_calibration", config.save_calibration) ||
        !set_double("log_frequency_hz", config.log_frequency_hz) ||
        !set_u32("max_consecutive_spi_failures", config.max_consecutive_spi_failures) ||
        !set_u32("fault_recovery_success_frames", config.fault_recovery_success_frames) ||
        !set_double("command_timeout_s", config.command_timeout_s) ||
        !set_double("uart_frame_deadline_s", config.uart_frame_deadline_s) ||
        !set_u32("uart_poll_timeout_ms", config.uart_poll_timeout_ms) ||
        !set_bool("auto_run", config.auto_run) ||
        !set_float("uart_test_yaw_rate_dps", config.uart_test_yaw_rate_dps) ||
        !set_float("uart_test_pitch_rate_dps", config.uart_test_pitch_rate_dps)) {
        return false;
    }

    std::uint32_t temporary = config.accel_conf;
    if (!set_u32("accel_conf", temporary) || temporary > 0xFFU) {
        error = "accel_conf 必须在 0x00 到 0xFF 之间";
        return false;
    }
    config.accel_conf = static_cast<std::uint8_t>(temporary);
    temporary = config.gyro_bandwidth;
    if (!set_u32("gyro_bandwidth", temporary) || temporary > 0x07U) {
        error = "gyro_bandwidth 必须在 0x00 到 0x07 之间";
        return false;
    }
    config.gyro_bandwidth = static_cast<std::uint8_t>(temporary);

    const auto matrix_it = entries.find("axis_mapping");
    if (matrix_it != entries.end() && !parseMatrix(matrix_it->second, config.axis_mapping)) {
        error = "axis_mapping 必须包含 9 个逗号分隔的数字";
        return false;
    }
    return validateConfig(config, error);
}

bool validateConfig(const AppConfig& config, std::string& error) {
    const auto finite = [](auto value) { return std::isfinite(static_cast<double>(value)); };
    if (config.accel_spi_device.empty() || config.gyro_spi_device.empty() || config.uart_device.empty() ||
        (config.save_calibration && config.calibration_output_file.empty())) {
        error = "设备路径不能为空";
        return false;
    }
    if (!finite(config.imu_frequency_hz) || !finite(config.uart_frequency_hz) ||
        !finite(config.log_frequency_hz) || config.spi_speed_hz == 0U || config.imu_frequency_hz < 1.0 ||
        config.uart_frequency_hz < 1.0 || config.log_frequency_hz <= 0.0) {
        error = "SPI 速度和各线程频率必须为正数";
        return false;
    }
    if (!finite(config.calibration_seconds) || !finite(config.command_timeout_s) ||
        !finite(config.uart_frame_deadline_s) || config.calibration_seconds < 0.1 ||
        config.command_timeout_s < 5.0 / config.uart_frequency_hz || config.command_timeout_s > 1.0 ||
        config.uart_frame_deadline_s < 2.0 / config.uart_frequency_hz ||
        config.uart_frame_deadline_s > config.command_timeout_s ||
        config.calibration_max_accel_stddev_g <= 0.0F ||
        config.max_consecutive_spi_failures == 0U || config.fault_recovery_success_frames == 0U ||
        config.uart_poll_timeout_ms > 10U) {
        error = "校准、命令超时、UART 帧截止时间或故障恢复参数无效";
        return false;
    }
    const double uart_poll_timeout_s = static_cast<double>(config.uart_poll_timeout_ms) / 1000.0;
    if (uart_poll_timeout_s >= config.uart_frame_deadline_s ||
        uart_poll_timeout_s > config.uart_frame_deadline_s * 0.5) {
        error = "UART poll 等待时间必须严格小于帧截止时间，并至少保留一半截止时间用于完成帧";
        return false;
    }
    if (!finite(config.accel_cutoff_hz) || !finite(config.gyro_cutoff_hz) ||
        config.accel_cutoff_hz <= 0.0F || config.gyro_cutoff_hz <= 0.0F ||
        config.accel_cutoff_hz >= 0.45 * config.imu_frequency_hz ||
        config.gyro_cutoff_hz >= 0.45 * config.imu_frequency_hz) {
        error = "低通截止频率必须为正且小于 IMU 频率的 45%";
        return false;
    }
    if (!finite(config.mahony_kp) || !finite(config.mahony_ki) ||
        !finite(config.mahony_accel_full_trust_error_g) ||
        !finite(config.mahony_accel_reject_error_g) || !finite(config.mahony_integral_limit) ||
        !finite(config.mahony_integral_decay_rate) || !finite(config.mahony_max_accel_reject_s) ||
        config.mahony_kp < 0.0F || config.mahony_ki < 0.0F ||
        config.mahony_accel_full_trust_error_g < 0.0F ||
        config.mahony_accel_reject_error_g <= config.mahony_accel_full_trust_error_g ||
        config.mahony_integral_limit < 0.0F || config.mahony_integral_decay_rate < 0.0F ||
        config.mahony_max_accel_reject_s <= 0.0) {
        error = "Mahony 增益、加速度可信度门限、积分限幅或降级时间无效";
        return false;
    }
    const auto valid_pid = [&finite](const PidConfig& pid) {
        return finite(pid.kp) && finite(pid.ki) && finite(pid.kd) &&
               finite(pid.output_limit_dps) && finite(pid.integral_limit) &&
               finite(pid.deadband_deg) && finite(pid.target_slew_dps) &&
               pid.kp >= 0.0F && pid.ki >= 0.0F && pid.kd >= 0.0F &&
               pid.output_limit_dps > 0.0F && pid.integral_limit >= 0.0F &&
               pid.deadband_deg >= 0.0F && pid.target_slew_dps > 0.0F;
    };
    if (!valid_pid(config.yaw_pid) || !valid_pid(config.pitch_pid)) {
        error = "PID 限幅、死区或目标斜坡参数无效";
        return false;
    }
    if (!AxisMapping(config.axis_mapping).valid()) {
        error = "axis_mapping 不是正交单位矩阵";
        return false;
    }
    if (config.yaw_rate_body_axis > 2U || config.pitch_rate_body_axis > 2U ||
        !finite(config.yaw_rate_sign) || !finite(config.pitch_rate_sign) ||
        std::fabs(std::fabs(config.yaw_rate_sign) - 1.0F) > 1.0e-5F ||
        std::fabs(std::fabs(config.pitch_rate_sign) - 1.0F) > 1.0e-5F) {
        error = "控制角速度轴必须为 0..2，符号必须为 +1 或 -1";
        return false;
    }
    if (!finite(config.default_target_yaw_deg) || !finite(config.default_target_pitch_deg) ||
        !finite(config.calibration_max_gyro_stddev_dps) ||
        !finite(config.calibration_max_mean_rate_dps) ||
        !finite(config.calibration_max_accel_stddev_g) ||
        config.calibration_max_gyro_stddev_dps <= 0.0F ||
        config.calibration_max_mean_rate_dps <= 0.0F ||
        config.calibration_max_accel_stddev_g <= 0.0F) {
        error = "目标角度或校准阈值包含非有限值或无效值";
        return false;
    }
    if (!finite(config.uart_test_yaw_rate_dps) || !finite(config.uart_test_pitch_rate_dps)) {
        error = "UART 测试角速度必须是有限值";
        return false;
    }
    const double uart_bits_per_second = config.uart_frequency_hz *
                                        static_cast<double>(kPacketLength) * 10.0;
    if (uart_bits_per_second > static_cast<double>(config.uart_baud) * 0.8) {
        error = "UART 频率和 10 字节 8N1 帧超过波特率的 80% 安全带宽";
        return false;
    }
    return true;
}

} // namespace gimbal
