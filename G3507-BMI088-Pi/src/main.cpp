#include "application.hpp"
#include "config.hpp"
#include "signal_stop.hpp"

#include <cmath>
#include <iostream>
#include <string>

namespace {

bool parseFloatArgument(const char* text, float& output) {
    try {
        std::size_t used = 0U;
        output = std::stof(text, &used);
        return text[used] == '\0' && std::isfinite(output);
    } catch (...) {
        return false;
    }
}

void printUsage(const char* program) {
    std::cout << "用法：" << program << " [选项]\n"
              << "  --config <文件>       配置文件（默认 config/gimbal.conf）\n"
              << "  --imu-only            只读取和低频显示姿态，不打开串口\n"
              << "  --uart-test           不读取 BMI088，发送受限固定角速度\n"
              << "  --attitude-uart       用原 10 字节帧输出当前 yaw/pitch 姿态\n"
              << "  --dry-run             完成计算和组包，但不写串口\n"
              << "  --auto-run            姿态首次有效后自动启用外环（默认关闭）\n"
              << "  --target <yaw> <pitch> 覆盖启动目标角度，单位为度\n"
              << "  --help                 显示本帮助\n";
}

} // namespace

int main(int argc, char** argv) {
    std::string config_path = "config/gimbal.conf";
    gimbal::RunOptions options{};
    for (int index = 1; index < argc; ++index) {
        const std::string argument = argv[index];
        if (argument == "--config" && index + 1 < argc) {
            config_path = argv[++index];
        } else if (argument == "--imu-only") {
            options.imu_only = true;
        } else if (argument == "--uart-test") {
            options.uart_test = true;
        } else if (argument == "--attitude-uart") {
            options.attitude_uart = true;
        } else if (argument == "--dry-run") {
            options.dry_run = true;
        } else if (argument == "--auto-run") {
            options.auto_run = true;
        } else if (argument == "--target" && index + 2 < argc) {
            options.target_override = parseFloatArgument(argv[index + 1], options.target_yaw_deg) &&
                                      parseFloatArgument(argv[index + 2], options.target_pitch_deg);
            index += 2;
            if (!options.target_override) {
                std::cerr << "--target 后必须是两个有效数字。" << std::endl;
                return 1;
            }
        } else if (argument == "--help") {
            printUsage(argv[0]);
            return 0;
        } else {
            std::cerr << "未知或不完整的选项：" << argument << std::endl;
            printUsage(argv[0]);
            return 1;
        }
    }
    if (options.imu_only && options.uart_test) {
        std::cerr << "--imu-only 与 --uart-test 不能同时使用。" << std::endl;
        return 1;
    }
    if (options.attitude_uart &&
        (options.imu_only || options.uart_test || options.dry_run ||
         options.auto_run || options.target_override)) {
        std::cerr << "--attitude-uart 不能与 --imu-only、--uart-test、--dry-run、"
                     "--auto-run 或 --target 同时使用。" << std::endl;
        return 1;
    }

    gimbal::AppConfig config{};
    std::string error;
    if (!gimbal::loadConfig(config_path, config, error)) {
        std::cerr << error << std::endl;
        return 1;
    }
    if (!gimbal::installSignalStopHandlers(error)) {
        std::cerr << error << std::endl;
        return 1;
    }
    gimbal::Application application(std::move(config), options);
    return application.run();
}
