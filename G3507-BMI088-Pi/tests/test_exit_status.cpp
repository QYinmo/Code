#include "exit_status.hpp"

#include <iostream>

int main() {
    if (gimbal::exitCodeFor(gimbal::ExitReason::normal_completion) != 0 ||
        gimbal::exitCodeFor(gimbal::ExitReason::user_stop) != 0 ||
        gimbal::exitCodeFor(gimbal::ExitReason::signal_stop) != 0) {
        std::cerr << "正常、用户或信号安全退出没有返回 0\n";
        return 1;
    }
    if (gimbal::exitCodeFor(gimbal::ExitReason::calibration_failure) == 0 ||
        gimbal::exitCodeFor(gimbal::ExitReason::bmi_initialization_failure) == 0 ||
        gimbal::exitCodeFor(gimbal::ExitReason::uart_initialization_failure) == 0 ||
        gimbal::exitCodeFor(gimbal::ExitReason::runtime_failure) == 0) {
        std::cerr << "启动或运行故障错误地返回了 0\n";
        return 1;
    }
    gimbal::ExitStatus status;
    status.record(gimbal::ExitReason::runtime_failure);
    status.record(gimbal::ExitReason::signal_stop);
    if (status.code() == 0 || status.reason() != gimbal::ExitReason::runtime_failure) {
        std::cerr << "后续信号退出覆盖了已锁存的运行故障\n";
        return 1;
    }
    return 0;
}
