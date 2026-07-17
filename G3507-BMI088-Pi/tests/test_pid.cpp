#include "pid_controller.hpp"

#include <cmath>
#include <iostream>

int main() {
    if (std::fabs(gimbal::AnglePidController::wrapErrorDeg(358.0F) + 2.0F) > 1.0e-5F ||
        std::fabs(gimbal::AnglePidController::wrapErrorDeg(-358.0F) - 2.0F) > 1.0e-5F) {
        std::cerr << "yaw 跨越 ±180° 误差计算失败\n";
        return 1;
    }
    gimbal::PidConfig config{100.0F, 0.0F, 0.0F, 5.0F, 0.1F, 0.0F, 10000.0F};
    gimbal::AnglePidController controller(config);
    float output = 0.0F;
    for (int i = 0; i < 100; ++i) output = controller.update(90.0F, 0.0F, 0.01, false);
    if (!std::isfinite(output) || std::fabs(output) > 5.0001F) {
        std::cerr << "PID 输出限幅失败\n";
        return 1;
    }
    gimbal::PidConfig integral_config{0.0F, 1.0F, 0.0F, 100.0F, 0.1F, 0.0F, 10000.0F};
    gimbal::AnglePidController integral_controller(integral_config);
    for (int i = 0; i < 100; ++i) {
        (void)integral_controller.update(1.0F, 0.0F, 0.01, false);
    }
    if (std::fabs(integral_controller.integralState() - 0.1F) > 1.0e-4F) {
        std::cerr << "PID 积分限幅失败\n";
        return 1;
    }
    controller.reset();
    const float wrapped = controller.update(-179.0F, 179.0F, 0.01, true);
    if (!std::isfinite(wrapped) || std::fabs(wrapped) > 5.0001F) {
        std::cerr << "PID yaw 跨越处理或限幅失败\n";
        return 1;
    }
    gimbal::PidConfig damping_config{1.0F, 0.0F, 2.0F, 100.0F, 0.0F,
                                      0.0F, 10000.0F, true};
    gimbal::AnglePidController damping(damping_config);
    const float positive_rate = damping.update(10.0F, 0.0F, 0.01, false, 2.0F);
    damping.reset();
    const float negative_rate = damping.update(10.0F, 0.0F, 0.01, false, -2.0F);
    if (std::fabs(positive_rate - 6.0F) > 1.0e-4F ||
        std::fabs(negative_rate - 14.0F) > 1.0e-4F) {
        std::cerr << "测量角速度阻尼方向错误\n";
        return 1;
    }
    gimbal::PidConfig no_kick_config{0.0F, 0.0F, 50.0F, 100.0F, 0.0F,
                                     0.0F, 10000.0F, true};
    gimbal::AnglePidController no_kick(no_kick_config);
    if (std::fabs(no_kick.update(90.0F, 0.0F, 0.01, false, 0.0F)) > 1.0e-6F) {
        std::cerr << "目标阶跃产生了 D 项冲击\n";
        return 1;
    }
    return 0;
}
