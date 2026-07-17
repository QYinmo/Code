#include "mahony_filter.hpp"

#include <cmath>
#include <iostream>
#include <limits>

namespace {

bool near(float lhs, float rhs, float tolerance) {
    return std::fabs(lhs - rhs) <= tolerance;
}

} // namespace

int main() {
    const gimbal::MahonyConfig config{1.5F, 0.2F, 0.08F, 0.30F, 0.03F, 1.0F, 0.30};
    gimbal::MahonyFilter filter(config);
    gimbal::Attitude attitude{};
    for (int i = 0; i < 1000; ++i) {
        if (!filter.update({0.0F, 0.0F, 1.0F}, {}, 0.002, attitude)) {
            std::cerr << "静止 1 g 更新失败\n";
            return 1;
        }
    }
    if (!near(filter.accelWeight(), 1.0F, 1.0e-6F) ||
        !near(filter.quaternionNorm(), 1.0F, 1.0e-5F)) {
        std::cerr << "静止可信度或四元数归一化失败\n";
        return 1;
    }

    for (int i = 0; i < 20; ++i) {
        if (!filter.update({0.0F, 0.0F, 1.5F}, {0.0F, 10.0F, 0.0F}, 0.002, attitude)) {
            std::cerr << "短时 1.5 g 应允许陀螺仪传播\n";
            return 1;
        }
    }
    if (filter.accelWeight() != 0.0F || filter.attitudeDegraded()) {
        std::cerr << "短时动态加速度门控或降级时间失败\n";
        return 1;
    }
    if (!filter.update({}, {}, 0.002, attitude) || filter.accelWeight() != 0.0F) {
        std::cerr << "零加速度应进入纯陀螺仪传播\n";
        return 1;
    }
    const float nan = std::numeric_limits<float>::quiet_NaN();
    if (filter.update({nan, 0.0F, 1.0F}, {}, 0.002, attitude) ||
        filter.update({0.0F, 0.0F, 1.0F}, {}, 0.0, attitude) ||
        filter.update({0.0F, 0.0F, 1.0F}, {}, std::numeric_limits<double>::infinity(), attitude)) {
        std::cerr << "NaN 或无效 dt 未被拒绝\n";
        return 1;
    }
    if (!filter.update({0.0F, 0.0F, 1.0F}, {}, 1.0, attitude)) {
        std::cerr << "有限异常 dt 应被有界限幅处理\n";
        return 1;
    }

    gimbal::MahonyFilter integral_filter({0.0F, 10.0F, 0.08F, 0.30F,
                                          0.01F, 0.0F, 0.30});
    for (int i = 0; i < 500; ++i) {
        if (!integral_filter.update({0.7071067F, 0.0F, 0.7071067F}, {}, 0.002, attitude)) {
            return 1;
        }
    }
    const auto integral = integral_filter.integralState();
    if (std::fabs(integral.x) > 0.01001F || std::fabs(integral.y) > 0.01001F ||
        std::fabs(integral.z) > 0.01001F) {
        std::cerr << "Mahony 积分限幅失败\n";
        return 1;
    }
    for (int i = 0; i < 200; ++i) {
        (void)filter.update({0.0F, 0.0F, 1.5F}, {}, 0.002, attitude);
    }
    if (!filter.attitudeDegraded() || !near(filter.quaternionNorm(), 1.0F, 1.0e-5F)) {
        std::cerr << "连续拒绝降级统计或持续归一化失败\n";
        return 1;
    }
    return 0;
}
