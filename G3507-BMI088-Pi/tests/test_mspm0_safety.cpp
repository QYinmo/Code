#include "gimbal_protocol.hpp"

extern "C" {
#include "gimbal_safety_monitor.h"
}

#include <cstdint>
#include <iostream>

namespace {

GimbalDecodedPacket decode(const gimbal::GimbalPacketPayload& payload) {
    const auto bytes = gimbal::serializePacket(payload);
    GimbalDecodedPacket decoded{};
    if (!GimbalProtocol_Parse(bytes.data(), &decoded)) {
        std::cerr << "测试包解析失败\n";
    }
    return decoded;
}

bool targetEquals(const GimbalSafetyMonitor& monitor,
                  std::int16_t yaw, std::int16_t pitch,
                  bool expected_active) {
    std::int16_t actual_yaw = 0;
    std::int16_t actual_pitch = 0;
    const bool active = GimbalSafetyMonitor_GetTarget(
        &monitor, &actual_yaw, &actual_pitch);
    return active == expected_active && actual_yaw == yaw &&
           actual_pitch == pitch;
}

} // namespace

int main() {
    const GimbalSafetyMonitorConfig valid_config{25U, 100U};
    const GimbalSafetyMonitorConfig invalid_config{100U, 100U};
    if (!GimbalSafetyMonitor_ValidateConfig(&valid_config) ||
        GimbalSafetyMonitor_ValidateConfig(&invalid_config)) {
        std::cerr << "MSPM0 超时配置约束错误\n";
        return 1;
    }

    GimbalSafetyMonitor monitor{};
    if (!GimbalSafetyMonitor_Init(&monitor, &valid_config)) return 1;

    const GimbalDecodedPacket rate =
        decode(gimbal::makeRateControlPayload(1U, 5.0F, -2.0F));
    GimbalRateControlValues rate_values{};
    GimbalAttitudeValues attitude_values{};
    if (!GimbalProtocol_GetRateControl(&rate, &rate_values) ||
        GimbalProtocol_GetAttitude(&rate, &attitude_values) ||
        rate_values.yaw_rate_cdeg_s != 500 ||
        rate_values.pitch_rate_cdeg_s != -200) {
        std::cerr << "MSPM0 mode 安全 accessor 错误\n";
        return 1;
    }

    if (GimbalSafetyMonitor_HandlePacket(&monitor, &rate, 1000U) !=
            GIMBAL_SAFETY_EVENT_RATE_UPDATED ||
        !targetEquals(monitor, 500, -200, true)) {
        std::cerr << "合法 RATE_CONTROL 未更新目标\n";
        return 1;
    }
    GimbalSafetyMonitor_Tick(&monitor, 1024U);
    if (!targetEquals(monitor, 500, -200, true)) {
        std::cerr << "目标超时前被过早归零\n";
        return 1;
    }
    GimbalSafetyMonitor_Tick(&monitor, 1025U);
    if (monitor.state != GIMBAL_SAFETY_TARGET_TIMEOUT ||
        !targetEquals(monitor, 0, 0, false)) {
        std::cerr << "25 ms 目标超时未归零\n";
        return 1;
    }
    GimbalSafetyMonitor_Tick(&monitor, 1100U);
    if (monitor.state != GIMBAL_SAFETY_COMMUNICATION_FAULT ||
        !targetEquals(monitor, 0, 0, false)) {
        std::cerr << "100 ms 未进入通信故障\n";
        return 1;
    }

    const GimbalDecodedPacket attitude =
        decode(gimbal::makeAttitudePayload(2U, 12.34F, -5.67F));
    if (!GimbalProtocol_GetAttitude(&attitude, &attitude_values) ||
        GimbalProtocol_GetRateControl(&attitude, &rate_values) ||
        attitude_values.yaw_cdeg != 1234 ||
        attitude_values.pitch_cdeg != -567 ||
        GimbalSafetyMonitor_HandlePacket(&monitor, &attitude, 1110U) !=
            GIMBAL_SAFETY_EVENT_ATTITUDE_IGNORED ||
        monitor.state != GIMBAL_SAFETY_COMMUNICATION_FAULT) {
        std::cerr << "mode 2 被误当角速度或错误恢复通信\n";
        return 1;
    }

    const GimbalDecodedPacket new_rate =
        decode(gimbal::makeRateControlPayload(3U, 1.0F, 2.0F));
    if (GimbalSafetyMonitor_HandlePacket(&monitor, &new_rate, 1120U) !=
            GIMBAL_SAFETY_EVENT_RATE_UPDATED ||
        !targetEquals(monitor, 100, 200, true)) {
        std::cerr << "通信恢复后明确新控制命令未生效\n";
        return 1;
    }

    const GimbalDecodedPacket stop = decode(gimbal::makeStopPayload(4U));
    if (GimbalSafetyMonitor_HandlePacket(&monitor, &stop, 1121U) !=
            GIMBAL_SAFETY_EVENT_STOPPED ||
        monitor.state != GIMBAL_SAFETY_STOPPED ||
        !targetEquals(monitor, 0, 0, false)) {
        std::cerr << "STOP 未立即归零\n";
        return 1;
    }

    const GimbalDecodedPacket fault = decode(gimbal::makeFaultPayload(5U));
    if (GimbalSafetyMonitor_HandlePacket(&monitor, &fault, 1122U) !=
            GIMBAL_SAFETY_EVENT_FAULT_LATCHED ||
        monitor.state != GIMBAL_SAFETY_REMOTE_FAULT ||
        !monitor.fault_latched) {
        std::cerr << "FAULT 未锁存安全停止\n";
        return 1;
    }
    if (GimbalSafetyMonitor_HandlePacket(&monitor, &new_rate, 1123U) !=
            GIMBAL_SAFETY_EVENT_FAULT_LATCHED ||
        targetEquals(monitor, 100, 200, true)) {
        std::cerr << "故障锁存后旧策略自动恢复非零目标\n";
        return 1;
    }
    GimbalSafetyMonitor_ClearFault(&monitor);
    if (monitor.state != GIMBAL_SAFETY_STOPPED ||
        !targetEquals(monitor, 0, 0, false)) {
        std::cerr << "清故障后未保持零目标 STOP\n";
        return 1;
    }

    GimbalDecodedPacket unknown{};
    unknown.mode = static_cast<GimbalMode>(9);
    unknown.yaw_value = 123;
    unknown.pitch_value = 456;
    if (GimbalSafetyMonitor_HandlePacket(&monitor, &unknown, 1200U) !=
            GIMBAL_SAFETY_EVENT_FAULT_LATCHED ||
        monitor.state != GIMBAL_SAFETY_REMOTE_FAULT ||
        !targetEquals(monitor, 0, 0, false)) {
        std::cerr << "未知 mode 未默认故障停止\n";
        return 1;
    }
    return 0;
}
