#pragma once

#include "bmi088.hpp"
#include "config.hpp"
#include "exit_status.hpp"
#include "gimbal_protocol.hpp"
#include "imu_types.hpp"
#include "safety_state.hpp"
#include "uart_port.hpp"

#include <array>
#include <atomic>
#include <chrono>
#include <cstdint>
#include <mutex>
#include <string>
#include <thread>

namespace gimbal {

struct RunOptions {
    bool imu_only{false};
    bool uart_test{false};
    bool attitude_uart{false};
    bool dry_run{false};
    bool auto_run{false};
    bool target_override{false};
    float target_yaw_deg{0.0F};
    float target_pitch_deg{0.0F};
};

class Application {
public:
    Application(AppConfig config, RunOptions options);
    ~Application();
    int run();
    void requestStop() noexcept { stop_requested_.store(true); }
    // 后续视觉模块可从独立线程调用此接口，无需改动控制循环。
    void setTargetAngles(float yaw_deg, float pitch_deg) noexcept;

private:
    struct SharedPacket {
        GimbalPacketPayload payload{};
        std::chrono::steady_clock::time_point updated{};
    };

    bool calibrate(Bmi088& imu, CalibrationResult& result);
    bool saveCalibration(const CalibrationResult& result) const;
    void controlLoop(Bmi088& imu, const CalibrationResult& calibration);
    void uartLoop();
    void statusLoop();
    void inputLoop();
    void publishPacket(const GimbalPacketPayload& payload);
    SharedPacket packetSnapshot();
    void sendShutdownPackets();
    bool stopRequested() noexcept;
    bool tryEnterRun(std::string& reason);
    SafetyState currentSafetyState();
    static float selectBodyRate(const Vector3& body_rate_dps, std::uint32_t axis,
                                float sign) noexcept;

    AppConfig config_;
    RunOptions options_;
    AxisMapping axis_mapping_;
    UartPort uart_;
    SafetyStateMachine safety_;
    std::atomic<bool> stop_requested_{false};
    std::atomic<bool> auto_run_pending_{false};
    std::atomic<float> target_yaw_deg_{0.0F};
    std::atomic<float> target_pitch_deg_{0.0F};
    std::mutex command_mutex_;
    SharedPacket shared_packet_{};
    std::mutex attitude_mutex_;
    Attitude attitude_{};
    std::mutex safety_mutex_;
    std::thread control_thread_;
    std::thread uart_thread_;
    std::thread status_thread_;
    std::thread input_thread_;
    std::atomic<std::uint64_t> imu_iterations_{0U};
    std::atomic<std::uint64_t> imu_failures_{0U};
    std::atomic<std::uint64_t> uart_packets_{0U};
    std::atomic<std::uint64_t> uart_failures_{0U};
    std::atomic<std::uint64_t> uart_would_block_{0U};
    std::atomic<std::uint64_t> uart_deadlines_{0U};
    std::atomic<std::int64_t> max_imu_lateness_us_{0};
    std::atomic<std::int64_t> max_uart_lateness_us_{0};
    ExitStatus exit_status_{};
};

} // namespace gimbal
