#include "application.hpp"

#include "low_pass_filter.hpp"
#include "mahony_filter.hpp"
#include "pid_controller.hpp"
#include "run_mode_policy.hpp"
#include "signal_stop.hpp"
#include "uart_tx_state_machine.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <fstream>
#include <iostream>
#include <poll.h>
#include <sstream>
#include <thread>
#include <unistd.h>
#include <utility>

namespace gimbal {
namespace {

using Clock = std::chrono::steady_clock;

class DryRunWriter final : public IUartWriter {
public:
    WriteResult writeSome(const std::uint8_t*, std::size_t length) noexcept override {
        return {WriteStatus::progress, length};
    }
    WaitStatus waitWritable(int) noexcept override { return WaitStatus::ready; }
    void discardOutput() noexcept override {}
};

Clock::duration periodFromFrequency(double frequency_hz) {
    return std::chrono::duration_cast<Clock::duration>(std::chrono::duration<double>(1.0 / frequency_hz));
}

void updateMaximum(std::atomic<std::int64_t>& maximum, std::int64_t value) {
    auto current = maximum.load(std::memory_order_relaxed);
    while (value > current &&
           !maximum.compare_exchange_weak(current, value, std::memory_order_relaxed)) {}
}

} // namespace

Application::Application(AppConfig config, RunOptions options)
    : config_(std::move(config)), options_(options), axis_mapping_(config_.axis_mapping),
      safety_(config_.fault_recovery_success_frames) {
    target_yaw_deg_.store(options_.target_override ? options_.target_yaw_deg
                                                    : config_.default_target_yaw_deg);
    target_pitch_deg_.store(options_.target_override ? options_.target_pitch_deg
                                                      : config_.default_target_pitch_deg);
    shared_packet_.updated = Clock::now();
    auto_run_pending_.store(!options_.attitude_uart &&
                            (options_.auto_run || config_.auto_run));
}

Application::~Application() {
    requestStop();
    if (control_thread_.joinable()) control_thread_.join();
    if (uart_thread_.joinable()) uart_thread_.join();
    if (status_thread_.joinable()) status_thread_.join();
    if (input_thread_.joinable()) input_thread_.join();
}

void Application::setTargetAngles(float yaw_deg, float pitch_deg) noexcept {
    if (std::isfinite(yaw_deg) && std::isfinite(pitch_deg)) {
        target_yaw_deg_.store(yaw_deg);
        target_pitch_deg_.store(pitch_deg);
    }
}

bool Application::stopRequested() noexcept {
    if (signalStopRequested()) {
        exit_status_.record(ExitReason::signal_stop);
        requestStop();
    }
    return stop_requested_.load();
}

SafetyState Application::currentSafetyState() {
    std::lock_guard<std::mutex> lock(safety_mutex_);
    return safety_.state();
}

float Application::selectBodyRate(const Vector3& body_rate_dps, std::uint32_t axis,
                                  float sign) noexcept {
    const float values[3]{body_rate_dps.x, body_rate_dps.y, body_rate_dps.z};
    return axis < 3U ? values[axis] * sign : 0.0F;
}

bool Application::tryEnterRun(std::string& reason) {
    if (!modeCanEnterRunning(options_.attitude_uart)) {
        reason = "姿态串口模式只输出 yaw/pitch，不允许进入角度控制。";
        return false;
    }
    Attitude current{};
    {
        std::lock_guard<std::mutex> lock(attitude_mutex_);
        current = attitude_;
    }
    const double now_s = std::chrono::duration<double>(Clock::now().time_since_epoch()).count();
    const RunPreconditions conditions{
        current.valid,
        current.valid && now_s - current.timestamp_s >= 0.0 &&
            now_s - current.timestamp_s <= 5.0 / config_.imu_frequency_hz,
        !options_.imu_only && (options_.dry_run || uart_.isOpen()),
        std::isfinite(target_yaw_deg_.load()) && std::isfinite(target_pitch_deg_.load())};
    std::lock_guard<std::mutex> lock(safety_mutex_);
    if (safety_.requestRun(conditions)) {
        reason.clear();
        return true;
    }
    std::ostringstream message;
    message << "不能进入 RUNNING：当前状态=" << safety_.stateName();
    if (!conditions.attitude_valid) message << "，姿态尚未有效";
    else if (!conditions.imu_fresh) message << "，IMU 数据已超时";
    if (!conditions.uart_ready) message << "，UART 未打开";
    if (!conditions.targets_finite) message << "，目标值不是有限数";
    if ((safety_.state() == SafetyState::fault || safety_.state() == SafetyState::degraded) &&
        safety_.consecutiveSuccesses() < config_.fault_recovery_success_frames) {
        message << "，连续健康帧不足 " << config_.fault_recovery_success_frames;
    }
    reason = message.str();
    return false;
}

void Application::publishPacket(const GimbalPacketPayload& requested) {
    GimbalPacketPayload payload = requested;
    if (!std::isfinite(payload.yaw_value) || !std::isfinite(payload.pitch_value)) {
        payload = makeFaultPayload();
    }
    std::lock_guard<std::mutex> lock(command_mutex_);
    shared_packet_.payload = payload;
    shared_packet_.updated = Clock::now();
}

Application::SharedPacket Application::packetSnapshot() {
    std::lock_guard<std::mutex> lock(command_mutex_);
    return shared_packet_;
}

bool Application::calibrate(Bmi088& imu, CalibrationResult& result) {
    const auto period = periodFromFrequency(config_.imu_frequency_hz);
    const std::size_t required_samples = std::max<std::size_t>(
        10U, static_cast<std::size_t>(config_.calibration_seconds * config_.imu_frequency_hz));
    for (int attempt = 1; attempt <= 3 && !stopRequested(); ++attempt) {
        std::cout << "开始第 " << attempt << " 次静止校准（约 " << config_.calibration_seconds
                  << " 秒），请勿移动云台……" << std::endl;
        publishPacket(makeStopPayload());
        Vector3 gyro_mean{};
        Vector3 gyro_m2{};
        Vector3 accel_mean{};
        Vector3 accel_m2{};
        std::size_t count = 0U;
        auto next = Clock::now();
        bool read_failed = false;
        while (count < required_samples && !stopRequested()) {
            next += period;
            ImuSample sample{};
            if (!imu.readSample(sample)) {
                ++imu_failures_;
                if (imu.consecutiveFailures() >= config_.max_consecutive_spi_failures) {
                    std::cerr << "校准期间 SPI 连续读取失败：" << imu.lastError() << std::endl;
                    read_failed = true;
                    break;
                }
                std::this_thread::sleep_until(next);
                continue;
            }
            const Vector3 accel = axis_mapping_.apply({sample.ax, sample.ay, sample.az});
            const Vector3 gyro = axis_mapping_.apply({sample.gx, sample.gy, sample.gz});
            ++count;
            const float n = static_cast<float>(count);
            const Vector3 delta{gyro.x - gyro_mean.x, gyro.y - gyro_mean.y, gyro.z - gyro_mean.z};
            gyro_mean.x += delta.x / n;
            gyro_mean.y += delta.y / n;
            gyro_mean.z += delta.z / n;
            gyro_m2.x += delta.x * (gyro.x - gyro_mean.x);
            gyro_m2.y += delta.y * (gyro.y - gyro_mean.y);
            gyro_m2.z += delta.z * (gyro.z - gyro_mean.z);
            const Vector3 accel_delta{accel.x - accel_mean.x, accel.y - accel_mean.y,
                                      accel.z - accel_mean.z};
            accel_mean.x += accel_delta.x / n;
            accel_mean.y += accel_delta.y / n;
            accel_mean.z += accel_delta.z / n;
            accel_m2.x += accel_delta.x * (accel.x - accel_mean.x);
            accel_m2.y += accel_delta.y * (accel.y - accel_mean.y);
            accel_m2.z += accel_delta.z * (accel.z - accel_mean.z);
            std::this_thread::sleep_until(next);
        }
        if (stopRequested()) return false;
        if (read_failed || count < 2U) return false;
        const float divisor = static_cast<float>(count - 1U);
        const Vector3 stddev{std::sqrt(gyro_m2.x / divisor), std::sqrt(gyro_m2.y / divisor),
                             std::sqrt(gyro_m2.z / divisor)};
        const Vector3 accel_stddev{std::sqrt(accel_m2.x / divisor), std::sqrt(accel_m2.y / divisor),
                                   std::sqrt(accel_m2.z / divisor)};
        const float mean_rate = std::sqrt(gyro_mean.x * gyro_mean.x + gyro_mean.y * gyro_mean.y +
                                          gyro_mean.z * gyro_mean.z);
        const float maximum_stddev = std::max({stddev.x, stddev.y, stddev.z});
        const float maximum_accel_stddev = std::max({accel_stddev.x, accel_stddev.y, accel_stddev.z});
        const float accel_norm = std::sqrt(accel_mean.x * accel_mean.x + accel_mean.y * accel_mean.y +
                                           accel_mean.z * accel_mean.z);
        if (maximum_stddev > config_.calibration_max_gyro_stddev_dps ||
            mean_rate > config_.calibration_max_mean_rate_dps ||
            maximum_accel_stddev > config_.calibration_max_accel_stddev_g ||
            accel_norm < 0.8F || accel_norm > 1.2F) {
            std::cerr << "检测到云台运动：陀螺仪最大标准差 " << maximum_stddev
                      << " °/s，均值模长 " << mean_rate << " °/s，加速度最大标准差 "
                      << maximum_accel_stddev << " g，均值模长 " << accel_norm
                      << " g；请保持静止，准备重试。"
                      << std::endl;
            std::this_thread::sleep_for(std::chrono::seconds(1));
            continue;
        }
        result.gyro_bias_dps = gyro_mean;
        result.accel_mean_g = accel_mean;
        result.valid = true;
        std::cout << "校准完成：陀螺仪零偏 [" << gyro_mean.x << ", " << gyro_mean.y << ", "
                  << gyro_mean.z << "] °/s；加速度均值 [" << result.accel_mean_g.x << ", "
                  << result.accel_mean_g.y << ", " << result.accel_mean_g.z << "] g" << std::endl;
        return true;
    }
    std::cerr << "三次校准均检测到明显运动，程序保持停止状态。" << std::endl;
    return false;
}

bool Application::saveCalibration(const CalibrationResult& result) const {
    if (!config_.save_calibration) return true;
    std::ofstream file(config_.calibration_output_file, std::ios::trunc);
    if (!file) {
        std::cerr << "无法保存校准结果到 " << config_.calibration_output_file << std::endl;
        return false;
    }
    file << "# BMI088 启动静止校准结果，仅供记录和后续集成使用\n"
         << "gyro_bias_dps = " << result.gyro_bias_dps.x << ',' << result.gyro_bias_dps.y << ','
         << result.gyro_bias_dps.z << "\naccel_mean_g = " << result.accel_mean_g.x << ','
         << result.accel_mean_g.y << ',' << result.accel_mean_g.z << '\n';
    if (!file) {
        std::cerr << "写入校准结果失败：" << config_.calibration_output_file << std::endl;
        return false;
    }
    std::cout << "校准结果已保存到 " << config_.calibration_output_file << std::endl;
    return true;
}

void Application::controlLoop(Bmi088& imu, const CalibrationResult& calibration) {
    LowPassFilter accel_filters[3]{LowPassFilter(config_.accel_cutoff_hz),
                                   LowPassFilter(config_.accel_cutoff_hz),
                                   LowPassFilter(config_.accel_cutoff_hz)};
    LowPassFilter gyro_filters[3]{LowPassFilter(config_.gyro_cutoff_hz),
                                  LowPassFilter(config_.gyro_cutoff_hz),
                                  LowPassFilter(config_.gyro_cutoff_hz)};
    MahonyFilter attitude_filter({config_.mahony_kp, config_.mahony_ki,
                                  config_.mahony_accel_full_trust_error_g,
                                  config_.mahony_accel_reject_error_g,
                                  config_.mahony_integral_limit,
                                  config_.mahony_integral_decay_rate,
                                  config_.mahony_max_accel_reject_s});
    AnglePidController yaw_controller(config_.yaw_pid);
    AnglePidController pitch_controller(config_.pitch_pid);
    const auto period = periodFromFrequency(config_.imu_frequency_hz);
    const auto timeout = period * 5;
    auto next = Clock::now();
    double previous_timestamp_s = 0.0;
    bool was_running = false;

    while (!stopRequested()) {
        const auto cycle_start = Clock::now();
        if (cycle_start > next) {
            const auto late = std::chrono::duration_cast<std::chrono::microseconds>(cycle_start - next).count();
            updateMaximum(max_imu_lateness_us_, late);
        }
        next += period;
        ImuSample sample{};
        if (!imu.readSample(sample)) {
            ++imu_failures_;
            const bool threshold = imu.consecutiveFailures() >= config_.max_consecutive_spi_failures;
            if (threshold) exit_status_.record(ExitReason::runtime_failure);
            {
                std::lock_guard<std::mutex> lock(safety_mutex_);
                safety_.sampleFailure(FaultReason::spi_read, threshold);
            }
            publishPacket(threshold ? makeFaultPayload() : makeStopPayload());
            if (imu.consecutiveFailures() == config_.max_consecutive_spi_failures) {
                std::cerr << "SPI 连续读取失败达到安全阈值：" << imu.lastError() << std::endl;
            }
            std::this_thread::sleep_until(next);
            continue;
        }
        double dt_s = previous_timestamp_s > 0.0 ? sample.timestamp_s - previous_timestamp_s
                                                  : 1.0 / config_.imu_frequency_hz;
        previous_timestamp_s = sample.timestamp_s;
        if (!std::isfinite(dt_s) || dt_s <= 0.0 || dt_s > 5.0 / config_.imu_frequency_hz) {
            ++imu_failures_;
            exit_status_.record(ExitReason::runtime_failure);
            attitude_filter.reset();
            for (auto& filter : accel_filters) filter.reset();
            for (auto& filter : gyro_filters) filter.reset();
            yaw_controller.reset();
            pitch_controller.reset();
            {
                std::lock_guard<std::mutex> lock(safety_mutex_);
                safety_.sampleFailure(FaultReason::sample_timeout, true);
            }
            publishPacket(makeFaultPayload());
            std::this_thread::sleep_until(next);
            continue;
        }
        const Vector3 mapped_accel = axis_mapping_.apply({sample.ax, sample.ay, sample.az});
        Vector3 mapped_gyro = axis_mapping_.apply({sample.gx, sample.gy, sample.gz});
        mapped_gyro.x -= calibration.gyro_bias_dps.x;
        mapped_gyro.y -= calibration.gyro_bias_dps.y;
        mapped_gyro.z -= calibration.gyro_bias_dps.z;
        const Vector3 filtered_accel{accel_filters[0].update(mapped_accel.x, dt_s),
                                     accel_filters[1].update(mapped_accel.y, dt_s),
                                     accel_filters[2].update(mapped_accel.z, dt_s)};
        const Vector3 filtered_gyro{gyro_filters[0].update(mapped_gyro.x, dt_s),
                                    gyro_filters[1].update(mapped_gyro.y, dt_s),
                                    gyro_filters[2].update(mapped_gyro.z, dt_s)};
        Attitude current{};
        current.timestamp_s = sample.timestamp_s;
        if (!attitude_filter.update(filtered_accel, filtered_gyro, dt_s, current)) {
            ++imu_failures_;
            {
                std::lock_guard<std::mutex> lock(safety_mutex_);
                safety_.sampleFailure(FaultReason::attitude_invalid, false);
            }
            publishPacket(makeStopPayload());
            std::this_thread::sleep_until(next);
            continue;
        }
        {
            std::lock_guard<std::mutex> lock(attitude_mutex_);
            attitude_ = current;
        }
        ++imu_iterations_;
        {
            std::lock_guard<std::mutex> lock(safety_mutex_);
            if (attitude_filter.attitudeDegraded()) {
                safety_.attitudeDegraded(FaultReason::accel_rejected);
            } else {
                safety_.sampleSuccess();
            }
        }
        if (auto_run_pending_.load()) {
            std::string reason;
            if (tryEnterRun(reason)) {
                auto_run_pending_.store(false);
                std::cout << "--auto-run 已在姿态有效后显式启用角度外环。" << std::endl;
            }
        }

        const SafetyState safety_state = currentSafetyState();
        const bool running = safety_state == SafetyState::running &&
                             !options_.imu_only && !options_.attitude_uart;
        if (running != was_running) {
            yaw_controller.reset(current.yaw_deg);
            pitch_controller.reset(current.pitch_deg);
            was_running = running;
        }
        const double now_s =
            std::chrono::duration<double>(Clock::now().time_since_epoch()).count();
        const PacketDecision attitude_decision = decideAttitudePublication(
            {options_.attitude_uart, true,
             attitude_filter.attitudeDegraded() || Clock::now() - cycle_start > timeout,
             safety_state, current, now_s, config_.command_timeout_s});
        if (options_.attitude_uart) {
            publishPacket(attitude_decision.should_send
                              ? attitude_decision.payload
                              : makeStopPayload());
        } else if (running && !attitude_filter.attitudeDegraded() &&
                   Clock::now() - cycle_start <= timeout) {
            const float yaw_body_rate = selectBodyRate(filtered_gyro, config_.yaw_rate_body_axis,
                                                        config_.yaw_rate_sign);
            const float pitch_body_rate = selectBodyRate(filtered_gyro, config_.pitch_rate_body_axis,
                                                          config_.pitch_rate_sign);
            const float yaw_rate = yaw_controller.update(target_yaw_deg_.load(), current.yaw_deg,
                                                         dt_s, true, yaw_body_rate);
            const float pitch_rate = pitch_controller.update(target_pitch_deg_.load(), current.pitch_deg,
                                                             dt_s, false, pitch_body_rate);
            publishPacket(makeRateControlPayload(0U, yaw_rate, pitch_rate));
        } else {
            publishPacket(safety_state == SafetyState::fault
                              ? makeFaultPayload()
                              : makeStopPayload());
        }
        std::this_thread::sleep_until(next);
        // 极端过载时重新锚定，避免无意义地连续追赶数百个过期周期。
        if (Clock::now() - next > timeout) next = Clock::now();
    }
    publishPacket(makeStopPayload());
}

void Application::uartLoop() {
    const auto period = periodFromFrequency(config_.uart_frequency_hz);
    auto next = Clock::now();
    SteadyMonotonicClock transmitter_clock;
    UartTxStateMachine transmitter(
        std::chrono::duration_cast<std::chrono::microseconds>(
            std::chrono::duration<double>(config_.uart_frame_deadline_s)),
        static_cast<int>(config_.uart_poll_timeout_ms), transmitter_clock);
    DryRunWriter dry_writer;
    IUartWriter& writer = options_.dry_run ? static_cast<IUartWriter&>(dry_writer)
                                           : static_cast<IUartWriter&>(uart_);
    while (!stopRequested()) {
        const auto now = Clock::now();
        if (now > next) {
            updateMaximum(max_uart_lateness_us_,
                std::chrono::duration_cast<std::chrono::microseconds>(now - next).count());
        }
        next += period;
        const SharedPacket latest = packetSnapshot();
        const PacketDecision decision = decideUartTransmission(
            options_.attitude_uart, latest.payload, latest.updated, now,
            config_.command_timeout_s);
        if (!decision.should_send) {
            // 校准期间、姿态无效、降级或数据超时时不发送旧姿态。
            std::this_thread::sleep_until(next);
            if (Clock::now() - next > period * 5) next = Clock::now();
            continue;
        }
        const bool substituted =
            decision.payload.mode != latest.payload.mode ||
            decision.payload.yaw_value != latest.payload.yaw_value ||
            decision.payload.pitch_value != latest.payload.pitch_value;
        transmitter.setLatest(decision.payload, substituted ? now : latest.updated);
        const TxEvent event = transmitter.process(writer);
        if (event == TxEvent::frame_complete || event == TxEvent::stop_recovered) {
            ++uart_packets_;
        } else if (event == TxEvent::would_block) {
            ++uart_would_block_;
        } else if (event == TxEvent::io_error || event == TxEvent::deadline_timeout) {
            ++uart_failures_;
            const UartFailurePolicy failure_policy =
                uartFailurePolicy(options_.attitude_uart);
            if (failure_policy.record_failure_exit) {
                exit_status_.record(ExitReason::runtime_failure);
            }
            if (event == TxEvent::deadline_timeout) ++uart_deadlines_;
            {
                std::lock_guard<std::mutex> lock(safety_mutex_);
                safety_.uartFault(event == TxEvent::deadline_timeout
                                      ? FaultReason::uart_deadline : FaultReason::uart_io);
            }
            if (failure_policy.request_process_stop) {
                requestStop();
            } else if (failure_policy.publish_fault_packet) {
                publishPacket(makeFaultPayload());
            }
        }
        std::this_thread::sleep_until(next);
        if (Clock::now() - next > period * 5) next = Clock::now();
    }
}

void Application::statusLoop() {
    const auto interval = std::chrono::duration<double>(1.0 / config_.log_frequency_hz);
    std::uint64_t last_imu = 0U;
    std::uint64_t last_uart = 0U;
    while (!stopRequested()) {
        std::this_thread::sleep_for(interval);
        if (stopRequested()) break;
        const auto imu_count = imu_iterations_.load();
        const auto uart_count = uart_packets_.load();
        Attitude current{};
        {
            std::lock_guard<std::mutex> lock(attitude_mutex_);
            current = attitude_;
        }
        std::string state_name;
        std::string fault_name;
        std::uint32_t healthy_frames = 0U;
        {
            std::lock_guard<std::mutex> lock(safety_mutex_);
            state_name = safety_.stateName();
            fault_name = safety_.faultName();
            healthy_frames = safety_.consecutiveSuccesses();
        }
        std::cout << "状态：" << state_name << "，最后故障=" << fault_name
                  << "，连续健康帧=" << healthy_frames << "；IMU "
                  << (imu_count - last_imu) * config_.log_frequency_hz
                  << " Hz，UART " << (uart_count - last_uart) * config_.log_frequency_hz
                  << " Hz，姿态 [yaw=" << current.yaw_deg << "°, pitch=" << current.pitch_deg
                  << "°, roll=" << current.roll_deg << "°]，累计失败 [SPI="
                  << imu_failures_.load() << ", UART=" << uart_failures_.load()
                  << ", EAGAIN=" << uart_would_block_.load()
                  << ", 截止超时=" << uart_deadlines_.load()
                  << "]，最大延迟 [IMU=" << max_imu_lateness_us_.exchange(0)
                  << " us, UART=" << max_uart_lateness_us_.exchange(0) << " us]" << std::endl;
        last_imu = imu_count;
        last_uart = uart_count;
    }
}

void Application::inputLoop() {
    if (options_.attitude_uart) {
        std::cout << "姿态串口模式终端命令：status、quit" << std::endl;
    } else {
        std::cout << "终端命令：target <yaw_deg> <pitch_deg>、run、stop、reset-fault、status、quit"
                  << std::endl;
    }
    while (!stopRequested()) {
        pollfd descriptor{STDIN_FILENO, POLLIN, 0};
        const int result = ::poll(&descriptor, 1U, 200);
        if (result < 0) continue;
        if (result == 0) continue;
        if ((descriptor.revents & (POLLHUP | POLLERR | POLLNVAL)) != 0) return;
        if ((descriptor.revents & POLLIN) == 0) continue;
        std::string line;
        if (!std::getline(std::cin, line)) return;
        std::istringstream command(line);
        std::string name;
        command >> name;
        const OperatorCommand operator_command = classifyOperatorCommand(name);
        if (options_.attitude_uart &&
            !operatorCommandAllowed(true, operator_command)) {
            std::cerr << "姿态串口模式只允许 status、quit，不发送任何电机控制命令。"
                      << std::endl;
            continue;
        }
        if (name == "target") {
            float yaw = 0.0F;
            float pitch = 0.0F;
            if (command >> yaw >> pitch && std::isfinite(yaw) && std::isfinite(pitch)) {
                setTargetAngles(yaw, pitch);
                std::cout << "目标已更新为 yaw=" << yaw << "°，pitch=" << pitch << "°" << std::endl;
            } else {
                std::cerr << "用法：target <yaw_deg> <pitch_deg>" << std::endl;
            }
        } else if (name == "run") {
            if (options_.uart_test) {
                std::lock_guard<std::mutex> lock(safety_mutex_);
                if (safety_.requestRun({true, true, true, true}))
                    std::cout << "UART 测试角速度已重新启用。" << std::endl;
            } else {
                std::string reason;
                if (tryEnterRun(reason)) std::cout << "角度外环已启用。" << std::endl;
                else std::cerr << reason << std::endl;
            }
        } else if (name == "stop") {
            {
                std::lock_guard<std::mutex> lock(safety_mutex_);
                safety_.requestStop();
            }
            publishPacket(makeStopPayload());
            std::cout << "已切换到停止模式。" << std::endl;
        } else if (name == "reset-fault") {
            std::lock_guard<std::mutex> lock(safety_mutex_);
            if (safety_.resetFault()) {
                std::cout << "故障锁存已清除，仍保持 STOP；确认安全后输入 run。" << std::endl;
            } else {
                std::cerr << "尚未达到连续健康帧恢复条件，不能清除故障。" << std::endl;
            }
        } else if (name == "status") {
            if (options_.attitude_uart) {
                Attitude current{};
                {
                    std::lock_guard<std::mutex> attitude_lock(attitude_mutex_);
                    current = attitude_;
                }
                std::lock_guard<std::mutex> safety_lock(safety_mutex_);
                std::cout << "状态=" << safety_.stateName()
                          << "，姿态 [yaw=" << current.yaw_deg
                          << "°, pitch=" << current.pitch_deg
                          << "°, roll=" << current.roll_deg
                          << "°]，姿态串口包数=" << uart_packets_.load() << std::endl;
                continue;
            }
            const SharedPacket current = packetSnapshot();
            std::lock_guard<std::mutex> lock(safety_mutex_);
            std::cout << "状态=" << safety_.stateName() << "，最后故障=" << safety_.faultName()
                      << "，连续健康帧=" << safety_.consecutiveSuccesses()
                      << "；目标 yaw=" << target_yaw_deg_.load() << "°，pitch="
                      << target_pitch_deg_.load() << "°；包模式="
                      << static_cast<unsigned>(current.payload.mode) << std::endl;
        } else if (name == "quit" || name == "exit") {
            exit_status_.record(ExitReason::user_stop);
            requestStop();
        } else if (!name.empty()) {
            std::cerr << "未知命令：" << name << std::endl;
        }
    }
}

void Application::sendShutdownPackets() {
    if (!uart_.isOpen() || options_.dry_run || options_.imu_only) return;
    for (const GimbalPacketPayload& payload : makeShutdownPayloads()) {
        const auto packet = serializePacket(payload);
        std::size_t offset = 0U;
        for (int attempt = 0; offset < packet.size() && attempt < 20; ++attempt) {
            const WriteResult result = uart_.writeSome(packet.data() + offset,
                                                       packet.size() - offset);
            if (result.status == WriteStatus::progress) offset += result.bytes;
            else if (result.status == WriteStatus::error) break;
            else (void)uart_.waitWritable(1);
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(2));
    }
}

int Application::run() {
    Bmi088 imu({config_.accel_spi_device, config_.gyro_spi_device, config_.spi_speed_hz,
                config_.accel_conf, config_.gyro_bandwidth});

    if (options_.uart_test) {
        const float safety_limit = 20.0F;
        const float yaw_rate = std::clamp(config_.uart_test_yaw_rate_dps, -safety_limit, safety_limit);
        const float pitch_rate = std::clamp(config_.uart_test_pitch_rate_dps, -safety_limit, safety_limit);
        std::cerr << "警告：UART 测试模式不读取 BMI088，将发送固定小角速度 yaw=" << yaw_rate
                  << " °/s、pitch=" << pitch_rate << " °/s；请确保机构安全。" << std::endl;
        if (!options_.dry_run && !uart_.openPort(config_.uart_device, config_.uart_baud)) {
            std::cerr << uart_.lastError() << std::endl;
            exit_status_.record(ExitReason::uart_initialization_failure);
            return exit_status_.code();
        }
        if (!options_.dry_run) {
            std::cout << "已打开 UART：" << uart_.requestedDevice() << " -> "
                      << uart_.resolvedDevice() << std::endl;
        }
        {
            std::lock_guard<std::mutex> lock(safety_mutex_);
            safety_.beginCalibration();
            safety_.calibrationSucceeded();
            (void)safety_.requestRun({true, true, true, true});
        }
        publishPacket(makeRateControlPayload(0U, yaw_rate, pitch_rate));
        uart_thread_ = std::thread(&Application::uartLoop, this);
        status_thread_ = std::thread(&Application::statusLoop, this);
        input_thread_ = std::thread(&Application::inputLoop, this);
        while (!stopRequested()) {
            if (currentSafetyState() == SafetyState::running) {
                publishPacket(makeRateControlPayload(0U, yaw_rate, pitch_rate));
            } else {
                publishPacket(makeStopPayload());
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
        }
    } else {
        if (!imu.initialize()) {
            std::cerr << "BMI088 初始化失败：" << imu.lastError()
                      << "。未发送任何有效运动命令。" << std::endl;
            exit_status_.record(ExitReason::bmi_initialization_failure);
            return exit_status_.code();
        }
        std::cout << "SPI 实际速度：加速度计 " << imu.actualAccelSpeedHz()
                  << " Hz，陀螺仪 " << imu.actualGyroSpeedHz() << " Hz";
        if (imu.actualAccelSpeedHz() != config_.spi_speed_hz ||
            imu.actualGyroSpeedHz() != config_.spi_speed_hz) {
            std::cout << "（与请求的 " << config_.spi_speed_hz << " Hz 不同）";
        }
        std::cout << std::endl;
        if (!options_.imu_only && !options_.dry_run &&
            !uart_.openPort(config_.uart_device, config_.uart_baud)) {
            std::cerr << uart_.lastError() << "。未发送任何有效运动命令。" << std::endl;
            exit_status_.record(ExitReason::uart_initialization_failure);
            return exit_status_.code();
        }
        if (!options_.imu_only && !options_.dry_run) {
            std::cout << "已打开 UART：" << uart_.requestedDevice() << " -> "
                      << uart_.resolvedDevice() << std::endl;
        }
        {
            std::lock_guard<std::mutex> lock(safety_mutex_);
            safety_.beginCalibration();
        }
        if (!options_.imu_only) {
            uart_thread_ = std::thread(&Application::uartLoop, this);
        }
        status_thread_ = std::thread(&Application::statusLoop, this);
        input_thread_ = std::thread(&Application::inputLoop, this);
        CalibrationResult calibration{};
        if (!calibrate(imu, calibration)) {
            if (!stop_requested_.load()) {
                exit_status_.record(ExitReason::calibration_failure);
                {
                    std::lock_guard<std::mutex> lock(safety_mutex_);
                    safety_.calibrationFailed();
                }
                publishPacket(makeFaultPayload());
                requestStop();
            }
        } else {
            (void)saveCalibration(calibration);
            {
                std::lock_guard<std::mutex> lock(safety_mutex_);
                safety_.calibrationSucceeded();
            }
            control_thread_ = std::thread(&Application::controlLoop, this, std::ref(imu), calibration);
            if (options_.attitude_uart) {
                std::cout << "姿态串口模式已启动：复用原 10 字节帧，mode=2，"
                             "第 5–6 字节为 yaw，第 7–8 字节为 pitch；不运行 PID。"
                          << std::endl;
            } else {
                std::cout << "校准完成后保持 STOP；输入 run 才会启用电机角速度命令。"
                          << std::endl;
            }
        }
        while (!stopRequested()) {
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
        }
    }

    requestStop();
    if (control_thread_.joinable()) control_thread_.join();
    if (uart_thread_.joinable()) uart_thread_.join();
    if (status_thread_.joinable()) status_thread_.join();
    if (input_thread_.joinable()) input_thread_.join();
    sendShutdownPackets();
    uart_.close();
    imu.close();
    std::cout << "程序已安全停止。" << std::endl;
    return exit_status_.code();
}

} // namespace gimbal
