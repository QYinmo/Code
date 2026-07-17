#pragma once

#include "imu_types.hpp"

#include <cstddef>
#include <cstdint>
#include <string>

namespace gimbal {

struct Bmi088Config {
    std::string accel_device;
    std::string gyro_device;
    std::uint32_t speed_hz{1'000'000U};
    std::uint8_t accel_conf{0xABU};
    std::uint8_t gyro_bandwidth{0x02U};
};

class Bmi088 {
public:
    explicit Bmi088(Bmi088Config config);
    ~Bmi088();
    Bmi088(const Bmi088&) = delete;
    Bmi088& operator=(const Bmi088&) = delete;

    bool initialize();
    bool readSample(ImuSample& sample);
    void close() noexcept;
    [[nodiscard]] const std::string& lastError() const noexcept { return last_error_; }
    [[nodiscard]] std::uint32_t consecutiveFailures() const noexcept { return consecutive_failures_; }
    [[nodiscard]] std::uint32_t actualAccelSpeedHz() const noexcept { return accel_actual_speed_hz_; }
    [[nodiscard]] std::uint32_t actualGyroSpeedHz() const noexcept { return gyro_actual_speed_hz_; }

    static std::int16_t decodeI16Le(const std::uint8_t* bytes) noexcept;
    static float accelRawToG(std::int16_t raw) noexcept;
    static float gyroRawToDps(std::int16_t raw) noexcept;
    static float temperatureRawToC(std::uint8_t msb, std::uint8_t lsb, bool& valid) noexcept;

private:
    enum class Target { accel, gyro };
    bool openAndConfigure(const std::string& path, int& fd, std::uint32_t& actual_speed_hz);
    bool readRegisters(Target target, std::uint8_t reg, std::uint8_t* data, std::size_t length);
    bool writeRegister(Target target, std::uint8_t reg, std::uint8_t value);
    bool writeAndVerify(Target target, std::uint8_t reg, std::uint8_t value, std::uint8_t mask);
    bool transfer(int fd, const std::uint8_t* tx, std::uint8_t* rx, std::size_t length);
    void setError(const std::string& message);

    Bmi088Config config_;
    int accel_fd_{-1};
    int gyro_fd_{-1};
    bool initialized_{false};
    std::uint32_t consecutive_failures_{0U};
    std::uint32_t suspicious_gyro_frames_{0U};
    std::uint32_t accel_actual_speed_hz_{0U};
    std::uint32_t gyro_actual_speed_hz_{0U};
    std::string last_error_;
};

} // namespace gimbal
