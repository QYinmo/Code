#include "bmi088.hpp"

#include <cerrno>
#include <chrono>
#include <cstring>
#include <fcntl.h>
#include <linux/spi/spidev.h>
#include <sstream>
#include <sys/ioctl.h>
#include <thread>
#include <unistd.h>
#include <utility>

namespace gimbal {
namespace {

constexpr std::uint8_t kReadBit = 0x80U;
constexpr std::uint8_t kAddressMask = 0x7FU;
constexpr std::uint8_t kAccelChipIdReg = 0x00U;
constexpr std::uint8_t kAccelDataReg = 0x12U;
constexpr std::uint8_t kAccelTempReg = 0x22U;
constexpr std::uint8_t kAccelConfReg = 0x40U;
constexpr std::uint8_t kAccelRangeReg = 0x41U;
constexpr std::uint8_t kAccelPowerConfReg = 0x7CU;
constexpr std::uint8_t kAccelPowerCtrlReg = 0x7DU;
constexpr std::uint8_t kAccelSoftResetReg = 0x7EU;
constexpr std::uint8_t kGyroChipIdReg = 0x00U;
constexpr std::uint8_t kGyroDataReg = 0x02U;
constexpr std::uint8_t kGyroRangeReg = 0x0FU;
constexpr std::uint8_t kGyroBandwidthReg = 0x10U;
constexpr std::uint8_t kGyroLpm1Reg = 0x11U;
constexpr std::uint8_t kGyroSoftResetReg = 0x14U;
constexpr std::uint8_t kAccelChipId = 0x1EU;
constexpr std::uint8_t kGyroChipId = 0x0FU;
constexpr std::uint8_t kSoftReset = 0xB6U;
constexpr std::uint8_t kAccelRange6g = 0x01U;
constexpr std::uint8_t kGyroRange500dps = 0x02U;

void delayMs(int milliseconds) {
    std::this_thread::sleep_for(std::chrono::milliseconds(milliseconds));
}

} // namespace

Bmi088::Bmi088(Bmi088Config config) : config_(std::move(config)) {}

Bmi088::~Bmi088() { close(); }

void Bmi088::setError(const std::string& message) { last_error_ = message; }

bool Bmi088::openAndConfigure(const std::string& path, int& fd, std::uint32_t& actual_speed_hz) {
    fd = ::open(path.c_str(), O_RDWR | O_CLOEXEC);
    if (fd < 0) {
        setError("无法打开 SPI 设备 " + path + "：" + std::strerror(errno));
        return false;
    }
    std::uint8_t mode = SPI_MODE_0;
    std::uint8_t bits = 8U;
    std::uint32_t speed = config_.speed_hz;
    if (::ioctl(fd, SPI_IOC_WR_MODE, &mode) < 0 ||
        ::ioctl(fd, SPI_IOC_RD_MODE, &mode) < 0 ||
        ::ioctl(fd, SPI_IOC_WR_BITS_PER_WORD, &bits) < 0 ||
        ::ioctl(fd, SPI_IOC_RD_BITS_PER_WORD, &bits) < 0 ||
        ::ioctl(fd, SPI_IOC_WR_MAX_SPEED_HZ, &speed) < 0 ||
        ::ioctl(fd, SPI_IOC_RD_MAX_SPEED_HZ, &speed) < 0) {
        setError("配置 SPI 设备 " + path + " 失败：" + std::strerror(errno));
        ::close(fd);
        fd = -1;
        return false;
    }
    if (mode != SPI_MODE_0 || bits != 8U) {
        setError("SPI 驱动未接受 Mode 0 / 8 bit 配置：" + path);
        ::close(fd);
        fd = -1;
        return false;
    }
    actual_speed_hz = speed;
    return true;
}

bool Bmi088::transfer(int fd, const std::uint8_t* tx, std::uint8_t* rx, std::size_t length) {
    spi_ioc_transfer operation{};
    operation.tx_buf = reinterpret_cast<std::uintptr_t>(tx);
    operation.rx_buf = reinterpret_cast<std::uintptr_t>(rx);
    operation.len = static_cast<std::uint32_t>(length);
    operation.speed_hz = config_.speed_hz;
    operation.bits_per_word = 8U;
    if (::ioctl(fd, SPI_IOC_MESSAGE(1), &operation) < 1) {
        setError("SPI 传输失败：" + std::string(std::strerror(errno)));
        return false;
    }
    return true;
}

bool Bmi088::readRegisters(Target target, std::uint8_t reg, std::uint8_t* data,
                           std::size_t length) {
    constexpr std::size_t kMaximumRead = 16U;
    if (data == nullptr || length == 0U || length > kMaximumRead) {
        setError("BMI088 读取参数或长度无效");
        return false;
    }
    std::uint8_t tx[kMaximumRead + 2U]{};
    std::uint8_t rx[kMaximumRead + 2U]{};
    const std::size_t prefix = target == Target::accel ? 2U : 1U;
    tx[0] = static_cast<std::uint8_t>(reg | kReadBit);
    const int fd = target == Target::accel ? accel_fd_ : gyro_fd_;
    if (!transfer(fd, tx, rx, length + prefix)) {
        return false;
    }
    for (std::size_t index = 0U; index < length; ++index) {
        data[index] = rx[index + prefix];
    }
    return true;
}

bool Bmi088::writeRegister(Target target, std::uint8_t reg, std::uint8_t value) {
    const std::uint8_t tx[2]{static_cast<std::uint8_t>(reg & kAddressMask), value};
    std::uint8_t rx[2]{};
    const int fd = target == Target::accel ? accel_fd_ : gyro_fd_;
    if (!transfer(fd, tx, rx, sizeof(tx))) {
        return false;
    }
    delayMs(1);
    return true;
}

bool Bmi088::writeAndVerify(Target target, std::uint8_t reg, std::uint8_t value,
                            std::uint8_t mask) {
    std::uint8_t actual = 0U;
    if (!writeRegister(target, reg, value) || !readRegisters(target, reg, &actual, 1U)) {
        return false;
    }
    if ((actual & mask) != (value & mask)) {
        std::ostringstream stream;
        stream << "BMI088 寄存器 0x" << std::hex << static_cast<unsigned>(reg)
               << " 回读校验失败：写入 0x" << static_cast<unsigned>(value)
               << "，读到 0x" << static_cast<unsigned>(actual);
        setError(stream.str());
        return false;
    }
    return true;
}

bool Bmi088::initialize() {
    close();
    last_error_.clear();
    consecutive_failures_ = 0U;
    suspicious_gyro_frames_ = 0U;
    if (!openAndConfigure(config_.accel_device, accel_fd_, accel_actual_speed_hz_) ||
        !openAndConfigure(config_.gyro_device, gyro_fd_, gyro_actual_speed_hz_)) {
        close();
        return false;
    }

    // spidev 的硬件片选在无传输时自动释放；两个 fd 此刻均处于未选中状态。
    delayMs(1);
    std::uint8_t value = 0U;
    // 加速度计上电后的第一次读操作将通信接口切换到 SPI。
    if (!readRegisters(Target::accel, kAccelChipIdReg, &value, 1U) ||
        !writeRegister(Target::accel, kAccelSoftResetReg, kSoftReset)) {
        return false;
    }
    delayMs(2);
    // 软复位恢复上电状态，必须再次读取才能重新进入 SPI 模式。
    if (!readRegisters(Target::accel, kAccelChipIdReg, &value, 1U) ||
        !writeRegister(Target::gyro, kGyroSoftResetReg, kSoftReset)) {
        return false;
    }
    delayMs(30);

    if (!readRegisters(Target::accel, kAccelChipIdReg, &value, 1U)) return false;
    if (value != kAccelChipId) {
        setError("加速度计芯片 ID 错误：期望 0x1E，实际 0x" +
                 [&value] { std::ostringstream s; s << std::hex << static_cast<unsigned>(value); return s.str(); }());
        return false;
    }
    if (!readRegisters(Target::gyro, kGyroChipIdReg, &value, 1U)) return false;
    if (value != kGyroChipId) {
        setError("陀螺仪芯片 ID 错误：期望 0x0F，实际 0x" +
                 [&value] { std::ostringstream s; s << std::hex << static_cast<unsigned>(value); return s.str(); }());
        return false;
    }

    // Bosch 推荐顺序：先关闭高级省电，再等待 5 ms 后使能测量核心。
    if (!writeAndVerify(Target::accel, kAccelPowerConfReg, 0x00U, 0xFFU)) return false;
    delayMs(5);
    if (!writeAndVerify(Target::accel, kAccelPowerCtrlReg, 0x04U, 0xFFU)) return false;
    delayMs(50);
    if (!writeAndVerify(Target::gyro, kGyroLpm1Reg, 0x00U, 0xE0U)) return false;
    delayMs(30);

    // ACC_CONF: 0xA? 为 normal filter；默认低四位 0xB 表示 800 Hz ODR。
    if (!writeAndVerify(Target::accel, kAccelConfReg, config_.accel_conf, 0xFFU) ||
        !writeAndVerify(Target::accel, kAccelRangeReg, kAccelRange6g, 0x03U) ||
        !writeAndVerify(Target::gyro, kGyroRangeReg, kGyroRange500dps, 0x07U) ||
        !writeAndVerify(Target::gyro, kGyroBandwidthReg, config_.gyro_bandwidth, 0x07U)) {
        return false;
    }
    initialized_ = true;
    return true;
}

std::int16_t Bmi088::decodeI16Le(const std::uint8_t* bytes) noexcept {
    return static_cast<std::int16_t>(static_cast<std::uint16_t>(bytes[0]) |
                                     (static_cast<std::uint16_t>(bytes[1]) << 8U));
}

float Bmi088::accelRawToG(std::int16_t raw) noexcept {
    return static_cast<float>(raw) * (6.0F / 32768.0F);
}

float Bmi088::gyroRawToDps(std::int16_t raw) noexcept {
    return static_cast<float>(raw) * (500.0F / 32768.0F);
}

float Bmi088::temperatureRawToC(std::uint8_t msb, std::uint8_t lsb, bool& valid) noexcept {
    valid = msb != 0x80U;
    if (!valid) return 0.0F;
    const std::uint16_t raw = static_cast<std::uint16_t>((static_cast<std::uint16_t>(msb) << 3U) |
                                                         (static_cast<std::uint16_t>(lsb) >> 5U));
    const std::int16_t signed_raw = raw > 1023U ? static_cast<std::int16_t>(raw - 2048U)
                                                : static_cast<std::int16_t>(raw);
    return static_cast<float>(signed_raw) * 0.125F + 23.0F;
}

bool Bmi088::readSample(ImuSample& sample) {
    if (!initialized_) {
        setError("BMI088 尚未初始化");
        ++consecutive_failures_;
        return false;
    }
    std::uint8_t accel[6]{};
    std::uint8_t gyro[6]{};
    std::uint8_t temperature[2]{};
    if (!readRegisters(Target::accel, kAccelDataReg, accel, sizeof(accel)) ||
        !readRegisters(Target::gyro, kGyroDataReg, gyro, sizeof(gyro)) ||
        !readRegisters(Target::accel, kAccelTempReg, temperature, sizeof(temperature))) {
        ++consecutive_failures_;
        return false;
    }
    const auto invalid_frame = [](const std::uint8_t* bytes, std::size_t size) {
        bool all_zero = true;
        bool all_ff = true;
        for (std::size_t i = 0U; i < size; ++i) {
            all_zero = all_zero && bytes[i] == 0x00U;
            all_ff = all_ff && bytes[i] == 0xFFU;
        }
        return all_zero || all_ff;
    };
    if (invalid_frame(accel, sizeof(accel))) {
        setError("BMI088 加速度帧为全 0 或全 0xFF，可能存在接线、片选或 dummy byte 问题");
        ++consecutive_failures_;
        return false;
    }
    // 静止时陀螺仪单帧恰好全 0 并非绝对不可能，只在连续出现时判故障。
    suspicious_gyro_frames_ = invalid_frame(gyro, sizeof(gyro))
                                  ? suspicious_gyro_frames_ + 1U : 0U;
    if (suspicious_gyro_frames_ >= 5U) {
        setError("BMI088 陀螺仪连续 5 帧为全 0 或全 0xFF");
        ++consecutive_failures_;
        return false;
    }
    bool temperature_valid = false;
    sample.timestamp_s = std::chrono::duration<double>(
        std::chrono::steady_clock::now().time_since_epoch()).count();
    sample.ax = accelRawToG(decodeI16Le(&accel[0]));
    sample.ay = accelRawToG(decodeI16Le(&accel[2]));
    sample.az = accelRawToG(decodeI16Le(&accel[4]));
    sample.gx = gyroRawToDps(decodeI16Le(&gyro[0]));
    sample.gy = gyroRawToDps(decodeI16Le(&gyro[2]));
    sample.gz = gyroRawToDps(decodeI16Le(&gyro[4]));
    sample.temperature_c = temperatureRawToC(temperature[0], temperature[1], temperature_valid);
    if (!temperature_valid || !finite(sample)) {
        setError("BMI088 温度或六轴数据无效");
        ++consecutive_failures_;
        return false;
    }
    consecutive_failures_ = 0U;
    return true;
}

void Bmi088::close() noexcept {
    initialized_ = false;
    if (accel_fd_ >= 0) {
        ::close(accel_fd_);
        accel_fd_ = -1;
    }
    if (gyro_fd_ >= 0) {
        ::close(gyro_fd_);
        gyro_fd_ = -1;
    }
}

} // namespace gimbal
