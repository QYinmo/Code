#include "uart_port.hpp"

#include <cerrno>
#include <cstring>
#include <fcntl.h>
#include <limits.h>
#include <poll.h>
#include <termios.h>
#include <unistd.h>

namespace gimbal {
namespace {

bool baudToTermios(std::uint32_t baud, speed_t& speed) {
    switch (baud) {
    case 9600U: speed = B9600; return true;
    case 115200U: speed = B115200; return true;
#ifdef B230400
    case 230400U: speed = B230400; return true;
#endif
#ifdef B460800
    case 460800U: speed = B460800; return true;
#endif
#ifdef B921600
    case 921600U: speed = B921600; return true;
#endif
    default: return false;
    }
}

} // namespace

UartPort::~UartPort() { close(); }

bool UartPort::openPort(const std::string& device, std::uint32_t baud) {
    close();
    requested_device_ = device;
    resolved_device_ = device;
    speed_t speed{};
    if (!baudToTermios(baud, speed)) {
        last_error_ = "当前系统 termios 不支持波特率 " + std::to_string(baud);
        return false;
    }
    fd_ = ::open(device.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK | O_CLOEXEC);
    if (fd_ < 0) {
        last_error_ = "无法打开串口 " + device + "：" + std::strerror(errno);
        return false;
    }
    char resolved[PATH_MAX]{};
    if (::realpath(device.c_str(), resolved) != nullptr) resolved_device_ = resolved;
    termios settings{};
    if (::tcgetattr(fd_, &settings) != 0) {
        last_error_ = "读取串口配置失败：" + std::string(std::strerror(errno));
        close();
        return false;
    }
    ::cfmakeraw(&settings);
    settings.c_cflag = static_cast<tcflag_t>((settings.c_cflag & ~CSIZE) | CS8 | CLOCAL | CREAD);
    settings.c_cflag &= static_cast<tcflag_t>(~(PARENB | CSTOPB | CRTSCTS));
    settings.c_iflag &= static_cast<tcflag_t>(~(IXON | IXOFF | IXANY));
    settings.c_cc[VMIN] = 0;
    settings.c_cc[VTIME] = 0;
    if (::cfsetispeed(&settings, speed) != 0 || ::cfsetospeed(&settings, speed) != 0 ||
        ::tcsetattr(fd_, TCSANOW, &settings) != 0 || ::tcflush(fd_, TCIOFLUSH) != 0) {
        last_error_ = "设置串口 8N1/波特率失败：" + std::string(std::strerror(errno));
        close();
        return false;
    }
    return true;
}

WriteResult UartPort::writeSome(const std::uint8_t* data, std::size_t length) noexcept {
    if (fd_ < 0 || data == nullptr || length == 0U) return {WriteStatus::error, 0U};
    for (;;) {
        const ssize_t result = ::write(fd_, data, length);
        if (result > 0) return {WriteStatus::progress, static_cast<std::size_t>(result)};
        if (result == 0) return {WriteStatus::would_block, 0U};
        if (errno == EINTR) continue;
        if (errno == EAGAIN || errno == EWOULDBLOCK) return {WriteStatus::would_block, 0U};
        last_error_ = "串口写入失败：" + std::string(std::strerror(errno));
        return {WriteStatus::error, 0U};
    }
}

WaitStatus UartPort::waitWritable(int timeout_ms) noexcept {
    if (fd_ < 0) return WaitStatus::error;
    pollfd descriptor{fd_, POLLOUT, 0};
    for (;;) {
        const int result = ::poll(&descriptor, 1U, timeout_ms);
        if (result > 0) {
            if ((descriptor.revents & POLLOUT) != 0) return WaitStatus::ready;
            last_error_ = "UART poll 返回错误事件";
            return WaitStatus::error;
        }
        if (result == 0) return WaitStatus::timeout;
        if (errno == EINTR) continue;
        last_error_ = "UART poll 失败：" + std::string(std::strerror(errno));
        return WaitStatus::error;
    }
}

void UartPort::discardOutput() noexcept {
    if (fd_ >= 0) (void)::tcflush(fd_, TCOFLUSH);
}

void UartPort::close() noexcept {
    if (fd_ >= 0) {
        ::close(fd_);
        fd_ = -1;
    }
}

} // namespace gimbal
