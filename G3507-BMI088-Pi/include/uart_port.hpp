#pragma once

#include <cstddef>
#include <cstdint>
#include <string>

namespace gimbal {

enum class WriteStatus { progress, would_block, error };

struct WriteResult {
    WriteStatus status{WriteStatus::error};
    std::size_t bytes{0U};
};

enum class WaitStatus { ready, timeout, error };

class IUartWriter {
public:
    virtual ~IUartWriter() = default;
    virtual WriteResult writeSome(const std::uint8_t* data, std::size_t length) noexcept = 0;
    virtual WaitStatus waitWritable(int timeout_ms) noexcept = 0;
    virtual void discardOutput() noexcept = 0;
};

class UartPort final : public IUartWriter {
public:
    UartPort() = default;
    ~UartPort();
    UartPort(const UartPort&) = delete;
    UartPort& operator=(const UartPort&) = delete;

    bool openPort(const std::string& device, std::uint32_t baud);
    // 非阻塞写：成功写入的字节数；EAGAIN 返回 0；其他错误返回 -1。
    WriteResult writeSome(const std::uint8_t* data, std::size_t length) noexcept override;
    WaitStatus waitWritable(int timeout_ms) noexcept override;
    void discardOutput() noexcept override;
    void close() noexcept;
    [[nodiscard]] bool isOpen() const noexcept { return fd_ >= 0; }
    [[nodiscard]] const std::string& lastError() const noexcept { return last_error_; }
    [[nodiscard]] const std::string& requestedDevice() const noexcept { return requested_device_; }
    [[nodiscard]] const std::string& resolvedDevice() const noexcept { return resolved_device_; }

private:
    int fd_{-1};
    std::string last_error_;
    std::string requested_device_;
    std::string resolved_device_;
};

} // namespace gimbal
