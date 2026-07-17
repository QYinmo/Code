#pragma once

#include "gimbal_protocol.hpp"
#include "uart_port.hpp"

#include <array>
#include <chrono>
#include <cstddef>
#include <cstdint>

namespace gimbal {

class IMonotonicClock {
public:
    using Clock = std::chrono::steady_clock;
    virtual ~IMonotonicClock() = default;
    virtual Clock::time_point now() noexcept = 0;
};

class SteadyMonotonicClock final : public IMonotonicClock {
public:
    Clock::time_point now() noexcept override { return Clock::now(); }
};

enum class TxEvent {
    none,
    frame_complete,
    would_block,
    io_error,
    deadline_timeout,
    stop_recovered
};

struct UartTxStats {
    std::uint64_t completed_frames{0U};
    std::uint64_t would_block_count{0U};
    std::uint64_t io_errors{0U};
    std::uint64_t deadline_timeouts{0U};
};

class UartTxStateMachine {
public:
    using Clock = std::chrono::steady_clock;
    UartTxStateMachine(std::chrono::microseconds frame_deadline, int poll_timeout_ms,
                       IMonotonicClock& clock);
    void setLatest(const GimbalPacketPayload& payload,
                   Clock::time_point updated) noexcept;
    TxEvent process(IUartWriter& writer) noexcept;
    void forceRecoveryStop() noexcept;

    [[nodiscard]] bool communicationFault() const noexcept { return communication_fault_; }
    [[nodiscard]] bool hasPending() const noexcept { return pending_valid_; }
    [[nodiscard]] std::size_t pendingOffset() const noexcept { return pending_offset_; }
    [[nodiscard]] PacketMode pendingMode() const noexcept { return pending_mode_; }
    [[nodiscard]] Clock::time_point pendingCreatedAt() const noexcept { return pending_created_at_; }
    [[nodiscard]] Clock::time_point firstWriteAt() const noexcept { return first_write_at_; }
    [[nodiscard]] const UartTxStats& stats() const noexcept { return stats_; }

private:
    void prepareFrame(Clock::time_point now) noexcept;
    TxEvent failFrame(TxEvent event, IUartWriter& writer) noexcept;

    std::chrono::microseconds frame_deadline_;
    int poll_timeout_ms_;
    IMonotonicClock& clock_;
    GimbalPacketPayload latest_{};
    Clock::time_point latest_updated_{};
    bool latest_valid_{false};
    std::array<std::uint8_t, kPacketLength> pending_{};
    std::size_t pending_offset_{0U};
    PacketMode pending_mode_{PacketMode::stop};
    Clock::time_point pending_created_at_{};
    Clock::time_point pending_source_updated_{};
    Clock::time_point first_write_at_{};
    bool pending_valid_{false};
    bool communication_fault_{false};
    bool recovery_stop_required_{false};
    std::uint8_t next_sequence_{0U};
    UartTxStats stats_{};
};

} // namespace gimbal
