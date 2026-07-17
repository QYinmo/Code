#include "uart_tx_state_machine.hpp"

#include <algorithm>
#include <cstdint>
#include <iostream>
#include <vector>

class FakeClock final : public gimbal::IMonotonicClock {
public:
    explicit FakeClock(Clock::time_point initial) : now_(initial) {}
    Clock::time_point now() noexcept override { return now_; }
    void advance(Clock::duration amount) noexcept { now_ += amount; }

private:
    Clock::time_point now_;
};

class FakeWriter final : public gimbal::IUartWriter {
public:
    explicit FakeWriter(std::vector<int> actions, FakeClock* clock = nullptr,
                        std::chrono::steady_clock::duration wait_advance = {},
                        gimbal::WaitStatus wait_status = gimbal::WaitStatus::timeout)
        : actions_(std::move(actions)), clock_(clock), wait_advance_(wait_advance),
          wait_status_(wait_status) {}

    gimbal::WriteResult writeSome(const std::uint8_t* data, std::size_t length) noexcept override {
        ++write_calls;
        const int action = action_index_ < actions_.size() ? actions_[action_index_++]
                                                           : static_cast<int>(length);
        if (action < 0) return {gimbal::WriteStatus::error, 0U};
        if (action == 0) return {gimbal::WriteStatus::would_block, 0U};
        const std::size_t count = std::min<std::size_t>(static_cast<std::size_t>(action), length);
        bytes.insert(bytes.end(), data, data + count);
        return {gimbal::WriteStatus::progress, count};
    }

    gimbal::WaitStatus waitWritable(int) noexcept override {
        if (clock_ != nullptr) clock_->advance(wait_advance_);
        return wait_status_;
    }
    void discardOutput() noexcept override { ++discard_count; resync_offset = bytes.size(); }

    std::vector<std::uint8_t> bytes;
    std::size_t discard_count{0U};
    std::size_t resync_offset{0U};
    std::size_t write_calls{0U};

private:
    std::vector<int> actions_;
    std::size_t action_index_{0U};
    FakeClock* clock_;
    std::chrono::steady_clock::duration wait_advance_;
    gimbal::WaitStatus wait_status_;
};

int main() {
    using Clock = gimbal::UartTxStateMachine::Clock;
    const auto start = Clock::time_point{} + std::chrono::seconds(1);

    FakeClock full_clock(start);
    FakeWriter full_writer({10});
    gimbal::UartTxStateMachine full(std::chrono::milliseconds(5), 1, full_clock);
    full.setLatest(gimbal::makeRateControlPayload(0U, 1.0F, -2.0F), start);
    if (full.process(full_writer) != gimbal::TxEvent::frame_complete ||
        full_writer.bytes.size() != gimbal::kPacketLength) {
        std::cerr << "UART 完整写测试失败\n";
        return 1;
    }

    FakeClock partial_clock(start);
    FakeWriter partial_writer({1, 9});
    gimbal::UartTxStateMachine partial(std::chrono::milliseconds(5), 1, partial_clock);
    partial.setLatest(gimbal::makeRateControlPayload(0U, 3.0F, 4.0F), start);
    if (partial.process(partial_writer) != gimbal::TxEvent::frame_complete ||
        partial_writer.bytes.size() != gimbal::kPacketLength ||
        partial.firstWriteAt() != start) {
        std::cerr << "UART 1+9 部分写测试失败\n";
        return 1;
    }

    FakeClock blocked_clock(start);
    FakeWriter blocked_writer({1, 0, 0, 10, 10});
    gimbal::UartTxStateMachine blocked(std::chrono::milliseconds(4), 1, blocked_clock);
    blocked.setLatest(gimbal::makeRateControlPayload(0U, 12.0F, -7.0F), start);
    if (blocked.process(blocked_writer) != gimbal::TxEvent::would_block) {
        std::cerr << "UART 多次 EAGAIN 测试失败\n";
        return 1;
    }
    blocked_clock.advance(std::chrono::milliseconds(2));
    if (blocked.process(blocked_writer) !=
            gimbal::TxEvent::would_block) {
        std::cerr << "UART 连续 EAGAIN 状态保持失败\n";
        return 1;
    }
    blocked_clock.advance(std::chrono::milliseconds(3));
    if (blocked.process(blocked_writer) !=
            gimbal::TxEvent::deadline_timeout ||
        !blocked.communicationFault() || blocked_writer.discard_count != 1U) {
        std::cerr << "UART 帧截止超时测试失败\n";
        return 1;
    }
    blocked.setLatest(gimbal::makeRateControlPayload(0U, 99.0F, 99.0F),
                      start + std::chrono::milliseconds(6));
    blocked_clock.advance(std::chrono::milliseconds(1));
    if (blocked.process(blocked_writer) !=
            gimbal::TxEvent::stop_recovered) {
        std::cerr << "UART 恢复 STOP 帧发送失败\n";
        return 1;
    }
    if (blocked_writer.bytes.size() < blocked_writer.resync_offset + gimbal::kPacketLength) {
        return 1;
    }
    const auto* recovered = blocked_writer.bytes.data() + blocked_writer.resync_offset;
    if (recovered[3] != static_cast<std::uint8_t>(gimbal::PacketMode::stop) ||
        recovered[4] != 0U || recovered[5] != 0U || recovered[6] != 0U || recovered[7] != 0U) {
        std::cerr << "UART 恢复后首先发送的不是零速度 STOP\n";
        return 1;
    }

    // 同一次 process 内：先写 1 字节，EAGAIN 后 poll 将模拟时钟推进越过截止时间。
    FakeClock crossing_clock(start);
    FakeWriter crossing_writer({1, 0, 9, 10}, &crossing_clock,
                               std::chrono::milliseconds(6), gimbal::WaitStatus::ready);
    gimbal::UartTxStateMachine crossing(std::chrono::milliseconds(5), 1, crossing_clock);
    crossing.setLatest(gimbal::makeRateControlPayload(0U, 20.0F, -20.0F), start);
    if (crossing.process(crossing_writer) != gimbal::TxEvent::deadline_timeout ||
        crossing_writer.write_calls != 2U || crossing_writer.bytes.size() != 1U ||
        crossing.stats().deadline_timeouts != 1U || !crossing.communicationFault()) {
        std::cerr << "UART poll 等待期间跨截止时间仍继续写入旧帧\n";
        return 1;
    }
    crossing.setLatest(gimbal::makeRateControlPayload(0U, 88.0F, 88.0F),
                       crossing_clock.now());
    if (crossing.process(crossing_writer) != gimbal::TxEvent::stop_recovered) {
        std::cerr << "跨截止时间故障恢复未优先完成 STOP\n";
        return 1;
    }
    const auto* crossing_recovered = crossing_writer.bytes.data() + crossing_writer.resync_offset;
    if (crossing_writer.bytes.size() != crossing_writer.resync_offset + gimbal::kPacketLength ||
        crossing_recovered[3] != static_cast<std::uint8_t>(gimbal::PacketMode::stop) ||
        crossing_recovered[4] != 0U || crossing_recovered[5] != 0U ||
        crossing_recovered[6] != 0U || crossing_recovered[7] != 0U) {
        std::cerr << "跨截止时间后旧非零帧被补发或恢复帧不是 STOP\n";
        return 1;
    }

    FakeClock error_clock(start);
    FakeWriter error_writer({-1});
    gimbal::UartTxStateMachine error_tx(
        std::chrono::milliseconds(5), 1, error_clock);
    error_tx.setLatest(
        gimbal::makeAttitudePayload(0U, 12.0F, -3.0F), start);
    if (error_tx.process(error_writer) != gimbal::TxEvent::io_error ||
        error_tx.stats().io_errors != 1U ||
        !error_tx.communicationFault() ||
        error_writer.discard_count != 1U) {
        std::cerr << "UART 系统调用错误未锁存通信故障\n";
        return 1;
    }
    return 0;
}
