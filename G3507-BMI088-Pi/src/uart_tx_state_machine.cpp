#include "uart_tx_state_machine.hpp"

#include <algorithm>

namespace gimbal {

UartTxStateMachine::UartTxStateMachine(std::chrono::microseconds frame_deadline,
                                       int poll_timeout_ms, IMonotonicClock& clock)
    : frame_deadline_(std::max(frame_deadline, std::chrono::microseconds(1))),
      poll_timeout_ms_(std::max(poll_timeout_ms, 0)), clock_(clock) {}

void UartTxStateMachine::setLatest(const GimbalPacketPayload& payload,
                                   Clock::time_point updated) noexcept {
    latest_ = payload;
    latest_updated_ = updated;
    latest_valid_ = true;
}

void UartTxStateMachine::prepareFrame(Clock::time_point now) noexcept {
    GimbalPacketPayload payload{};
    if (!recovery_stop_required_ && latest_valid_) payload = latest_;
    if (recovery_stop_required_) {
        payload = makeStopPayload();
    }
    payload.sequence = next_sequence_++;
    pending_ = serializePacket(payload);
    pending_offset_ = 0U;
    pending_mode_ = payload.mode;
    pending_created_at_ = now;
    pending_source_updated_ = latest_updated_;
    first_write_at_ = {};
    pending_valid_ = true;
}

TxEvent UartTxStateMachine::failFrame(TxEvent event, IUartWriter& writer) noexcept {
    if (event == TxEvent::deadline_timeout) ++stats_.deadline_timeouts;
    else ++stats_.io_errors;
    writer.discardOutput();
    pending_valid_ = false;
    pending_offset_ = 0U;
    latest_valid_ = false;
    communication_fault_ = true;
    recovery_stop_required_ = true;
    return event;
}

TxEvent UartTxStateMachine::process(IUartWriter& writer) noexcept {
    auto now = clock_.now();
    if (pending_valid_ && now - pending_created_at_ >= frame_deadline_) {
        return failFrame(TxEvent::deadline_timeout, writer);
    }
    // 尚未写出任何字节时允许最新目标替换旧目标，不形成命令队列。
    if (pending_valid_ && pending_offset_ == 0U && latest_valid_ &&
        latest_updated_ > pending_source_updated_ && !recovery_stop_required_) {
        pending_valid_ = false;
    }
    if (!pending_valid_) prepareFrame(now);

    for (int attempt = 0; attempt < 2; ++attempt) {
        // writeSome 之前总是重新读取真实单调时钟，特别覆盖 poll 等待后的第二次写入。
        now = clock_.now();
        if (now - pending_created_at_ >= frame_deadline_) {
            return failFrame(TxEvent::deadline_timeout, writer);
        }
        const WriteResult result = writer.writeSome(pending_.data() + pending_offset_,
                                                    pending_.size() - pending_offset_);
        if (result.status == WriteStatus::error) {
            return failFrame(TxEvent::io_error, writer);
        }
        if (result.status == WriteStatus::would_block || result.bytes == 0U) {
            ++stats_.would_block_count;
            const WaitStatus wait_status = writer.waitWritable(poll_timeout_ms_);
            if (wait_status == WaitStatus::error) {
                return failFrame(TxEvent::io_error, writer);
            }
            now = clock_.now();
            if (now - pending_created_at_ >= frame_deadline_) {
                return failFrame(TxEvent::deadline_timeout, writer);
            }
            if (wait_status == WaitStatus::timeout) return TxEvent::would_block;
            if (attempt == 1) return TxEvent::would_block;
            continue;
        }
        const std::size_t remaining = pending_.size() - pending_offset_;
        pending_offset_ += std::min(result.bytes, remaining);
        if (first_write_at_ == Clock::time_point{}) first_write_at_ = now;
        if (pending_offset_ == pending_.size()) {
            ++stats_.completed_frames;
            pending_valid_ = false;
            const bool recovered = recovery_stop_required_ && pending_mode_ == PacketMode::stop;
            if (recovered) {
                recovery_stop_required_ = false;
                communication_fault_ = false;
                return TxEvent::stop_recovered;
            }
            return TxEvent::frame_complete;
        }
    }
    return TxEvent::none;
}

void UartTxStateMachine::forceRecoveryStop() noexcept {
    pending_valid_ = false;
    pending_offset_ = 0U;
    latest_valid_ = false;
    communication_fault_ = true;
    recovery_stop_required_ = true;
}

} // namespace gimbal
