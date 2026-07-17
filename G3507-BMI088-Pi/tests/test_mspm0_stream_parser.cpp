#include "gimbal_protocol.hpp"

extern "C" {
#include "gimbal_stream_parser.h"
}

#include <array>
#include <cstddef>
#include <cstdint>
#include <iostream>
#include <vector>

namespace {

struct Capture {
    std::vector<GimbalDecodedPacket> packets;
};

void captureFrame(const std::uint8_t[GIMBAL_PACKET_LENGTH],
                  const GimbalDecodedPacket* packet, void* context) {
    static_cast<Capture*>(context)->packets.push_back(*packet);
}

std::array<std::uint8_t, gimbal::kPacketLength>
ratePacket(std::uint8_t sequence, float yaw, float pitch) {
    return gimbal::serializePacket(
        gimbal::makeRateControlPayload(sequence, yaw, pitch));
}

bool feedChunks(const std::vector<std::uint8_t>& bytes,
                const std::vector<std::size_t>& chunks,
                std::size_t expected_frames) {
    GimbalStreamParser parser;
    Capture capture;
    std::size_t offset = 0U;
    GimbalStreamParser_Init(&parser);
    for (std::size_t chunk : chunks) {
        if (offset + chunk > bytes.size()) return false;
        (void)GimbalStreamParser_Push(
            &parser, bytes.data() + offset, chunk, captureFrame, &capture);
        offset += chunk;
    }
    if (offset < bytes.size()) {
        (void)GimbalStreamParser_Push(
            &parser, bytes.data() + offset, bytes.size() - offset,
            captureFrame, &capture);
    }
    return capture.packets.size() == expected_frames;
}

} // namespace

int main() {
    const auto first = ratePacket(0x10U, 1.0F, -2.0F);
    const std::vector<std::uint8_t> first_bytes(first.begin(), first.end());
    if (!feedChunks(first_bytes, {10U}, 1U) ||
        !feedChunks(first_bytes, {1U, 9U}, 1U) ||
        !feedChunks(first_bytes, {2U, 8U}, 1U) ||
        !feedChunks(first_bytes, {3U, 1U, 4U, 2U}, 1U)) {
        std::cerr << "完整包或任意分块解析失败\n";
        return 1;
    }
    std::vector<std::size_t> single_bytes(10U, 1U);
    if (!feedChunks(first_bytes, single_bytes, 1U)) {
        std::cerr << "逐字节解析失败\n";
        return 1;
    }

    const auto second = ratePacket(0x11U, 3.0F, 4.0F);
    std::vector<std::uint8_t> noisy{0x00U, 0x5AU, 0x11U, 0xFFU};
    noisy.insert(noisy.end(), first.begin(), first.end());
    noisy.insert(noisy.end(), second.begin(), second.end());
    GimbalStreamParser parser;
    Capture capture;
    GimbalStreamParser_Init(&parser);
    (void)GimbalStreamParser_Push(
        &parser, noisy.data(), noisy.size(), captureFrame, &capture);
    if (capture.packets.size() != 2U ||
        parser.stats.discarded_bytes < 4U ||
        parser.stats.valid_frames != 2U) {
        std::cerr << "噪声跳过或粘包解析失败\n";
        return 1;
    }

    auto bad_crc = first;
    bad_crc[8] ^= 0x01U;
    std::vector<std::uint8_t> recovery(bad_crc.begin(), bad_crc.end());
    recovery.insert(recovery.end(), second.begin(), second.end());
    GimbalStreamParser_Reset(&parser);
    capture.packets.clear();
    (void)GimbalStreamParser_Push(
        &parser, recovery.data(), recovery.size(), captureFrame, &capture);
    if (capture.packets.size() != 1U ||
        capture.packets[0].sequence != 0x11U ||
        parser.stats.candidate_errors == 0U ||
        parser.stats.crc_errors == 0U) {
        std::cerr << "CRC 错包后未滑动恢复正确包\n";
        return 1;
    }

    auto bad_mode = first;
    bad_mode[3] = 9U;
    const std::uint16_t bad_mode_crc =
        gimbal::crc16Modbus(bad_mode.data(), 8U);
    bad_mode[8] = static_cast<std::uint8_t>(bad_mode_crc & 0xFFU);
    bad_mode[9] = static_cast<std::uint8_t>(bad_mode_crc >> 8U);
    std::vector<std::uint8_t> mode_recovery(
        bad_mode.begin(), bad_mode.end());
    mode_recovery.insert(
        mode_recovery.end(), second.begin(), second.end());
    GimbalStreamParser_Reset(&parser);
    capture.packets.clear();
    (void)GimbalStreamParser_Push(
        &parser, mode_recovery.data(), mode_recovery.size(),
        captureFrame, &capture);
    if (capture.packets.size() != 1U ||
        capture.packets[0].sequence != 0x11U ||
        parser.stats.candidate_errors == 0U ||
        parser.stats.crc_errors != 0U) {
        std::cerr << "未知 mode 候选帧后未安全重同步\n";
        return 1;
    }

    std::vector<std::uint8_t> missing(first.begin(), first.end());
    missing.erase(missing.begin() + 5);
    missing.insert(missing.end(), second.begin(), second.end());
    GimbalStreamParser_Reset(&parser);
    capture.packets.clear();
    for (std::uint8_t byte : missing) {
        (void)GimbalStreamParser_PushByte(
            &parser, byte, captureFrame, &capture);
    }
    if (capture.packets.size() != 1U ||
        capture.packets[0].sequence != 0x11U) {
        std::cerr << "缺字节错位后未重新同步\n";
        return 1;
    }

    const auto embedded_header = ratePacket(0x20U, -232.06F, 5.0F);
    if (embedded_header[4] != 0x5AU || embedded_header[5] != 0xA5U ||
        !feedChunks(
            {embedded_header.begin(), embedded_header.end()},
            {4U, 2U, 4U}, 1U)) {
        std::cerr << "payload 内部类似包头时解析失败\n";
        return 1;
    }

    const auto wrap_ff = ratePacket(0xFFU, 0.0F, 0.0F);
    const auto wrap_00 = ratePacket(0x00U, 0.0F, 0.0F);
    const auto jump_02 = ratePacket(0x02U, 0.0F, 0.0F);
    std::vector<std::uint8_t> sequences(wrap_ff.begin(), wrap_ff.end());
    sequences.insert(sequences.end(), wrap_00.begin(), wrap_00.end());
    sequences.insert(sequences.end(), jump_02.begin(), jump_02.end());
    GimbalStreamParser_Reset(&parser);
    capture.packets.clear();
    (void)GimbalStreamParser_Push(
        &parser, sequences.data(), sequences.size(), captureFrame, &capture);
    if (capture.packets.size() != 3U ||
        parser.stats.sequence_jumps != 1U ||
        parser.stats.input_bytes != sequences.size()) {
        std::cerr << "sequence 回绕或跳变统计错误\n";
        return 1;
    }
    return 0;
}
