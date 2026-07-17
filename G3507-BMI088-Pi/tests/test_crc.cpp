#include "gimbal_protocol.hpp"

#include <array>
#include <cstdint>
#include <iostream>

int main() {
    constexpr std::array<std::uint8_t, 9> vector{'1', '2', '3', '4', '5', '6', '7', '8', '9'};
    const auto actual = gimbal::crc16Modbus(vector.data(), vector.size());
    if (actual != 0x4B37U) {
        std::cerr << "CRC-16/MODBUS 已知向量失败，实际值=" << actual << '\n';
        return 1;
    }
    return 0;
}
