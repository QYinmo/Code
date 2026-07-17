#include "bmi088.hpp"

#include <cmath>
#include <cstdint>
#include <iostream>

bool near(float lhs, float rhs, float tolerance = 1.0e-5F) {
    return std::fabs(lhs - rhs) <= tolerance;
}

int main() {
    const std::uint8_t positive[2]{0x34U, 0x12U};
    const std::uint8_t negative[2]{0x00U, 0x80U};
    if (gimbal::Bmi088::decodeI16Le(positive) != 0x1234 ||
        gimbal::Bmi088::decodeI16Le(negative) != static_cast<std::int16_t>(-32768)) {
        std::cerr << "BMI088 小端 int16 解析失败\n";
        return 1;
    }
    if (!near(gimbal::Bmi088::accelRawToG(16384), 3.0F) ||
        !near(gimbal::Bmi088::accelRawToG(-32768), -6.0F) ||
        !near(gimbal::Bmi088::gyroRawToDps(16384), 250.0F) ||
        !near(gimbal::Bmi088::gyroRawToDps(-32768), -500.0F)) {
        std::cerr << "±6 g 或 ±500 °/s 比例换算失败\n";
        return 1;
    }
    bool valid = false;
    if (!near(gimbal::Bmi088::temperatureRawToC(0x00U, 0x00U, valid), 23.0F) || !valid) {
        std::cerr << "温度换算失败\n";
        return 1;
    }
    return 0;
}
