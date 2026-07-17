#include "safety_state.hpp"

#include <iostream>

int main() {
    gimbal::SafetyStateMachine state(3U);
    state.beginCalibration();
    if (state.state() != gimbal::SafetyState::calibrating) return 1;
    state.calibrationSucceeded();
    if (state.state() != gimbal::SafetyState::stop_ready || state.isRunning()) {
        std::cerr << "校准完成后没有保持 STOP_READY\n";
        return 1;
    }
    if (state.requestRun({false, true, true, true}) ||
        !state.requestRun({true, true, true, true}) || !state.isRunning()) {
        std::cerr << "run 前置条件检查失败\n";
        return 1;
    }
    state.requestStop();
    if (state.isRunning()) return 1;
    state.requestRun({true, true, true, true});
    state.sampleFailure(gimbal::FaultReason::spi_read, false);
    if (state.state() != gimbal::SafetyState::degraded) return 1;
    state.sampleSuccess();
    state.sampleSuccess();
    state.sampleSuccess();
    if (state.state() != gimbal::SafetyState::stop_ready || state.isRunning()) {
        std::cerr << "偶发故障恢复后不应自动恢复 RUNNING\n";
        return 1;
    }
    state.sampleFailure(gimbal::FaultReason::spi_read, true);
    if (state.state() != gimbal::SafetyState::fault) return 1;
    state.sampleSuccess(); state.sampleSuccess(); state.sampleSuccess();
    if (!state.requestRun({true, true, true, true})) {
        std::cerr << "锁存故障满足健康帧且用户重新 run 后未恢复\n";
        return 1;
    }
    state.beginCalibration();
    state.calibrationFailed();
    if (state.state() != gimbal::SafetyState::fault ||
        state.lastFault() != gimbal::FaultReason::calibration_failed) return 1;
    return 0;
}
