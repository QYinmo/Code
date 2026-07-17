#include "gimbal_safety_monitor.h"

#include <string.h>

static void GimbalSafetyMonitor_ZeroTarget(GimbalSafetyMonitor *monitor)
{
    monitor->yaw_target_cdeg_s = 0;
    monitor->pitch_target_cdeg_s = 0;
}

bool GimbalSafetyMonitor_ValidateConfig(
    const GimbalSafetyMonitorConfig *config)
{
    return config != NULL && config->target_timeout_ms > 0U &&
           config->target_timeout_ms <
               config->communication_fault_timeout_ms;
}

bool GimbalSafetyMonitor_Init(
    GimbalSafetyMonitor *monitor,
    const GimbalSafetyMonitorConfig *config)
{
    if (monitor == NULL || !GimbalSafetyMonitor_ValidateConfig(config)) {
        return false;
    }
    (void)memset(monitor, 0, sizeof(*monitor));
    monitor->config = *config;
    monitor->state = GIMBAL_SAFETY_STOPPED;
    return true;
}

GimbalSafetyMonitorEvent GimbalSafetyMonitor_HandlePacket(
    GimbalSafetyMonitor *monitor, const GimbalDecodedPacket *packet,
    uint32_t now_ms)
{
    GimbalRateControlValues rate;
    if (monitor == NULL || packet == NULL) {
        return GIMBAL_SAFETY_EVENT_NONE;
    }
    if (packet->mode == GIMBAL_MODE_ATTITUDE) {
        return GIMBAL_SAFETY_EVENT_ATTITUDE_IGNORED;
    }
    if (packet->mode == GIMBAL_MODE_FAULT ||
        packet->mode > GIMBAL_MODE_FAULT) {
        GimbalSafetyMonitor_ZeroTarget(monitor);
        monitor->fault_latched = true;
        monitor->state = GIMBAL_SAFETY_REMOTE_FAULT;
        return GIMBAL_SAFETY_EVENT_FAULT_LATCHED;
    }
    if (monitor->fault_latched) {
        GimbalSafetyMonitor_ZeroTarget(monitor);
        return GIMBAL_SAFETY_EVENT_FAULT_LATCHED;
    }

    monitor->last_control_frame_ms = now_ms;
    monitor->has_control_frame = true;
    if (packet->mode == GIMBAL_MODE_STOP) {
        GimbalSafetyMonitor_ZeroTarget(monitor);
        monitor->state = GIMBAL_SAFETY_STOPPED;
        return GIMBAL_SAFETY_EVENT_STOPPED;
    }
    if (!GimbalProtocol_GetRateControl(packet, &rate)) {
        GimbalSafetyMonitor_ZeroTarget(monitor);
        monitor->fault_latched = true;
        monitor->state = GIMBAL_SAFETY_REMOTE_FAULT;
        return GIMBAL_SAFETY_EVENT_FAULT_LATCHED;
    }
    monitor->yaw_target_cdeg_s = rate.yaw_rate_cdeg_s;
    monitor->pitch_target_cdeg_s = rate.pitch_rate_cdeg_s;
    monitor->state = GIMBAL_SAFETY_ACTIVE;
    return GIMBAL_SAFETY_EVENT_RATE_UPDATED;
}

void GimbalSafetyMonitor_Tick(
    GimbalSafetyMonitor *monitor, uint32_t now_ms)
{
    uint32_t age_ms;
    if (monitor == NULL || monitor->fault_latched ||
        !monitor->has_control_frame) {
        return;
    }
    age_ms = now_ms - monitor->last_control_frame_ms;
    if (age_ms >= monitor->config.communication_fault_timeout_ms) {
        GimbalSafetyMonitor_ZeroTarget(monitor);
        monitor->state = GIMBAL_SAFETY_COMMUNICATION_FAULT;
        return;
    }
    if (monitor->state == GIMBAL_SAFETY_ACTIVE &&
        age_ms >= monitor->config.target_timeout_ms) {
        /*
         * 本可移植实现选择在目标超时后立即归零。具体电机驱动可在把该
         * 目标交给速度环前增加 20~30 ms 的受控斜坡，但不得延后最终归零。
         */
        GimbalSafetyMonitor_ZeroTarget(monitor);
        monitor->state = GIMBAL_SAFETY_TARGET_TIMEOUT;
    }
}

void GimbalSafetyMonitor_ClearFault(GimbalSafetyMonitor *monitor)
{
    if (monitor == NULL) {
        return;
    }
    GimbalSafetyMonitor_ZeroTarget(monitor);
    monitor->fault_latched = false;
    monitor->has_control_frame = false;
    monitor->state = GIMBAL_SAFETY_STOPPED;
}

bool GimbalSafetyMonitor_GetTarget(
    const GimbalSafetyMonitor *monitor, int16_t *yaw_cdeg_s,
    int16_t *pitch_cdeg_s)
{
    if (monitor == NULL || yaw_cdeg_s == NULL || pitch_cdeg_s == NULL) {
        return false;
    }
    *yaw_cdeg_s = monitor->yaw_target_cdeg_s;
    *pitch_cdeg_s = monitor->pitch_target_cdeg_s;
    return monitor->state == GIMBAL_SAFETY_ACTIVE;
}
