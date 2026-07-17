#ifndef GIMBAL_SAFETY_MONITOR_H
#define GIMBAL_SAFETY_MONITOR_H

#include "gimbal_protocol.h"

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    uint32_t target_timeout_ms;
    uint32_t communication_fault_timeout_ms;
} GimbalSafetyMonitorConfig;

typedef enum {
    GIMBAL_SAFETY_STOPPED = 0,
    GIMBAL_SAFETY_ACTIVE,
    GIMBAL_SAFETY_TARGET_TIMEOUT,
    GIMBAL_SAFETY_COMMUNICATION_FAULT,
    GIMBAL_SAFETY_REMOTE_FAULT
} GimbalSafetyMonitorState;

typedef enum {
    GIMBAL_SAFETY_EVENT_NONE = 0,
    GIMBAL_SAFETY_EVENT_RATE_UPDATED,
    GIMBAL_SAFETY_EVENT_STOPPED,
    GIMBAL_SAFETY_EVENT_ATTITUDE_IGNORED,
    GIMBAL_SAFETY_EVENT_FAULT_LATCHED
} GimbalSafetyMonitorEvent;

typedef struct {
    GimbalSafetyMonitorConfig config;
    GimbalSafetyMonitorState state;
    int16_t yaw_target_cdeg_s;
    int16_t pitch_target_cdeg_s;
    uint32_t last_control_frame_ms;
    bool has_control_frame;
    bool fault_latched;
} GimbalSafetyMonitor;

bool GimbalSafetyMonitor_ValidateConfig(
    const GimbalSafetyMonitorConfig *config);
bool GimbalSafetyMonitor_Init(
    GimbalSafetyMonitor *monitor,
    const GimbalSafetyMonitorConfig *config);
GimbalSafetyMonitorEvent GimbalSafetyMonitor_HandlePacket(
    GimbalSafetyMonitor *monitor, const GimbalDecodedPacket *packet,
    uint32_t now_ms);
void GimbalSafetyMonitor_Tick(
    GimbalSafetyMonitor *monitor, uint32_t now_ms);
void GimbalSafetyMonitor_ClearFault(GimbalSafetyMonitor *monitor);
bool GimbalSafetyMonitor_GetTarget(
    const GimbalSafetyMonitor *monitor, int16_t *yaw_cdeg_s,
    int16_t *pitch_cdeg_s);

#ifdef __cplusplus
}
#endif

#endif /* GIMBAL_SAFETY_MONITOR_H */
