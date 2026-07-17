# MSPM0G3507 协议模块集成说明

本目录中的代码是可移植纯 C，不依赖 Linux、树莓派或具体 MSPM0 SDK：

- `gimbal_protocol.h/.c`：10 字节协议、CRC、整包校验、解析和按 mode 的安全 accessor；
- `gimbal_stream_parser.h/.c`：连续 UART 字节流的滑动重同步解析；
- `gimbal_safety_monitor.h/.c`：目标超时、通信故障、STOP/FAULT 和 mode 分发。

## 推荐接收流程

UART DMA、FIFO 或中断每次得到任意长度数据后调用：

```c
GimbalStreamParser_Push(&parser, bytes, length, OnGimbalFrame, context);
```

回调中把已经通过包头、mode 和 CRC 校验的包交给安全监控：

```c
static void OnGimbalFrame(
    const uint8_t frame[GIMBAL_PACKET_LENGTH],
    const GimbalDecodedPacket *packet,
    void *context)
{
    AppContext *app = (AppContext *)context;
    (void)frame;
    (void)GimbalSafetyMonitor_HandlePacket(
        &app->monitor, packet, app->monotonic_ms);
}
```

主循环或固定周期中调用：

```c
GimbalSafetyMonitor_Tick(&monitor, monotonic_ms);
```

只有下面调用返回 `true` 时，目标才允许进入速度闭环：

```c
int16_t yaw_cdeg_s;
int16_t pitch_cdeg_s;
bool active = GimbalSafetyMonitor_GetTarget(
    &monitor, &yaw_cdeg_s, &pitch_cdeg_s);
```

## mode 分发

- `GIMBAL_MODE_STOP`：立即把目标归零；
- `GIMBAL_MODE_RATE_CONTROL`：通过
  `GimbalProtocol_GetRateControl()` 取得 0.01°/s 目标；
- `GIMBAL_MODE_ATTITUDE`：只允许通过
  `GimbalProtocol_GetAttitude()` 取得 0.01° 姿态，安全监控会忽略它；
- `GIMBAL_MODE_FAULT` 或未知 mode：锁存故障并归零。

不要直接把 `GimbalDecodedPacket.yaw_value/pitch_value` 当作角速度。必须先检查
`mode`，最好只使用对应的安全 accessor。

## 超时配置

推荐初值：

```c
const GimbalSafetyMonitorConfig config = {
    .target_timeout_ms = 25U,
    .communication_fault_timeout_ms = 100U
};
```

必须满足：

```text
0 < target_timeout_ms < communication_fault_timeout_ms
```

本实现选择在目标超时后立即把目标归零。若电机需要更平滑的行为，可在速度闭环
入口增加受控降速斜坡，但不能取消约 100 ms 的最终通信故障停止。

`FAULT` 和未知 mode 会锁存，必须由上层确认机械、电气和通信状态安全后显式调用：

```c
GimbalSafetyMonitor_ClearFault(&monitor);
```

清除后仍保持零目标；只有后续明确的新 `RATE_CONTROL` 才能恢复非零目标。

## 必须由具体工程实现的保护

本目录不包含具体电机驱动、CAN、定时器和 ADC 代码。MSPM0 工程仍必须独立实现：

- 机械限位；
- 过流和过温；
- 急停；
- 电机速度/电流闭环；
- 看门狗；
- UART/DMA 错误恢复；
- 最终的功率级关断。

主机单元测试不能替代真实 MSPM0 和电机硬件验证。
