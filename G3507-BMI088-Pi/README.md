# 树莓派 5 + BMI088 + MSPM0G3507 二维云台控制

本项目是在树莓派 5 上运行的 C++17 控制程序。它通过 Linux `spidev` 读取 BMI088，完成静止零偏校准、坐标映射、低通滤波和 Mahony 姿态解算，再运行 yaw/pitch 角度外环；树莓派只向 MSPM0G3507 发送目标角速度，不计算或发送电机电流。

## 1. 系统架构与任务分工

```text
BMI088 加速度计(CE0) ─┐
                      ├─ SPI0 ─ 树莓派 5
BMI088 陀螺仪(CE1) ──┘             │
                                   ├─ 500 Hz：校准/轴映射/低通/Mahony
目标角度（配置/命令行/终端）─────────┤
                                   ├─ 500 Hz：yaw/pitch 角度 PID/PD
                                   │
                                   └─ UART 460800 8N1，10 字节命令包
                                                    │
                                                    ▼
                                              MSPM0G3507
                                   速度闭环 / GM6020 CAN / 电流指令
                                   限位 / 过流过温 / 通信失联保护
```

树莓派负责相对姿态和角度外环，输出单位为 degree/s。MSPM0G3507 必须负责电机速度闭环、GM6020 CAN 通信、电流控制、机械限位以及最终的安全保护。两端均应实现通信超时停机，不能把树莓派发送的速度值直接当成电流值。

高频 IMU/控制循环和 UART 发送循环均用 `std::chrono::steady_clock` 的绝对时间点调度。安全状态机为 `BOOT/CALIBRATING → STOP_READY → RUNNING → DEGRADED/FAULT`。UART 采用非阻塞、有界等待和“只保留最新目标”策略；部分帧超过截止时间会锁存通信故障，恢复后先发送 STOP。状态线程默认每秒输出状态、最后故障、恢复健康帧数、频率和最大调度延迟。

## 2. 硬件接线

### BMI088 SPI0

本工程假定 BMI088 的加速度计和陀螺仪共用时钟与数据线，但各用一个硬件片选。电平必须为 3.3 V。

| 树莓派 5 物理引脚 | GPIO/功能 | BMI088 |
|---:|---|---|
| 1 或 17 | 3.3 V | VCC/VDDIO（以模块规格为准） |
| 6 等 | GND | GND |
| 23 | GPIO11 / SPI0 SCLK | SCLK |
| 19 | GPIO10 / SPI0 MOSI | SDI/MOSI |
| 21 | GPIO9 / SPI0 MISO | SDO/MISO |
| 24 | GPIO8 / SPI0 CE0 | 加速度计 CS，`/dev/spidev0.0` |
| 26 | GPIO7 / SPI0 CE1 | 陀螺仪 CS，`/dev/spidev0.1` |

不要向 BMI088 的 IO 引脚施加 5 V。具体模块的电源、稳压和去耦要求应以模块原理图与 Bosch 数据手册为准。

### UART

| 树莓派 5 物理引脚 | 功能 | MSPM0G3507 |
|---:|---|---|
| 8 | GPIO14 / TXD | UART RX |
| 10 | GPIO15 / RXD | UART TX（本项目发送单向命令时可选） |
| 6 等 | GND | GND |

两端必须共地，逻辑电平必须兼容 3.3 V。不要接 RS-232 电平。树莓派 5 上 `/dev/serial0` 常指向独立调试 UART，**不能默认认为它就是排针 8/10 的 GPIO14/GPIO15**。本项目配置示例使用 `/dev/ttyAMA0`，但最终节点必须以实机查询结果为准。通信参数为 460800 baud、8 数据位、无校验、1 停止位、无流控。

## 3. 启用 SPI 和串口

可在树莓派上运行：

```bash
sudo raspi-config
```

在 `Interface Options` 中启用 SPI 和 Serial Port；串口登录 shell 选择禁用，串口硬件选择启用。Bookworm 系统也可检查 `/boot/firmware/config.txt` 中是否有：

```text
dtparam=spi=on
enable_uart=1
dtoverlay=uart0-pi5
```

`dtoverlay=uart0-pi5` 用于将适用 UART 映射到 Pi 5 的 GPIO14/GPIO15；实际镜像、设备树和其他 overlay 可能改变编号。旧系统的配置文件可能位于 `/boot/config.txt`。修改后重启，必须执行：

```bash
ls -l /dev/spidev0.0 /dev/spidev0.1
readlink -f /dev/serial0
ls -l /dev/ttyAMA*
pinctrl get 14
pinctrl get 15
stty -F /dev/ttyAMA0 -a
```

只有 `pinctrl` 显示 GPIO14/15 已配置为目标 UART 的 TX/RX，且设备节点与查询结果一致后，才能把该节点写入 `uart_device`。程序启动时会打印“配置路径 -> 解析后的真实路径”，辅助发现 `/dev/serial0` 指向错误 UART 的问题。

如果串口仍被登录控制台占用，检查 `/boot/firmware/cmdline.txt`（旧系统为 `/boot/cmdline.txt`），删除 `console=serial0,115200`、`console=ttyAMA0,...` 或其他指向该 UART 的 `console=` 项；必须保持该文件原有的单行格式，然后重启。

程序正常运行不应依赖 `sudo`。把用户加入系统已有的 `spi` 和 `dialout` 组，然后重新登录：

```bash
sudo usermod -aG spi,dialout "$USER"
```

不同镜像可能没有 `spi` 组，此时应使用只授权相应设备节点的 udev 规则，而不是长期以 root 身份运行控制程序。

## 4. BMI088 配置与换算

驱动保留了参考文件 `bsp_bmi088.c/.h` 的寄存器、上电顺序和换算：

1. 打开两个 spidev；无传输时硬件片选均释放。
2. 首次读取加速度计芯片 ID，使其进入 SPI 模式。
3. 对加速度计写 `0xB6` 软复位，等待后再次读 ID 恢复 SPI 模式。
4. 对陀螺仪写 `0xB6` 软复位。
5. 校验加速度计 ID `0x1E` 和陀螺仪 ID `0x0F`。
6. 按 `ACC_PWR_CONF`、等待、`ACC_PWR_CTRL` 的顺序启动加速度计。
7. 配置加速度计 ±6 g、陀螺仪 ±500 °/s，并对关键寄存器回读校验。

默认 `ACC_CONF=0xAB`，即 normal filter、800 Hz ODR；默认 `GYRO_BANDWIDTH=0x02`，即 1000 Hz ODR / 116 Hz 带宽。控制程序以 500 Hz 读取，避免参考驱动 100 Hz 配置限制整个系统。若修改寄存器值，必须先核对 BMI088 数据手册。

换算公式为：

```text
acceleration_g = raw_i16 × 6 / 32768
angular_rate_dps = raw_i16 × 500 / 32768
temperature_c = signed_11bit_raw × 0.125 + 23
```

加速度计 SPI 读帧在地址字节之后有一个额外 dummy byte；陀螺仪没有。两者的六轴数据均按小端有符号 16 位解析。

## 5. 构建、测试和运行

需要支持 C++17 的编译器、CMake、Linux spidev 头文件和 pthread：

```bash
cmake -S . -B build
cmake --build build -j
ctest --test-dir build --output-on-failure
```

严格告警构建会把本项目 C/C++ target 的告警视为错误：

```bash
cmake -S . -B build-strict -DGIMBAL_WARNINGS_AS_ERRORS=ON
cmake --build build-strict -j
ctest --test-dir build-strict --output-on-failure
```

正常运行：

```bash
./build/gimbal_controller --config config/gimbal.conf
```

可在启动时覆盖目标角度：

```bash
./build/gimbal_controller --target 30 -10
```

**安全默认行为：初始化和校准完成后仍保持 `STOP_READY`，不会驱动电机。必须由操作者确认机构安全后输入 `run`。** 只有明确传入 `--auto-run`（或显式把配置 `auto_run=true`）时，程序才会在姿态首次有效且所有前置条件满足后自动进入 `RUNNING`；默认配置为 `false`。

运行期间支持以下终端命令：

```text
target <yaw_deg> <pitch_deg>  设置目标角度
run                           启用角度外环
stop                          清空 PID 状态并发送停止模式
reset-fault                   健康帧满足条件后清除故障，仍保持 STOP
status                        显示当前目标和模式
quit                          安全退出
```

测试模式：

```bash
./build/gimbal_controller --imu-only
./build/gimbal_controller --attitude-uart
./build/gimbal_controller --uart-test
./build/gimbal_controller --dry-run
```

- `--imu-only`：初始化和校准 BMI088，低频显示姿态，不打开或写入 UART。
- `--attitude-uart`：初始化和校准 BMI088，复用原 G3507 10 字节帧输出当前
  yaw/pitch 姿态角，不运行 PID，不发送角速度控制帧。
- `--uart-test`：不依赖 BMI088，发送配置中的固定小角速度；启动时有醒目警告，并额外限制在 ±20 °/s。使用前应悬空电机或拆除负载。
- `--dry-run`：正常读取 IMU、计算控制量并生成协议包，但不真正打开或写入 UART。可与 `--uart-test` 组合，纯粹验证组包调度。

姿态串口模式示例：

```bash
./build/gimbal_controller --config config/gimbal.conf --attitude-uart
```

默认以 `uart_frequency_hz=500` Hz、460800 baud、8N1 输出原 10 字节二进制帧：

```text
5A A5 sequence 02 yaw_L yaw_H pitch_L pitch_H crc_L crc_H
```

`mode=02` 表示姿态模式；第 5–6 字节是 yaw，第 7–8 字节是 pitch，均为
`int16` 小端、单位 0.01°。连续 yaw 在编码前折回到 `[-180°, 180°]`。
该模式只允许 `status` 和 `quit` 终端命令；`run`、`stop`、`target` 和
`reset-fault` 被禁用。
它不能与 `--imu-only`、`--uart-test`、`--dry-run`、`--auto-run` 或 `--target`
组合使用。接收端继续使用原 10 字节包头、长度和 CRC 校验，只需按 `mode=2`
解释第 5–8 字节。

启动校准默认持续约 2 秒。若陀螺仪或加速度计波动过大，程序会提示保持静止并重试，最多三次。普通控制模式在校准期间和校准成功后保持停止；姿态串口模式在姿态首次有效前不发帧，正常运行期间只发送 mode 2 姿态帧。姿态无效、降级或超过 `command_timeout_s` 时不继续发送旧姿态。`quit`、SIGINT 或 SIGTERM 退出时，为保证同一线路上潜在电机接收端的安全，会在 mode 2 流结束后发送 5 个 yaw/pitch 均为 0 的 mode 0 STOP 包；接收端必须按 mode 分发，不能把这些 STOP 当姿态角。`run` 前会检查姿态有效、IMU 未超时、目标为有限值、UART 已打开（`dry-run` 除外）。`save_calibration=true` 时，结果会写到 `calibration_output_file`；本次控制始终使用内存中的本次校准值。

## 6. 配置文件

配置使用无第三方依赖的 `key = value` 格式，`#` 后为注释。轴矩阵由 9 个逗号分隔数字组成，必须是正交单位矩阵。

| 配置项 | 默认值 | 说明 |
|---|---:|---|
| `accel_spi_device` | `/dev/spidev0.0` | 加速度计硬件片选设备 |
| `gyro_spi_device` | `/dev/spidev0.1` | 陀螺仪硬件片选设备 |
| `spi_speed_hz` | `1000000` | SPI Mode 0、8 bit 的时钟速度 |
| `accel_conf` | `0xAB` | 加速度计滤波/ODR 寄存器 |
| `gyro_bandwidth` | `0x02` | 陀螺仪 ODR/带宽寄存器 |
| `uart_device` | `/dev/ttyAMA0` | Pi 5 排针示例，必须按实机映射修改 |
| `uart_baud` | `460800` | UART 波特率 |
| `imu_frequency_hz` | `500` | IMU、姿态和角度外环频率 |
| `uart_frequency_hz` | `500` | 发包频率 |
| `accel_cutoff_hz` | `30` | 加速度低通截止频率 |
| `gyro_cutoff_hz` | `45` | 陀螺仪低通截止频率 |
| `mahony_kp/ki` | `1.5/0.02` | Mahony 比例/积分增益 |
| `mahony_accel_*_error_g` | `0.08/0.30` | 加速度全信任和完全拒绝误差门限 |
| `mahony_integral_limit` | `0.10` | Mahony 三轴积分状态限幅 |
| `mahony_max_accel_reject_s` | `0.30` | 连续纯陀螺仪传播后标记降级的时间 |
| `yaw_kp/ki/kd` | 见配置文件 | yaw 角度外环增益 |
| `pitch_kp/ki/kd` | 见配置文件 | pitch 角度外环增益 |
| `*_use_measured_rate` | `true` | D 项使用映射后的机体角速度阻尼 |
| `*_rate_body_axis/sign` | 见配置文件 | yaw/pitch 阻尼所选 body 轴和符号 |
| `*_max_rate_dps` | `120/90` | 目标角速度绝对限幅 |
| `*_integral_limit` | `20` | 积分状态限幅 |
| `*_deadband_deg` | `0.05` | 角度误差死区 |
| `*_target_slew_dps` | `90` | 目标角度斜坡速度 |
| `default_target_*_deg` | `0` | 默认 yaw/pitch 目标角度 |
| `axis_mapping` | 单位矩阵 | 传感器坐标到云台机体坐标 |
| `calibration_seconds` | `2.0` | 静止校准采样时间 |
| `calibration_max_*` | 见配置文件 | 陀螺仪和加速度计静止检测阈值 |
| `save_calibration` | `false` | 是否保存本次校准记录 |
| `calibration_output_file` | `config/calibration.conf` | 校准记录路径 |
| `max_consecutive_spi_failures` | `5` | 连续 SPI 故障阈值 |
| `fault_recovery_success_frames` | `25` | 故障后允许人工恢复前所需健康帧 |
| `command_timeout_s` | `0.1` | UART 旧控制目标失效时间 |
| `uart_frame_deadline_s` | `0.010` | 部分写帧必须完成的严格截止时间 |
| `uart_poll_timeout_ms` | `1` | 每次等待 POLLOUT 的有界时间 |
| `auto_run` | `false` | 校准后自动运行；安全默认关闭 |
| `log_frequency_hz` | `1` | 状态日志频率 |
| `uart_test_*_rate_dps` | `5/0` | UART 测试固定速度 |

配置校验要求 `uart_poll_timeout_ms` 严格小于 `uart_frame_deadline_s`，并至少保留一半帧截止时间用于完成剩余字节；该约束用于拒绝明显错误配置，运行时仍会在 poll 返回后重新读取单调时钟并强制截止。

轴映射示例：若机体 `x=sensor_y`、`y=-sensor_x`、`z=sensor_z`：

```text
axis_mapping = 0,1,0, -1,0,0, 0,0,1
```

不要在 BMI088 驱动中改轴顺序；所有轴交换和符号变换都应放在此矩阵中。

Mahony 根据 `abs(norm(accel)-1 g)` 平滑计算加速度可信度：误差不超过 full-trust 门限时正常校正，过渡区使用 smoothstep 降权，达到 reject 门限后短时只由陀螺仪传播。拒绝期间积分项衰减，三轴积分均有限幅；连续拒绝超过安全时间会进入 `DEGRADED`，恢复足够健康帧后只回到 `STOP_READY`，不会自动恢复运动。

角度外环默认采用 `rate_cmd = Kp × angle_error + Ki × integral - Kd × measured_body_rate`，避免目标阶跃造成误差微分冲击。角速度轴是在 `axis_mapping` 后从 body X/Y/Z 中选择，不能按传感器原始轴猜测。调参顺序应为：先令 Ki=Kd=0 从小到大调整 Kp，再增加 Kd 抑制振荡，最后确有静差时才增加少量 Ki。

## 7. UART 10 字节协议

协议固定为小端序，不能直接发送 C/C++ 结构体内存。树莓派和所附 MSPM0 C 代码均逐字节处理。

| 偏移 | 长度 | 字段 | 说明 |
|---:|---:|---|---|
| 0 | 2 | `header` | 固定 `0xA55A`，线上为 `5A A5` |
| 2 | 1 | `sequence` | 每次准备一个新帧时加 1，自然溢出 |
| 3 | 1 | `mode` | 0 停止，1 角速度有效，2 姿态角，3 故障 |
| 4 | 2 | `yaw` | mode 1：0.01°/s；mode 2：0.01°，小端 |
| 6 | 2 | `pitch` | mode 1：0.01°/s；mode 2：0.01°，小端 |
| 8 | 2 | `crc16` | 前 8 字节的 CRC，小端 |

例如 `25.36 °/s` 编码为 `2536`，`-8.50 °/s` 编码为 `-850`。编码前会同时执行控制器角速度限幅、`int16` 范围限幅和 NaN/Inf 检查。

树莓派端使用中性 `GimbalPacketPayload.yaw_value/pitch_value`，避免把 mode 2
角度命名成角速度；调用者通过 `getRateControlValues()` 或
`getAttitudeValues()` 按 mode 安全读取。MSPM0 端对应使用
`GimbalDecodedPacket.yaw_value/pitch_value`，并通过
`GimbalProtocol_GetRateControl()` 或 `GimbalProtocol_GetAttitude()` 解读。
mode 不匹配时 accessor 返回 `false`。

姿态模式同样乘以 100 后编码到第 5–8 字节。例如 yaw `12.34°`、pitch
`-5.67°` 时，第 5–8 字节为：

```text
D2 04 C9 FD
```

CRC 使用 **CRC-16/MODBUS**：

```text
Polynomial = 0xA001
Initial value = 0xFFFF
RefIn = true
RefOut = true
XorOut = 0x0000
覆盖范围 = 偏移 0..7 的前 8 字节，不包含 CRC 字段
```

MSPM0G3507 可直接加入以下文件：

- `mspm0_protocol/gimbal_protocol.h`
- `mspm0_protocol/gimbal_protocol.c`
- `mspm0_protocol/gimbal_stream_parser.h`
- `mspm0_protocol/gimbal_stream_parser.c`
- `mspm0_protocol/gimbal_safety_monitor.h`
- `mspm0_protocol/gimbal_safety_monitor.c`

其中包含包长、包头、模式枚举、CRC、`uint16/int16` 小端读取、整包合法性检查、流式重同步和通信安全监控。`GimbalStreamParser` 接收逐字节或任意长度分块，滑动搜索 `5A A5`；候选帧失败时只前移一个字节，并统计输入字节、成功帧、候选/CRC 错误、丢弃字节和 sequence 跳变。sequence 在树莓派“准备新帧”时递增，因此跳变只用于诊断，不能单独作为电机故障依据。

`GimbalSafetyMonitor` 默认可配置为 25 ms 目标超时、100 ms 通信故障，且强制
`target_timeout_ms < communication_fault_timeout_ms`。mode 1 更新角速度目标，
mode 0 立即归零，mode 2 只作为姿态遥测而绝不进入电机速度闭环，mode 3 或未知
mode 锁存故障。通信恢复后不会恢复旧目标，必须收到明确的新 mode 1 命令。
具体接入见 `mspm0_protocol/README.md`。

`UartTxStateMachine` 记录 pending 帧创建时间、首次写入时间、偏移和模式。短暂 `EAGAIN` 使用 `poll(POLLOUT)` 最多等待配置的 1 ms；生产实现通过注入的 `SteadyMonotonicClock` 读取 `steady_clock`，poll 返回后、任何再次写入前都会重新检查真实截止时间。帧过期后会丢弃内核待发送输出、锁存 UART 故障，并在恢复时首先发完整零速度 STOP，不补发旧的非零目标。已经到达线路的半帧无法撤回，因此 MSPM0 接收端必须实现以下重同步：滑动搜索双字节包头 `5A A5`，取得固定 10 字节候选帧，验证 mode 和 CRC；失败时只前移一个字节继续搜索。MSPM0 还必须独立实现约 20~30 ms 的目标降速、约 100 ms 的通信故障判定和最终电机保护。

## 8. 安全行为

- BMI088 初始化或芯片 ID 校验失败时，绝不进入有效角速度模式。
- 程序启动、校准期间和校准成功后都保持 STOP；默认必须人工输入 `run`。
- `run` 前检查姿态、IMU 时效、UART 和目标值；`stop`、故障或模式切换会清空 PID 积分、历史和目标斜坡。
- 单次 SPI/姿态异常进入 `DEGRADED` 并停止输出；连续失败达到阈值或 UART 帧超时进入锁存 `FAULT`。恢复足够健康帧后仍须 `run` 或先 `reset-fault`，不会自动恢复运动。
- 连续全 0、全 `0xFF`、无效温度和非有限值均判为异常。
- UART 线程发现控制目标过期时改发停止包；pending 部分帧还有独立截止时间和恢复 STOP 机制。
- Mahony 对动态线加速度平滑降权，短时允许纯陀螺仪传播，长时间不可信则标记姿态降级。
- PID 有输出限幅、积分限幅、条件积分抗饱和、死区和目标斜坡；模式切换会清空积分及历史误差。
- SIGINT、SIGTERM 或 `quit` 退出时，程序先停止线程，再尝试连续发送 5 个零速度停止包，最后关闭串口。
- 姿态串口模式同样执行上述 5 个 mode 0 STOP；安全停止优先于保持纯 mode 2 数据流。
- 非停止模式出现 NaN/Inf 时会被强制转为零速度故障模式。

进程退出码由 `ExitStatus` 明确决定：正常完成、`quit`、SIGINT 和 SIGTERM 返回 `0`；BMI088/UART 初始化失败、真实校准失败以及已锁存的运行故障返回 `2`。校准期间收到用户退出或信号不会被误报成校准算法失败。

这些措施不能代替 MSPM0G3507 端的 20~30 ms 目标降速、约 100 ms 通信失联、机械限位、过流、过温和急停保护。

## 9. yaw 漂移与扩展接口

BMI088 没有磁力计。Mahony 滤波可借助重力长期修正 roll 和 pitch，但重力无法观测绕竖直轴的 yaw，因此本工程的 yaw 是连续展开的相对角度，会随陀螺仪零偏和温漂逐渐漂移。连续展开只消除了 `+180°` 到 `-180°` 的数值跳变，并没有消除物理漂移。

`MahonyFilter::applyYawCorrectionDeg()` 已为 GM6020 编码器或视觉参考预留渐进式 yaw 修正入口；`Application::setTargetAngles()` 可由未来的 OpenCV/YOLO 线程提交目标角度。当前工程不引入 OpenCV、YOLO 或任何视觉依赖。接入绝对参考时必须处理坐标系、时间戳、延迟和异常值，不能直接用不连续测量覆盖四元数。

## 10. 普通 Linux 的实时性

树莓派普通 Linux 不是硬实时系统。绝对时间调度能避免简单相对 sleep 的长期累计漂移，但不能消除调度抢占、缺页、IRQ 和驱动造成的抖动。程序统计实际循环频率和最大延迟；部署时可进一步固定 CPU、调整线程优先级、减少后台负载或使用 PREEMPT_RT，但 MSPM0G3507 必须独立承担快速闭环与最终安全保护。

## 11. 故障排查

### 找不到 `/dev/spidev0.0` 或 `/dev/spidev0.1`

确认 SPI 已启用并重启；检查设备树是否占用了 CE0/CE1；运行 `ls -l /dev/spidev*`。如果只有一个设备节点，检查对应片选是否被其他 overlay 占用。

### 芯片 ID 错误

加速度计应为 `0x1E`，陀螺仪应为 `0x0F`。检查是否把两个 CS 接反、MISO/MOSI 是否反接、供电和共地是否可靠，以及模块是否确实使用 BMI088。先保持默认 1 MHz 排查。

### 数据全部为 0 或 `0xFF`

通常是器件未供电、MISO 悬空、片选错误、SPI 模式错误或接线问题。逻辑分析仪应看到 Mode 0、地址最高位为读标志以及正确的 CE0/CE1 切换。

### 加速度数据错位或数值异常

加速度计每次 SPI 读取在地址字节后额外返回一个 dummy byte，陀螺仪没有。若两者采用相同偏移，加速度六轴数据会整体错位。本驱动已分别处理，不要删除该差异。

### 串口权限不足

检查配置的 `/dev/ttyAMA0`（或实机确认的其他节点）以及当前用户的组。若使用 `/dev/serial0`，先执行 `readlink -f /dev/serial0`，不能假定其对应排针。将用户加入 `dialout` 或配置有限的 udev 规则；不要把 `sudo` 作为常规启动方式。

### 460800 波特率配置失败

确认内核/驱动的 termios 定义支持 `B460800`，用 `pinctrl get 14/15` 确认配置节点确实映射到排针，并关闭登录控制台。不要用会动态改频导致误差过大的软串口。

### MSPM0 报 CRC 错误

确认收到的包恰好为 10 字节，包头线上顺序为 `5A A5`，所有 `int16` 和 CRC 都是低字节先到；CRC 初值为 `0xFFFF`，覆盖前 8 字节，不把末尾 CRC 自身算进去。还要检查串口是否同为 460800 8N1。

## 12. 用逻辑分析仪验证 UART

1. 分析仪地线与树莓派/MSPM0 共地，采样率建议至少为波特率的 10 倍。
2. 选择 UART 解码，460800 baud、8N1、LSB first、非反相。
3. 正常情况下每约 2 ms 出现一帧，每帧固定 10 字节。
4. 确认每帧从 `5A A5` 开始；sequence 通常递增但发送失败时允许丢号；停止模式下 mode 为 0 且两路速度均为 0。
5. 将捕获的前 8 字节按 CRC-16/MODBUS 计算，结果应等于最后两字节的小端值。

已知示例：sequence `0x12`、mode `1`、yaw `25.36 °/s`、pitch `-8.50 °/s` 的完整帧为：

```text
5A A5 12 01 E8 09 AE FC 97 73
```

其中 CRC 数值为 `0x7397`，线上低字节 `97` 先发送。

## 13. 单元测试范围

测试不访问真实硬件或真实时间睡眠，覆盖：CRC 已知向量、10 字节序列化和 mode 安全 accessor、IMU 换算、PID 限幅和测量角速度阻尼、目标阶跃无 D 冲击、Mahony 动态加速度门控/异常输入/积分限幅/四元数归一化、UART 完整写/部分写/EAGAIN/系统调用错误/严格帧截止/恢复 STOP、安全状态和退出码，以及：

- 姿态模式校准前、无效、降级和超时时不发旧帧；
- mode 2 yaw/pitch 编码、yaw 折回、控制命令隔离和退出 5 个 STOP；
- MSPM0 逐字节、任意分块、噪声、粘包、错位和 CRC 错包重同步；
- sequence `0xFF → 0x00` 回绕和跳变统计；
- MSPM0 mode 分发、mode 2 隔离、25 ms 目标超时和 100 ms 通信故障；
- 普通构建与 `GIMBAL_WARNINGS_AS_ERRORS=ON` 严格构建。

```bash
ctest --test-dir build --output-on-failure
```

## 14. 必须遵守的实机验证顺序

1. 先运行 `--imu-only`：确认芯片 ID、静止加速度模长约 1 g、角速度接近 0、温度合理和 body 轴方向正确。
2. 再运行 `--dry-run`：确认校准后状态为 `STOP_READY`，默认不产生有效运动命令；输入 `run` 后才计算 RATE_CONTROL。
3. 用逻辑分析仪验证 UART：460800 8N1、每帧 10 字节、线上包头 `5A A5`、小端字段和 CRC 正确。
4. MSPM0G3507 暂不接电机或让电机悬空：验证双字节包头/固定长度/CRC 重同步、sequence 丢号处理、20~30 ms 目标降速和约 100 ms 通信故障。
5. 最后才以很低角速度限幅连接机构，依次验证 `stop`、SIGINT、拔 UART、拔 SPI、机械限位、过流和过温保护。

仍需真实硬件验证：Pi 5 UART overlay 与实际设备节点、BMI088 轴矩阵、SPI 波形及可用最高速率、500 Hz 调度抖动、静止阈值和温漂、Mahony/PID 参数、UART 长时间误码率、MSPM0 字节流重同步，以及整机所有保护链路。本项目的无硬件测试不能证明真实 SPI/UART 波形或云台运动安全。
