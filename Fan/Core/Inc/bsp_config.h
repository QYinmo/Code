#ifndef __BSP_CONFIG_H
#define __BSP_CONFIG_H

#include "main.h"
#include <stdbool.h>

/* ===========================================================
 *  引脚映射说明 (由 CubeMX main.h 定义, 此处仅做注释)
 *
 *  KEY1_Pin / KEY1_GPIO_Port   → PA4 (故障复位)
 *  KEY2_Pin / KEY2_GPIO_Port   → PA5 (延时+)
 *  KEY3_Pin / KEY3_GPIO_Port   → PA6 (延时-)
 *  LED_GREEN_Pin  / LED_GREEN_GPIO_Port   → PB0
 *  LED_YELLOW_Pin / LED_YELLOW_GPIO_Port  → PB1
 *  LED_RED_Pin    / LED_RED_GPIO_Port     → PB2
 *  BUZZER_Pin     / BUZZER_GPIO_Port      → PB5
 *  PWM 输出       → TIM1_CH1 → PA8 (直接接4线风扇蓝线 PWM)
 *  风扇测速       → TIM3_CH2 → PA7 (4线风扇 TACH 信号)
 *  I2C1_SCL/SDA   → PB6/PB7  (室内 AHT20 + MLX90614 + OLED)
 *  I2C2_SCL/SDA   → PB10/PB11 (室外 AHT20 环境参考)
 * =========================================================== */

/* ===========================================================
 *  ADC 通道 (与 CubeMX adc.c 配合, 运行时动态切换通道)
 * =========================================================== */
#define MQ2_ADC_CHANNEL         ADC_CHANNEL_0   /* PA0 — MQ-2 气体传感器 */
#define ACS712_ADC_CHANNEL      ADC_CHANNEL_1   /* PA1 — ACS712 电流传感器 */

/* ===========================================================
 *  PWM 满量程 (与 tim.c 中 TIM1 ARR=2879 对应)
 * =========================================================== */
#define PWM_MAX_DUTY            2879    /* 100% 对应 ARR 值 */

/* ===========================================================
 *  热水检测阈值 (迟滞比较)
 * =========================================================== */
#define HOTWATER_ON_THRESHOLD   5.0f    /* T_water - T_ambient >= 5℃ → 热水开启 */
#define HOTWATER_OFF_THRESHOLD  3.0f    /* 迟滞关闭阈值 */

/* ===========================================================
 *  MQ-2 气体报警
 * =========================================================== */
#define GAS_ALARM_THRESHOLD     2200    /* ADC 值超过此值视为报警 */
#define GAS_RELEASE_THRESHOLD   1800    /* ADC 值低于此值才解除报警 (迟滞) */
#define GAS_ALARM_CONFIRM_CNT   3       /* 连续 N 次超阈值才触发报警 */
#define GAS_RELEASE_CONFIRM_CNT 5       /* 连续 N 次低于释放阈值才解除 */
#define MQ2_PREHEAT_TIME_MS     20000   /* MQ-2 预热时间 20 秒 */

/* ===========================================================
 *  传感器数据新鲜度 / 降级阈值
 * =========================================================== */
#define SENSOR_STALE_TIMEOUT_MS     2000    /* 超过此时间无新数据视为 stale */
#define INDOOR_AHT20_FAIL_DEGRADE   10      /* 室内 AHT20 连续失败 N 次后降级 */
#define MLX90614_FAIL_DEGRADE       10      /* MLX90614 连续失败 N 次后降级 */
#define OUTDOOR_DEGRADE_DUTY_PCT    40      /* 室外离线保守模式固定占空比 (%) */

/* ===========================================================
 *  ACS712 堵转电流检测
 *  ACS712-5A: 灵敏度 185mV/A, 零电流 2.5V
 *  经分压后 (分压比 0.6): STM32 ADC 零点 ≈ 1861
 *  以下为 **默认值**, 实际值由上电自适应校准覆盖
 * =========================================================== */
#define ACS712_ZERO_CURRENT_ADC_DEFAULT 1861
#define ACS712_STALL_THRESHOLD_DEFAULT  207
#define STALL_DETECT_TIME_MS            800     /* 持续 800ms 才判定堵转 (4线风扇内置保护, 电流响应延迟) */
#define STALL_RPM_MIN                   100     /* PWM>30% 时 RPM 低于此值视为异常 */
#define STALL_RPM_TIMEOUT_MS            2000    /* RPM 持续低于阈值超过此时间才判堵转 (4线风扇直接看 RPM) */
#define STALL_LATCH_CONFIRM_MS          3000    /* stall_detected 须持续 3s 才锁存 (防低PWM/RPM延迟误判) */
#define ACS712_CALIB_SAMPLES            50      /* 上电零点校准采样次数 */
#define ACS712_STALL_MULTIPLIER_NUM     5       /* 堵转阈值 = 峰值 × 5/2 */
#define ACS712_STALL_MULTIPLIER_DEN     2
#define ACS712_STARTUP_CALIB_MS         2000    /* 风扇启动后 2s 内记录峰值电流 */

/* ===========================================================
 *  气体应急试启动参数 (堵转锁存 + 气体报警共存时)
 * =========================================================== */
#define EMERG_RETRY_INTERVAL_MS     10000   /* 每隔 10s 尝试一次启动 */
#define EMERG_RETRY_DUTY_PCT        40      /* 试启动占空比 40% */
#define EMERG_RETRY_DURATION_MS     2000    /* spin-up 后仍无 RPM 则判定失败的检测窗口 */
#define EMERG_SPINUP_GRACE_MS       1500    /* 试启动旋转建立宽限期, 此期间不检查 RPM */
#define EMERG_RPM_MIN               50      /* 试启动成功最低 RPM */

/* ===========================================================
 *  风扇延时停机
 * =========================================================== */
#define FAN_DELAY_DEFAULT       30      /* 默认延时 30 秒 */
#define FAN_DELAY_MIN           10      /* 最短 10 秒 */
#define FAN_DELAY_MAX           120     /* 最长 120 秒 */
#define FAN_DELAY_STEP          10      /* 按键调节步进 10 秒 */
#define FAN_DELAY_FORCE_MULT    3       /* 强制停机 = 延时 × 3 (防无限运行) */

/* ===========================================================
 *  PID 闭环控制默认参数 (湿度 PID)
 *
 *  PV = 室内绝对湿度 (g/m³)
 *  SP = 室外绝对湿度 (g/m³)
 *  CV = 风扇 PWM 占空比 (0 ~ PWM_MAX_DUTY)
 * =========================================================== */
#define PID_KP_DEFAULT          200.0f
#define PID_KI_DEFAULT          5.0f
#define PID_KD_DEFAULT          50.0f
#define PID_OUTPUT_MIN          0.0f
#define PID_OUTPUT_MAX          ((float)PWM_MAX_DUTY)

/* ===========================================================
 *  绝对湿度相关 (Magnus-Tetens 公式)
 * =========================================================== */
#define ABS_HUMI_CONVERGE_THRESHOLD  2.0f   /* 室内外绝对湿度差 < 2 g/m³ 视为收敛 */
#define ABS_HUMI_OUTDOOR_FAIL_MAX    10     /* 室外传感器连续失败 N 次后使用经验值 */
#define ABS_HUMI_OUTDOOR_DEFAULT_T   20.0f  /* 室外传感器失效时的默认温度 */
#define ABS_HUMI_OUTDOOR_DEFAULT_RH  50.0f  /* 室外传感器失效时的默认湿度 */

/* ===========================================================
 *  温度保护 (防止排风过猛人体感觉发冷)
 * =========================================================== */
#define TEMP_LOW_LIMIT          18.0f   /* 室温低于此值限制风扇功率 */
#define TEMP_LIMIT_MAX_DUTY_PCT 50      /* 低温保护时最大占空比 (%) */

/* 热水运行时最低占空比 (防止 PID 输出为 0 但蒸汽未到达传感器) */
#define HOT_WATER_MIN_DUTY_PCT  30

/* 旧线性映射参数 (GAS_ALARM 全速运行时仍保留备用) */
#define TEMP_LOW_BOUND          20.0f
#define TEMP_HIGH_BOUND         40.0f
#define DUTY_LOW_PERCENT        30.0f
#define DUTY_HIGH_PERCENT       90.0f

/* ===========================================================
 *  风扇测速 (4线风扇 TACH 信号, TIM3_CH2 输入捕获)
 * =========================================================== */
#define FAN_TACH_PULSES_PER_REV 2       /* 每转脉冲数 (绝大多数4线风扇同样为2) */
#define FAN_TACH_TIMEOUT_MS     2000    /* 无脉冲超过此时间视为停转 */
#define FAN_TACH_TIM_FREQ_HZ    1000000 /* TIM3 计数频率 1MHz */

/* ===========================================================
 *  温度 / 湿度 双通道调速参数
 *  HOT_WATER 阶段: target_duty = max(duty_temp, duty_humi)
 *  DELAY_STOP 阶段: 湿度主导, 温度通道随时间衰减
 *  不再采用加权求和, 由需求更高的通道主导
 * =========================================================== */

/* 温度通道: 水温-室温 温差线性映射 */
#define TEMP_DELTA_LOW          5.0f    /* 温差映射下界 (℃), 对应最低占空比 */
#define TEMP_DELTA_HIGH         25.0f   /* 温差映射上界 (℃), 对应最高占空比 */
#define TEMP_DUTY_LOW_PCT       30.0f   /* 温差下界对应占空比 (%) */
#define TEMP_DUTY_HIGH_PCT      90.0f   /* 温差上界对应占空比 (%) */

/* 室温修正系数 (每度 ±3%) */
#define TEMP_CORRECTION_CENTER  25.0f   /* 修正中心温度 (℃) */
#define TEMP_CORRECTION_RATE    0.03f   /* 每度修正比例 */
#define TEMP_CORRECTION_MIN     0.7f    /* 修正系数下限 */
#define TEMP_CORRECTION_MAX     1.3f    /* 修正系数上限 */

/* 升降速斜率限制 (每秒允许的最大变化量, 基于 ARR=2879, 运行时按真实 dt 换算) */
#define SLEW_RATE_UP_PER_SEC    400     /* 升速: 每秒最多 +400 ≈ +13.9%/s (快速响应) */
#define SLEW_RATE_DOWN_PER_SEC  150     /* 降速: 每秒最多 -150 ≈ -5.2%/s (平滑过渡) */

/* DELAY_STOP 温度衰减窗口: 绑定基础延时 (fan_delay_sec × 1000ms)
 * 运行时动态计算, 不再使用固定常量
 * decay_ms = g_sys.fan_delay_sec * 1000UL
 * 如基础延时 30s → 温度通道在前 30s 从 1.0 衰减到 0.0, 然后完全由湿度主导
 */

/* 4线风扇特有: 0% PWM 时风扇仍低速转, 最低有效 PWM 约 20% */
#define FAN_4WIRE_MIN_PWM_PCT   20      /* 4线风扇最低有效/稳定运行占空比 (%) */

/* ===========================================================
 *  滤波参数
 * =========================================================== */
#define TEMP_FILTER_SIZE        5       /* 温度滑动平均窗口 */
#define ADC_FILTER_SIZE         8       /* ADC 多次采样取均值 */

/* ===========================================================
 *  Flash 参数存储 (利用 STM32F103C8T6 内部 Flash 最后一页)
 *  64KB Flash, 页大小 1KB, 最后一页 Page63 = 0x0800FC00
 * =========================================================== */
#define FLASH_STORAGE_PAGE_ADDR 0x0800FC00UL
#define FLASH_STORAGE_MAGIC     0xA5C3E1D7UL
#define FLASH_SAVE_DEFER_MS     3000    /* 按键后延迟 3s 再写 Flash */

/* ===========================================================
 *  系统状态机
 * =========================================================== */
typedef enum {
    SYS_STATE_IDLE = 0,     /* 空闲, 风扇停止 */
    SYS_STATE_PREHEAT,      /* MQ-2 预热中 (系统正常响应按键/显示) */
    SYS_STATE_HOT_WATER,    /* 热水运行,温湿双通道联合 PID 调速 */
    SYS_STATE_DELAY_STOP,   /* 延时除湿，湿度主导、温度衰减参与 */
    SYS_STATE_GAS_ALARM,    /* 气体报警, 全速运行 */
    SYS_STATE_STALL_FAULT,      /* 堵转故障, 停机报警 (锁存, 需 KEY1 复位) */
    SYS_STATE_STALL_GAS_EMERG,  /* 堵转锁存 + 气体应急试启动 */
} SystemState_t;

/* ===========================================================
 *  系统全局数据结构
 * =========================================================== */
typedef struct {
    /* ---------- 状态机 ---------- */
    SystemState_t state;

    /* ---------- 室内传感器 (AHT20 #1 via I2C1) ---------- */
    float ambient_temp;         /* 室内温度 ℃ */
    float ambient_humi;         /* 室内相对湿度 RH% */
    float indoor_abs_humi;      /* 室内绝对湿度 g/m³ */
    bool  indoor_aht20_ok;      /* 室内 AHT20 在线 */
    uint32_t indoor_aht20_last; /* 最后成功读取时刻 */

    /* ---------- 室外传感器 (AHT20 #2 via I2C2) ---------- */
    float outdoor_temp;         /* 室外温度 ℃ */
    float outdoor_humi;         /* 室外相对湿度 RH% */
    float outdoor_abs_humi;     /* 室外绝对湿度 g/m³ */
    bool  outdoor_sensor_ok;    /* 室外传感器是否正常 */

    /* ---------- 红外传感器 (MLX90614 via I2C1) ---------- */
    float ir_ambient_temp;      /* MLX90614 Ta */
    float ir_object_temp;       /* MLX90614 Tobj (水温) */
    bool  mlx90614_ok;          /* MLX90614 Ta 在线 */
    bool  mlx90614_obj_ok;      /* MLX90614 Tobj 在线 (独立管理) */
    uint32_t mlx90614_last;     /* Ta 最后成功读取时刻 */
    uint32_t mlx90614_obj_last; /* Tobj 最后成功读取时刻 */

    /* ---------- ADC 数据 ---------- */
    uint16_t gas_adc;           /* MQ-2 ADC 值 */
    uint16_t current_adc;       /* ACS712 ADC 值 */
    float    fan_current_a;     /* 折算后的风扇电流 A */

    /* ---------- 风扇转速 ---------- */
    uint16_t fan_rpm;           /* 实时转速 RPM */

    /* ---------- 标志位 ---------- */
    bool hot_water_detected;
    bool gas_detected;
    bool stall_detected;
    bool fan_running;
    bool mq2_ready;
    bool humidity_converged;    /* 室内外绝对湿度已收敛 */

    /* ---------- PWM ---------- */
    uint16_t pwm_duty;          /* 当前 PWM 比较值 (0 ~ PWM_MAX_DUTY) */

    /* ---------- 延时/计时 ---------- */
    uint32_t delay_start_tick;
    uint32_t delay_stop_enter_tick;  /* 进入 DELAY_STOP 的时刻 (温度衰减计算用) */
    uint32_t stall_start_tick;
    uint32_t fan_delay_sec;

    /* ---------- ACS712 自适应校准 ---------- */
    uint16_t acs712_zero_adc;           /* 动态零点 */
    uint16_t acs712_stall_threshold;    /* 动态堵转阈值 */

    /* ---------- 堵转/RPM 联合检测 ---------- */
    uint32_t rpm_low_start_tick;        /* RPM 持续低于阈值的起始时刻 */

    /* ---------- 堵转锁存 ---------- */
    bool     stall_latched;             /* 堵转已锁存, 仅 KEY1 可清除 */
    uint32_t stall_latch_pending_tick;  /* 锁存确认计时起点 (持续 STALL_LATCH_CONFIRM_MS 才锁存) */

    /* ---------- 气体应急试启动 ---------- */
    uint32_t emerg_retry_tick;          /* 上一次试启动/冷却时刻 */
    uint8_t  emerg_phase;              /* 0=冷却等待, 1=试运行中 */
    bool     emerg_running;            /* 试启动风扇正在运行 */

    /* ---------- Flash 延迟保存 ---------- */
    bool     flash_dirty;               /* 参数已修改待保存 */
    uint32_t flash_dirty_tick;          /* 最后一次修改的时刻 */

    /* ---------- 按键 (SysTick 中断扫描) ---------- */
    volatile uint8_t key_event;         /* KEY_EVENT_xxx, 主循环消费后清零 */

    /* ---------- MQ-2 预热 ---------- */
    uint32_t preheat_start_tick;        /* 预热开始时刻 */

    /* ---------- ACS712 启动峰值自适应 ---------- */
    bool     acs712_startup_done;       /* 启动校准已完成 */
    uint32_t acs712_startup_tick;       /* 风扇首次启动时刻 */
    uint16_t acs712_peak_adc;           /* 启动阶段峰值 ADC */
} SystemData_t;

/* 全局数据实例 */
extern SystemData_t g_sys;

/* 按键事件定义 (位域, SysTick 中设置, 主循环中消费) */
#define KEY_EVENT_NONE      0x00
#define KEY_EVENT_RESET     0x01
#define KEY_EVENT_DELAY_UP  0x02
#define KEY_EVENT_DELAY_DN  0x04

#endif /* __BSP_CONFIG_H */
