/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "adc.h"
#include "i2c.h"
#include "iwdg.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "bsp_config.h"
#include "bsp_aht20.h"
#include "bsp_mlx90614.h"
#include "bsp_oled.h"
#include "bsp_pid.h"
#include "bsp_humidity.h"
#include "bsp_flash_storage.h"
#include "bsp_fan_tacho.h"
#include <stdio.h>
#include <string.h>
#include <math.h>
#include <stdarg.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
SystemData_t g_sys = {
    .state = SYS_STATE_PREHEAT,
    .fan_delay_sec = FAN_DELAY_DEFAULT,
    .acs712_zero_adc = ACS712_ZERO_CURRENT_ADC_DEFAULT,
    .acs712_stall_threshold = ACS712_STALL_THRESHOLD_DEFAULT,
};

static PID_Controller_t g_pid;
static uint8_t outdoor_fail_count = 0;

/* 非阻塞 AHT20 句柄 */
static AHT20_Handle_t h_aht20_indoor;
static AHT20_Handle_t h_aht20_outdoor;

/* 室内传感器连续失败计数 */
static uint8_t indoor_aht20_fail_cnt = 0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
static void App_Init(void);
static void App_ReadSensors(void);
static void App_UpdateStateMachine(void);
static void App_ExecuteOutput(void);
static void App_UpdateDisplay(void);
static void App_HandleKeys(void);
static void App_FlashDeferredSave(void);
static void App_ACS712_PeakTrack(void);

static uint16_t ADC_ReadChannel(uint32_t channel);
static uint16_t ADC_ReadFiltered(uint32_t channel, uint8_t count);
static float Filter_SlidingAverage(float new_value, float *buf, uint8_t size, uint8_t *index, bool *filled);
static bool Detect_HotWater(float t_water, float t_ambient);
static bool Detect_GasAlarm(uint16_t gas_adc);
static bool Detect_FanStall(uint16_t current_adc, uint16_t pwm_duty);
static float Convert_ACS712_ToCurrent(uint16_t adc_value);
static void ACS712_CalibrateZero(void);
static void I2C2_BusRecovery(void);
static void Fan_SetDuty(uint16_t duty);
static void Fan_Stop(void);
static void Buzzer_On(void);
static void Buzzer_Off(void);
static void LED_Set(bool green, bool yellow, bool red);
static void Debug_Print(const char *fmt, ...);

/* 温度 / 湿度双通道调速 */
static float Calc_TempDuty(float delta_t, float ambient_temp);
static uint16_t SlewRateLimit(float target, uint16_t current);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_ADC1_Init();
  MX_I2C1_Init();
  MX_TIM1_Init();
  MX_USART1_UART_Init();
  MX_I2C2_Init();
  MX_TIM3_Init();
  MX_IWDG_Init();
  /* USER CODE BEGIN 2 */
  App_Init();
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    App_HandleKeys();
    App_ReadSensors();
    App_UpdateStateMachine();
    App_ExecuteOutput();
    App_UpdateDisplay();
    App_FlashDeferredSave();
    App_ACS712_PeakTrack();
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_LSI|RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.LSIState = RCC_LSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADC;
  PeriphClkInit.AdcClockSelection = RCC_ADCPCLK2_DIV6;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

/* ======================== 应用层初始化 ======================== */
static void App_Init(void)
{
    FlashStorage_t flash_data;

    /* 启动 PWM 输出 */
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
    Fan_Stop();
    LED_Set(false, false, false);
    Buzzer_Off();

    /* ADC 自校准 */
    HAL_ADCEx_Calibration_Start(&hadc1);

    /* 从 Flash 加载参数 (带范围校验) */
    if (Flash_Load(&flash_data)) {
        if (flash_data.fan_delay_sec >= FAN_DELAY_MIN &&
            flash_data.fan_delay_sec <= FAN_DELAY_MAX) {
            g_sys.fan_delay_sec = flash_data.fan_delay_sec;
        }
        if (flash_data.pid_kp >= 0.0f && flash_data.pid_kp <= 10000.0f &&
            flash_data.pid_ki >= 0.0f && flash_data.pid_ki <= 1000.0f &&
            flash_data.pid_kd >= 0.0f && flash_data.pid_kd <= 10000.0f) {
            PID_Init(&g_pid, flash_data.pid_kp, flash_data.pid_ki, flash_data.pid_kd,
                     PID_OUTPUT_MIN, PID_OUTPUT_MAX);
        } else {
            PID_Init(&g_pid, PID_KP_DEFAULT, PID_KI_DEFAULT, PID_KD_DEFAULT,
                     PID_OUTPUT_MIN, PID_OUTPUT_MAX);
        }
        if (flash_data.acs_zero_adc >= 1500 && flash_data.acs_zero_adc <= 2200) {
            g_sys.acs712_zero_adc = flash_data.acs_zero_adc;
        }
        Debug_Print("Flash: 参数加载成功\r\n");
    } else {
        PID_Init(&g_pid, PID_KP_DEFAULT, PID_KI_DEFAULT, PID_KD_DEFAULT,
                 PID_OUTPUT_MIN, PID_OUTPUT_MAX);
        Debug_Print("Flash: 首次启动, 使用默认参数\r\n");
    }

    /* 初始化室内 AHT20 (I2C1) — 阻塞初始化命令 + 非阻塞句柄 */
    if (AHT20_Init(&hi2c1) == HAL_OK) {
        g_sys.indoor_aht20_ok = true;
    } else {
        g_sys.indoor_aht20_ok = false;
        Debug_Print("ERR:AHT20 室内 Init Fail\r\n");
    }
    AHT20_Handle_Init(&h_aht20_indoor, &hi2c1);
    g_sys.indoor_aht20_last = HAL_GetTick();

    /* 初始化室外 AHT20 (I2C2) */
    if (AHT20_Init(&hi2c2) == HAL_OK) {
        g_sys.outdoor_sensor_ok = true;
    } else {
        g_sys.outdoor_sensor_ok = false;
        Debug_Print("WARN:AHT20 室外 Init Fail\r\n");
    }
    AHT20_Handle_Init(&h_aht20_outdoor, &hi2c2);

    /* 初始化风扇测速 */
    FanTacho_Init();

    /* 初始化 OLED 显示屏 */
    OLED_Init(&hi2c1);
    OLED_Clear();
    OLED_ShowString(0, 0, "Smart Exhaust V3");
    OLED_ShowString(0, 2, "MQ2 Preheating");
    OLED_ShowString(0, 4, "ACS712 Calib...");
    OLED_Refresh();

    /* ACS712 零点自适应校准 (风扇静止时采样) */
    ACS712_CalibrateZero();

    /* 非阻塞预热: 记录时间, 进入 PREHEAT 状态 */
    g_sys.preheat_start_tick = HAL_GetTick();
    g_sys.state = SYS_STATE_PREHEAT;
    g_sys.mq2_ready = false;

    Debug_Print("系统启动... MQ-2 非阻塞预热中\r\n");
}

/* ======================== 传感器采样 ======================== */
static void App_ReadSensors(void)
{
    static float ir_obj_buf[TEMP_FILTER_SIZE] = {0};
    static float env_temp_buf[TEMP_FILTER_SIZE] = {0};
    static uint8_t ir_idx = 0, env_idx = 0;
    static bool ir_filled = false, env_filled = false;
    float mlx_ta = 0.0f, mlx_tobj = 0.0f;
    uint32_t now = HAL_GetTick();

    /* ---- 室内 AHT20 (非阻塞) ---- */
    AHT20_Poll(&h_aht20_indoor);
    if (h_aht20_indoor.data_fresh) {
        g_sys.ambient_temp = Filter_SlidingAverage(
            h_aht20_indoor.temperature, env_temp_buf, TEMP_FILTER_SIZE, &env_idx, &env_filled);
        g_sys.ambient_humi = h_aht20_indoor.humidity;
        g_sys.indoor_abs_humi = Humidity_CalcAbsolute(g_sys.ambient_temp, g_sys.ambient_humi);
        g_sys.indoor_aht20_ok = true;
        g_sys.indoor_aht20_last = now;
        indoor_aht20_fail_cnt = 0;
    } else if (h_aht20_indoor.state == AHT20_STATE_ERROR) {
        if (indoor_aht20_fail_cnt < 255) indoor_aht20_fail_cnt++;
        if (indoor_aht20_fail_cnt >= INDOOR_AHT20_FAIL_DEGRADE) {
            g_sys.indoor_aht20_ok = false;
        }
    }
    if (g_sys.indoor_aht20_ok && (now - g_sys.indoor_aht20_last > SENSOR_STALE_TIMEOUT_MS)) {
        g_sys.indoor_aht20_ok = false;
    }

    /* ---- 室外 AHT20 (非阻塞) ---- */
    AHT20_Poll(&h_aht20_outdoor);
    if (h_aht20_outdoor.data_fresh) {
        g_sys.outdoor_temp = h_aht20_outdoor.temperature;
        g_sys.outdoor_humi = h_aht20_outdoor.humidity;
        g_sys.outdoor_abs_humi = Humidity_CalcAbsolute(g_sys.outdoor_temp, g_sys.outdoor_humi);
        g_sys.outdoor_sensor_ok = true;
        outdoor_fail_count = 0;
    } else if (h_aht20_outdoor.state == AHT20_STATE_ERROR) {
        if (++outdoor_fail_count >= ABS_HUMI_OUTDOOR_FAIL_MAX) {
            g_sys.outdoor_sensor_ok = false;
            g_sys.outdoor_temp = ABS_HUMI_OUTDOOR_DEFAULT_T;
            g_sys.outdoor_humi = ABS_HUMI_OUTDOOR_DEFAULT_RH;
            g_sys.outdoor_abs_humi = Humidity_CalcAbsolute(ABS_HUMI_OUTDOOR_DEFAULT_T, ABS_HUMI_OUTDOOR_DEFAULT_RH);
            if (outdoor_fail_count == ABS_HUMI_OUTDOOR_FAIL_MAX) {
                I2C2_BusRecovery();
                AHT20_Init(&hi2c2);
                AHT20_Handle_Init(&h_aht20_outdoor, &hi2c2);
                outdoor_fail_count = 0;
            }
        }
    }

    /* ---- 红外测温 MLX90614 (I2C1, 交替读 Ta/Tobj 减半阻塞) ---- */
    {
        static uint32_t mlx_last_read = 0;
        static bool mlx_read_ta_next = true;
        static uint8_t mlx_ta_fail_cnt = 0;
        static uint8_t mlx_obj_fail_cnt = 0;
        if (now - mlx_last_read >= 150) {
            mlx_last_read = now;
            HAL_StatusTypeDef ret;
            if (mlx_read_ta_next) {
                ret = MLX90614_ReadAmbient(&hi2c1, &mlx_ta);
                if (ret == HAL_OK) {
                    g_sys.ir_ambient_temp = mlx_ta;
                    g_sys.mlx90614_ok = true;
                    g_sys.mlx90614_last = now;
                    mlx_ta_fail_cnt = 0;
                } else {
                    if (mlx_ta_fail_cnt < 255) mlx_ta_fail_cnt++;
                    if (mlx_ta_fail_cnt >= MLX90614_FAIL_DEGRADE) {
                        g_sys.mlx90614_ok = false;
                    }
                }
            } else {
                ret = MLX90614_ReadObject(&hi2c1, &mlx_tobj);
                if (ret == HAL_OK) {
                    g_sys.ir_object_temp = Filter_SlidingAverage(mlx_tobj, ir_obj_buf, TEMP_FILTER_SIZE, &ir_idx, &ir_filled);
                    g_sys.mlx90614_obj_ok = true;
                    g_sys.mlx90614_obj_last = now;
                    mlx_obj_fail_cnt = 0;
                } else {
                    if (mlx_obj_fail_cnt < 255) mlx_obj_fail_cnt++;
                    if (mlx_obj_fail_cnt >= MLX90614_FAIL_DEGRADE) {
                        g_sys.mlx90614_obj_ok = false;
                    }
                }
            }
            mlx_read_ta_next = !mlx_read_ta_next;
        }
    }
    if (g_sys.mlx90614_ok && (now - g_sys.mlx90614_last > SENSOR_STALE_TIMEOUT_MS)) {
        g_sys.mlx90614_ok = false;
    }
    if (g_sys.mlx90614_obj_ok && (now - g_sys.mlx90614_obj_last > SENSOR_STALE_TIMEOUT_MS)) {
        g_sys.mlx90614_obj_ok = false;
    }

    /* ---- ADC 采样: 气体 + 电流 ---- */
    g_sys.gas_adc = ADC_ReadFiltered(MQ2_ADC_CHANNEL, ADC_FILTER_SIZE);
    g_sys.current_adc = ADC_ReadFiltered(ACS712_ADC_CHANNEL, ADC_FILTER_SIZE);
    g_sys.fan_current_a = Convert_ACS712_ToCurrent(g_sys.current_adc);

    /* ---- 风扇转速 ---- */
    g_sys.fan_rpm = FanTacho_GetRPM();

    /* ---- 状态判断 ---- */
    g_sys.gas_detected = g_sys.mq2_ready ? Detect_GasAlarm(g_sys.gas_adc) : false;
    /* 热水判定需要 MLX90614 Tobj 和室内 AHT20 同时有效 */
    g_sys.hot_water_detected = (g_sys.mlx90614_obj_ok && g_sys.indoor_aht20_ok) ?
        Detect_HotWater(g_sys.ir_object_temp, g_sys.ambient_temp) : false;
    g_sys.stall_detected = Detect_FanStall(g_sys.current_adc, g_sys.pwm_duty);
    /* 传感器降级时禁止收敛判定, 仅走基础延时 + 强制超时 */
    g_sys.humidity_converged = (g_sys.indoor_aht20_ok && g_sys.outdoor_sensor_ok) ?
        Humidity_IsConverged(g_sys.indoor_abs_humi, g_sys.outdoor_abs_humi) : false;
}

/* ======================== 状态机 ======================== */
static void App_UpdateStateMachine(void)
{
    SystemState_t prev = g_sys.state;

    /* PREHEAT 特殊处理: 预热完成后转 IDLE */
    if (g_sys.state == SYS_STATE_PREHEAT) {
        if ((HAL_GetTick() - g_sys.preheat_start_tick) >= MQ2_PREHEAT_TIME_MS) {
            g_sys.mq2_ready = true;
            g_sys.state = SYS_STATE_IDLE;
            Debug_Print("MQ-2 就绪\r\n");
        }
        if (prev != g_sys.state) {
            Debug_Print("State: %d -> %d\r\n", prev, g_sys.state);
        }
        return;
    }

    /* --- 堵转锁存: stall_detected 须持续 STALL_LATCH_CONFIRM_MS 才真正锁存 --- */
    if (g_sys.stall_detected && !g_sys.stall_latched) {
        if (g_sys.stall_latch_pending_tick == 0) {
            g_sys.stall_latch_pending_tick = HAL_GetTick();
        } else if ((HAL_GetTick() - g_sys.stall_latch_pending_tick) >= STALL_LATCH_CONFIRM_MS) {
            g_sys.stall_latched = true;
            g_sys.stall_latch_pending_tick = 0;
            g_sys.emerg_phase = 0;
            g_sys.emerg_running = false;
            g_sys.emerg_retry_tick = HAL_GetTick();
            Debug_Print("堵转锁存!\r\n");
        }
    } else if (!g_sys.stall_detected && !g_sys.stall_latched) {
        /* stall_detected 消失且尚未锁存 → 重置确认计时 */
        g_sys.stall_latch_pending_tick = 0;
    }

    /* --- 状态转换逻辑 --- */
    if (g_sys.stall_latched) {
        /* 堵转已锁存期间 */
        if (g_sys.gas_detected) {
            g_sys.state = SYS_STATE_STALL_GAS_EMERG;
        } else {
            /* 气体解除 or 没有气体 → 普通堵转锁存 */
            if (g_sys.state == SYS_STATE_STALL_GAS_EMERG) {
                /* 退出应急: 停风扇, 重置应急状态 */
                g_sys.emerg_phase = 0;
                g_sys.emerg_running = false;
            }
            g_sys.state = SYS_STATE_STALL_FAULT;
        }
    } else {
        /* 无堵转锁存 — 正常优先级逻辑 */
        if (g_sys.gas_detected) {
            g_sys.state = SYS_STATE_GAS_ALARM;
        } else {
            switch (g_sys.state) {
            case SYS_STATE_GAS_ALARM:
                g_sys.state = g_sys.hot_water_detected ? SYS_STATE_HOT_WATER : SYS_STATE_IDLE;
                break;

            case SYS_STATE_IDLE:
                if (g_sys.hot_water_detected) {
                    g_sys.state = SYS_STATE_HOT_WATER;
                    PID_Reset(&g_pid);
                    PID_SetSetpoint(&g_pid, g_sys.outdoor_abs_humi);
                }
                break;

            case SYS_STATE_HOT_WATER:
                /* setpoint 刷新已移至 App_ExecuteOutput() 中 PID_Compute 调用前 */
                if (!g_sys.hot_water_detected) {
                    g_sys.state = SYS_STATE_DELAY_STOP;
                    g_sys.delay_start_tick = HAL_GetTick();
                    g_sys.delay_stop_enter_tick = HAL_GetTick(); /* 记录进入时刻, 用于温度衰减 */
                }
                break;

            case SYS_STATE_DELAY_STOP: {
                uint32_t elapsed = HAL_GetTick() - g_sys.delay_start_tick;
                uint32_t base_ms = g_sys.fan_delay_sec * 1000UL;
                uint32_t force_ms = g_sys.fan_delay_sec * FAN_DELAY_FORCE_MULT * 1000UL;

                if (g_sys.hot_water_detected) {
                    g_sys.state = SYS_STATE_HOT_WATER;
                } else if (elapsed < base_ms) {
                    /* 基础延时阶段: 无条件运行 */
                } else if (g_sys.humidity_converged) {
                    g_sys.state = SYS_STATE_IDLE;
                    PID_Reset(&g_pid);
                } else if (elapsed >= force_ms) {
                    g_sys.state = SYS_STATE_IDLE;
                    PID_Reset(&g_pid);
                    Debug_Print("强制超时停机\r\n");
                }
                break;
            }

            default:
                g_sys.state = SYS_STATE_IDLE;
                break;
            }
        }
    }

    if (prev != g_sys.state) {
        Debug_Print("State: %d -> %d\r\n", prev, g_sys.state);
    }
}

/* ======================== 执行输出 ======================== */
static void App_ExecuteOutput(void)
{
    static uint32_t beep_tick = 0;
    static bool beep_flip = false;
    uint16_t min_duty = (uint16_t)(PWM_MAX_DUTY * HOT_WATER_MIN_DUTY_PCT / 100);
    uint16_t temp_limit = (uint16_t)(PWM_MAX_DUTY * TEMP_LIMIT_MAX_DUTY_PCT / 100);
    uint16_t outdoor_degrade_duty = (uint16_t)(PWM_MAX_DUTY * OUTDOOR_DEGRADE_DUTY_PCT / 100);

    /* 喂狗 */
    HAL_IWDG_Refresh(&hiwdg);

    switch (g_sys.state) {
    case SYS_STATE_PREHEAT:
        g_sys.pwm_duty = 0;
        Fan_Stop();
        /* 黄灯慢闪表示预热中 */
        LED_Set(false, (HAL_GetTick() / 500) & 1, false);
        Buzzer_Off();
        break;

    case SYS_STATE_GAS_ALARM:
        g_sys.pwm_duty = PWM_MAX_DUTY;
        Fan_SetDuty(g_sys.pwm_duty);
        LED_Set(false, false, true);
        if (HAL_GetTick() - beep_tick >= 200) {
            beep_tick = HAL_GetTick();
            beep_flip = !beep_flip;
            if (beep_flip) Buzzer_On(); else Buzzer_Off();
        }
        break;

    case SYS_STATE_STALL_FAULT:
        g_sys.pwm_duty = 0;
        Fan_Stop();
        LED_Set(false, false, true);
        if (HAL_GetTick() - beep_tick >= 500) {
            beep_tick = HAL_GetTick();
            beep_flip = !beep_flip;
            if (beep_flip) Buzzer_On(); else Buzzer_Off();
        }
        break;

    case SYS_STATE_STALL_GAS_EMERG: {
        /* 更急促的蜂鸣器报警: 100ms 翻转 */
        LED_Set(false, false, true);
        if (HAL_GetTick() - beep_tick >= 100) {
            beep_tick = HAL_GetTick();
            beep_flip = !beep_flip;
            if (beep_flip) Buzzer_On(); else Buzzer_Off();
        }

        /* --- 间歇试启动逻辑 --- */
        uint16_t emerg_duty = (uint16_t)(PWM_MAX_DUTY * EMERG_RETRY_DUTY_PCT / 100);
        uint32_t now_e = HAL_GetTick();

        if (g_sys.emerg_phase == 0) {
            /* 冷却/等待阶段 */
            g_sys.pwm_duty = 0;
            Fan_Stop();
            if (now_e - g_sys.emerg_retry_tick >= EMERG_RETRY_INTERVAL_MS) {
                /* 开始试启动 */
                g_sys.emerg_phase = 1;
                g_sys.emerg_retry_tick = now_e;
                g_sys.emerg_running = true;
                Debug_Print("EMERG: 试启动\r\n");
            }
        } else {
            /* 试运行阶段 */
            uint32_t run_elapsed = now_e - g_sys.emerg_retry_tick;

            /* 电流异常检查: 仅在旋转建立宽限期之后 (启动浪涌电流正常, 不检查) */
            bool current_abnormal_e = false;
            if (run_elapsed >= EMERG_SPINUP_GRACE_MS) {
                int16_t diff_e = (int16_t)g_sys.current_adc - (int16_t)g_sys.acs712_zero_adc;
                if (diff_e < 0) diff_e = -diff_e;
                current_abnormal_e = (diff_e > (int16_t)g_sys.acs712_stall_threshold);
            }

            if (current_abnormal_e) {
                /* 电流异常, 立即停止 */
                g_sys.emerg_phase = 0;
                g_sys.emerg_retry_tick = now_e;
                g_sys.emerg_running = false;
                g_sys.pwm_duty = 0;
                Fan_Stop();
                Debug_Print("EMERG: 电流异常, 停止\r\n");
            } else if (run_elapsed >= (EMERG_SPINUP_GRACE_MS + EMERG_RETRY_DURATION_MS)
                       && g_sys.fan_rpm < EMERG_RPM_MIN) {
                /* 旋转建立宽限 + 检测窗口均已过, RPM 仍过低 → 停止 */
                g_sys.emerg_phase = 0;
                g_sys.emerg_retry_tick = now_e;
                g_sys.emerg_running = false;
                g_sys.pwm_duty = 0;
                Fan_Stop();
                Debug_Print("EMERG: RPM过低, 停止\r\n");
            } else {
                /* 正在试运行 / 试启动成功继续运行 */
                g_sys.pwm_duty = emerg_duty;
                Fan_SetDuty(emerg_duty);
            }
        }
        break;
    }

    case SYS_STATE_HOT_WATER: {
        /* --- 温度通道 --- */
        float delta_t_hw = g_sys.ir_object_temp - g_sys.ambient_temp;
        float duty_temp_hw = Calc_TempDuty(delta_t_hw, g_sys.ambient_temp);

        /* --- 湿度通道 (PID) --- */
        float duty_humi_hw;
        if (!g_sys.outdoor_sensor_ok) {
            /* 室外传感器离线 → 湿度通道使用保守固定值 */
            duty_humi_hw = (float)outdoor_degrade_duty;
        } else {
            /* 每次计算前刷新 setpoint, 保证湿度基线实时跟踪室外变化 */
            PID_SetSetpoint(&g_pid, g_sys.outdoor_abs_humi);
            duty_humi_hw = PID_Compute(&g_pid, g_sys.indoor_abs_humi);
        }

        /* --- 取较大值: 谁更急谁主导 --- */
        float target_duty_hw = (duty_temp_hw > duty_humi_hw) ? duty_temp_hw : duty_humi_hw;

        /* --- 安全约束 --- */
        if (target_duty_hw < (float)min_duty) target_duty_hw = (float)min_duty;
        if (g_sys.ambient_temp < TEMP_LOW_LIMIT && target_duty_hw > (float)temp_limit) {
            target_duty_hw = (float)temp_limit;
        }

        /* --- 斜率限制 → 最终输出 --- */
        g_sys.pwm_duty = SlewRateLimit(target_duty_hw, g_sys.pwm_duty);
        Fan_SetDuty(g_sys.pwm_duty);
        LED_Set(false, true, false);
        Buzzer_Off();

        /* --- 串口调试 (每 2s) --- */
        {
            static uint32_t dbg_hw_tick = 0;
            if (HAL_GetTick() - dbg_hw_tick >= 2000) {
                dbg_hw_tick = HAL_GetTick();
                Debug_Print("HW d_T:%u d_H:%u max:%u slew:%u PWM:%u%% RPM:%u\r\n",
                            (uint16_t)duty_temp_hw, (uint16_t)duty_humi_hw,
                            (uint16_t)target_duty_hw, g_sys.pwm_duty,
                            (g_sys.pwm_duty * 100) / PWM_MAX_DUTY, g_sys.fan_rpm);
            }
        }
        break;
    }

    case SYS_STATE_DELAY_STOP: {
        /* --- 温度通道 (热水已停, 温差逐渐缩小) --- */
        float delta_t_ds = g_sys.ir_object_temp - g_sys.ambient_temp;
        float duty_temp_ds = Calc_TempDuty(delta_t_ds, g_sys.ambient_temp);

        /* --- 湿度通道 --- */
        float duty_humi_ds;
        if (!g_sys.outdoor_sensor_ok) {
            duty_humi_ds = (float)outdoor_degrade_duty;
        } else {
            /* 每次计算前刷新 setpoint, 保证湿度基线实时跟踪室外变化 */
            PID_SetSetpoint(&g_pid, g_sys.outdoor_abs_humi);
            duty_humi_ds = PID_Compute(&g_pid, g_sys.indoor_abs_humi);
        }

        /* --- 湿度主导, 温度衰减参与 --- */
        /* 衰减窗口绑定基础延时: decay_ms = fan_delay_sec * 1000 */
        uint32_t decay_ms = g_sys.fan_delay_sec * 1000UL;
        float decay = 1.0f - ((float)(HAL_GetTick() - g_sys.delay_stop_enter_tick)
                              / (float)decay_ms);
        if (decay < 0.0f) decay = 0.0f;
        float duty_temp_decayed = duty_temp_ds * decay;

        /* 取湿度需求与衰减后温度需求的较大值 */
        float target_duty_ds = (duty_humi_ds > duty_temp_decayed) ? duty_humi_ds : duty_temp_decayed;

        /* --- 安全约束: DELAY_STOP 最低用 FAN_4WIRE_MIN_PWM_PCT --- */
        uint16_t ds_min_duty = (uint16_t)(PWM_MAX_DUTY * FAN_4WIRE_MIN_PWM_PCT / 100);
        if (target_duty_ds < (float)ds_min_duty) target_duty_ds = (float)ds_min_duty;
        if (g_sys.ambient_temp < TEMP_LOW_LIMIT && target_duty_ds > (float)temp_limit) {
            target_duty_ds = (float)temp_limit;
        }

        /* --- 斜率限制 → 最终输出 --- */
        g_sys.pwm_duty = SlewRateLimit(target_duty_ds, g_sys.pwm_duty);
        Fan_SetDuty(g_sys.pwm_duty);
        LED_Set(true, true, false);
        Buzzer_Off();

        /* --- 串口调试 (每 2s) --- */
        {
            static uint32_t dbg_ds_tick = 0;
            if (HAL_GetTick() - dbg_ds_tick >= 2000) {
                dbg_ds_tick = HAL_GetTick();
                Debug_Print("DS d_T:%u d_H:%u dcy:%.2f tgt:%u slew:%u RPM:%u\r\n",
                            (uint16_t)duty_temp_ds, (uint16_t)duty_humi_ds,
                            decay, (uint16_t)target_duty_ds, g_sys.pwm_duty, g_sys.fan_rpm);
            }
        }
        break;
    }

    case SYS_STATE_IDLE:
    default:
        g_sys.pwm_duty = 0;
        Fan_Stop();
        LED_Set(true, false, false);
        Buzzer_Off();
        break;
    }
}

/* ======================== 显示更新 ======================== */
static void App_UpdateDisplay(void)
{
    static uint32_t last_refresh = 0;
    /* 限制刷新频率, 避免主循环无延时时 OLED 刷新过快 */
    if (HAL_GetTick() - last_refresh < 200) return;
    last_refresh = HAL_GetTick();

    char line[32];

    OLED_Clear();

    /* 第 0 行: 模式 + 传感器状态 */
    switch (g_sys.state) {
    case SYS_STATE_PREHEAT:
        snprintf(line, sizeof(line), "MODE:PREHEAT %lus",
                 (unsigned long)((MQ2_PREHEAT_TIME_MS - (HAL_GetTick() - g_sys.preheat_start_tick)) / 1000));
        OLED_ShowString(0, 0, line);
        break;
    case SYS_STATE_GAS_ALARM:
        OLED_ShowString(0, 0, "MODE:GAS ALARM");
        break;
    case SYS_STATE_STALL_FAULT:
        OLED_ShowString(0, 0, "MODE:STALL ERR");
        break;
    case SYS_STATE_STALL_GAS_EMERG:
        OLED_ShowString(0, 0, "GAS+FAN FAULT");
        break;
    case SYS_STATE_HOT_WATER:
        OLED_ShowString(0, 0, g_sys.outdoor_sensor_ok ? "MODE:HOT WATER" : "MODE:HOT W(DEG)");
        break;
    case SYS_STATE_DELAY_STOP:
        OLED_ShowString(0, 0, "MODE:DEHUMID");
        break;
    default:
        OLED_ShowString(0, 0, "MODE:IDLE");
        break;
    }

    /* 第 1 行: 室内温湿度 + 传感器状态 */
    snprintf(line, sizeof(line), "In:%4.1fC %2.0f%%%c",
             g_sys.ambient_temp, g_sys.ambient_humi,
             g_sys.indoor_aht20_ok ? ' ' : '!');
    OLED_ShowString(0, 1, line);

    /* 第 2 行: 室外温湿度 */
    if (g_sys.outdoor_sensor_ok) {
        snprintf(line, sizeof(line), "Out:%4.1fC %2.0f%%", g_sys.outdoor_temp, g_sys.outdoor_humi);
    } else {
        snprintf(line, sizeof(line), "Out:OFFLINE");
    }
    OLED_ShowString(0, 2, line);

    /* 第 3 行: 绝对湿度差 & 水温 + MLX 状态 */
    snprintf(line, sizeof(line), "dAH:%4.1f Tw:%4.1f%c",
             g_sys.indoor_abs_humi - g_sys.outdoor_abs_humi, g_sys.ir_object_temp,
             g_sys.mlx90614_obj_ok ? ' ' : '!');
    OLED_ShowString(0, 3, line);

    /* 第 4 行: PWM & RPM + 温度通道ΔT */
    {
        float disp_dt = g_sys.ir_object_temp - g_sys.ambient_temp;
        snprintf(line, sizeof(line), "P:%2u%% R:%u dT:%.0f",
                 (g_sys.pwm_duty * 100) / PWM_MAX_DUTY, g_sys.fan_rpm, disp_dt);
    }
    OLED_ShowString(0, 4, line);

    /* 第 5 行: 气体 & 电流 */
    snprintf(line, sizeof(line), "Gas:%4u I:%1.2fA", g_sys.gas_adc, g_sys.fan_current_a);
    OLED_ShowString(0, 5, line);

    /* 第 6 行: 延时 / 状态 — Base/Force 分离 */
    if (g_sys.state == SYS_STATE_DELAY_STOP) {
        uint32_t elapsed_s = (HAL_GetTick() - g_sys.delay_start_tick) / 1000UL;
        uint32_t base_s = g_sys.fan_delay_sec;
        uint32_t force_s = g_sys.fan_delay_sec * FAN_DELAY_FORCE_MULT;
        uint32_t base_rem = (elapsed_s < base_s) ? (base_s - elapsed_s) : 0;
        uint32_t force_rem = (elapsed_s < force_s) ? (force_s - elapsed_s) : 0;

        if (base_rem > 0) {
            snprintf(line, sizeof(line), "B:%lus F:%lus",
                     (unsigned long)base_rem, (unsigned long)force_rem);
        } else {
            snprintf(line, sizeof(line), "B:OK F:%lus",
                     (unsigned long)force_rem);
        }
        OLED_ShowString(0, 6, line);
        OLED_ShowString(90, 6, g_sys.humidity_converged ? "H:OK" : "H:HI");
    } else {
        snprintf(line, sizeof(line), "Dly:%lus", (unsigned long)g_sys.fan_delay_sec);
        OLED_ShowString(0, 6, line);
        OLED_ShowString(66, 6, g_sys.gas_detected ? "Gas:ALM" : "Gas:NOR");
    }

    OLED_Refresh();
}

/* ======================== 按键处理 (消费 SysTick 中断设置的标志) ======================== */
static void App_HandleKeys(void)
{
    /* 原子快照: 关中断取值并清零, 防止 SysTick 置位与清零竞争 */
    __disable_irq();
    uint8_t evt = g_sys.key_event;
    g_sys.key_event = KEY_EVENT_NONE;
    __enable_irq();

    if (evt == KEY_EVENT_NONE) return;

    if (evt & KEY_EVENT_RESET) {
        if (g_sys.stall_latched) {
            g_sys.stall_latched = false;
            g_sys.stall_detected = false;
            g_sys.stall_start_tick = 0;
            g_sys.rpm_low_start_tick = 0;
            g_sys.stall_latch_pending_tick = 0;
            g_sys.emerg_phase = 0;
            g_sys.emerg_running = false;
            g_sys.emerg_retry_tick = 0;
            Debug_Print("堵转复位 (KEY1)\r\n");
        }
    }
    if (evt & KEY_EVENT_DELAY_UP) {
        if (g_sys.fan_delay_sec < FAN_DELAY_MAX) {
            g_sys.fan_delay_sec += FAN_DELAY_STEP;
            g_sys.flash_dirty = true;
            g_sys.flash_dirty_tick = HAL_GetTick();
        }
    }
    if (evt & KEY_EVENT_DELAY_DN) {
        if (g_sys.fan_delay_sec > FAN_DELAY_MIN) {
            g_sys.fan_delay_sec -= FAN_DELAY_STEP;
            g_sys.flash_dirty = true;
            g_sys.flash_dirty_tick = HAL_GetTick();
        }
    }
}

/* ======================== Flash 延迟保存 ======================== */
static void App_FlashDeferredSave(void)
{
    if (g_sys.flash_dirty &&
        (HAL_GetTick() - g_sys.flash_dirty_tick >= FLASH_SAVE_DEFER_MS)) {
        FlashStorage_t save_data;
        save_data.magic = FLASH_STORAGE_MAGIC;
        save_data.pid_kp = g_pid.Kp;
        save_data.pid_ki = g_pid.Ki;
        save_data.pid_kd = g_pid.Kd;
        save_data.fan_delay_sec = g_sys.fan_delay_sec;
        save_data.acs_zero_adc = g_sys.acs712_zero_adc;
        save_data.reserved = 0;
        if (Flash_Save(&save_data)) {
            g_sys.flash_dirty = false;
            Debug_Print("延时已保存: %lu s\r\n", (unsigned long)g_sys.fan_delay_sec);
        } else {
            g_sys.flash_dirty_tick = HAL_GetTick();
            Debug_Print("WARN: Flash 保存失败, 稍后重试\r\n");
        }
    }
}

/* ======================== ACS712 启动峰值自适应 ======================== */
static void App_ACS712_PeakTrack(void)
{
    if (g_sys.fan_running) {
        if (!g_sys.acs712_startup_done) {
            if (g_sys.acs712_startup_tick == 0) {
                g_sys.acs712_startup_tick = HAL_GetTick();
                g_sys.acs712_peak_adc = 0;
            }

            int16_t diff = (int16_t)g_sys.current_adc - (int16_t)g_sys.acs712_zero_adc;
            if (diff < 0) diff = -diff;
            uint16_t abs_diff = (uint16_t)diff;
            if (abs_diff > g_sys.acs712_peak_adc) {
                g_sys.acs712_peak_adc = abs_diff;
            }

            if ((HAL_GetTick() - g_sys.acs712_startup_tick) >= ACS712_STARTUP_CALIB_MS) {
                if (g_sys.acs712_peak_adc > 10) {
                    g_sys.acs712_stall_threshold =
                        (uint16_t)(g_sys.acs712_peak_adc * ACS712_STALL_MULTIPLIER_NUM / ACS712_STALL_MULTIPLIER_DEN);
                }
                g_sys.acs712_startup_done = true;
                Debug_Print("ACS712 峰值:%u 堵转阈值:%u\r\n", g_sys.acs712_peak_adc, g_sys.acs712_stall_threshold);
            }
        }
    } else {
        /* 风扇停转时复位, 下次启动重新学习 */
        if (g_sys.acs712_startup_done || g_sys.acs712_startup_tick != 0) {
            g_sys.acs712_startup_done = false;
            g_sys.acs712_startup_tick = 0;
            g_sys.acs712_peak_adc = 0;
        }
    }
}

/* ======================== 核心算法: 热水判定 (迟滞比较) ======================== */
static bool Detect_HotWater(float t_water, float t_ambient)
{
    static bool hot_state = false;
    float delta = t_water - t_ambient;

    if (!hot_state) {
        if (delta >= HOTWATER_ON_THRESHOLD) hot_state = true;
    } else {
        if (delta < HOTWATER_OFF_THRESHOLD) hot_state = false;
    }
    return hot_state;
}

/* ======================== 核心算法: 气体/堵转判断 ======================== */
static bool Detect_GasAlarm(uint16_t gas_adc)
{
    static uint8_t alarm_count = 0;
    static uint8_t release_count = 0;
    static bool gas_alarm_state = false;

    if (!gas_alarm_state) {
        if (gas_adc >= GAS_ALARM_THRESHOLD) {
            release_count = 0;
            if (++alarm_count >= GAS_ALARM_CONFIRM_CNT) return (gas_alarm_state = true);
        } else {
            alarm_count = 0;
        }
    } else {
        if (gas_adc < GAS_RELEASE_THRESHOLD) {
            alarm_count = 0;
            if (++release_count >= GAS_RELEASE_CONFIRM_CNT) {
                gas_alarm_state = false;
            }
        } else {
            release_count = 0;
        }
    }
    return gas_alarm_state;
}

static bool Detect_FanStall(uint16_t current_adc, uint16_t pwm_duty)
{
    if (pwm_duty <= (PWM_MAX_DUTY * 30 / 100)) {
        g_sys.stall_start_tick = 0;
        g_sys.rpm_low_start_tick = 0;
        return false;
    }

    /* 判据1: 电流异常偏大 (堵转时电机锁定电流升高) */
    int16_t diff = (int16_t)current_adc - (int16_t)g_sys.acs712_zero_adc;
    if (diff < 0) diff = -diff;
    bool current_abnormal = (diff > (int16_t)g_sys.acs712_stall_threshold);

    /* 判据2: RPM 持续低于阈值 (有 PWM 输出但风扇不转) */
    bool rpm_low = (g_sys.fan_rpm < STALL_RPM_MIN);

    if (rpm_low) {
        if (g_sys.rpm_low_start_tick == 0) {
            g_sys.rpm_low_start_tick = HAL_GetTick();
        }
    } else {
        g_sys.rpm_low_start_tick = 0;
    }
    bool rpm_timeout = (g_sys.rpm_low_start_tick != 0) &&
                       ((HAL_GetTick() - g_sys.rpm_low_start_tick) >= STALL_RPM_TIMEOUT_MS);

    /* 电流异常持续计时 */
    if (current_abnormal) {
        if (g_sys.stall_start_tick == 0) {
            g_sys.stall_start_tick = HAL_GetTick();
        }
    } else {
        g_sys.stall_start_tick = 0;
    }
    bool current_timeout = (g_sys.stall_start_tick != 0) &&
                           ((HAL_GetTick() - g_sys.stall_start_tick) >= STALL_DETECT_TIME_MS);

    /* 任一条件满足即判定堵转 */
    return (current_timeout || rpm_timeout);
}

static float Convert_ACS712_ToCurrent(uint16_t adc_value)
{
    float v_adc = 3.3f * adc_value / 4095.0f;
    float v_zero = 3.3f * g_sys.acs712_zero_adc / 4095.0f;
    float v_sensor = (v_adc - v_zero) / 0.6f;
    float current = v_sensor / 0.185f;
    if (fabsf(current) < 0.03f) current = 0.0f;
    return fabsf(current);
}

/* ======================== ACS712 自适应零点校准 ======================== */
static void ACS712_CalibrateZero(void)
{
    uint32_t sum = 0;
    for (uint8_t i = 0; i < ACS712_CALIB_SAMPLES; i++) {
        sum += ADC_ReadChannel(ACS712_ADC_CHANNEL);
        HAL_Delay(5);
    }
    g_sys.acs712_zero_adc = (uint16_t)(sum / ACS712_CALIB_SAMPLES);
    g_sys.acs712_stall_threshold = ACS712_STALL_THRESHOLD_DEFAULT;
    Debug_Print("ACS712 零点: %u\r\n", g_sys.acs712_zero_adc);
}

/* ======================== I2C2 总线死锁恢复 ======================== */
static void I2C2_BusRecovery(void)
{
    GPIO_InitTypeDef gpio;

    /* 关闭 I2C2 外设 */
    HAL_I2C_DeInit(&hi2c2);

    /* 将 PB10(SCL) 和 PB11(SDA) 都切为开漏输出 + 上拉 */
    gpio.Pin = GPIO_PIN_10;
    gpio.Mode = GPIO_MODE_OUTPUT_OD;
    gpio.Pull = GPIO_PULLUP;
    gpio.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(GPIOB, &gpio);

    gpio.Pin = GPIO_PIN_11;
    gpio.Mode = GPIO_MODE_OUTPUT_OD;
    gpio.Pull = GPIO_PULLUP;
    gpio.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(GPIOB, &gpio);

    /* 释放 SDA (拉高) */
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_11, GPIO_PIN_SET);

    /* 发送 9 个时钟脉冲, 让从设备释放 SDA */
    for (uint8_t i = 0; i < 9; i++) {
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_10, GPIO_PIN_RESET);
        HAL_Delay(1);
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_10, GPIO_PIN_SET);
        HAL_Delay(1);
        if (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_11) == GPIO_PIN_SET) break;
    }

    /* 生成 STOP 条件: SDA 低 → SCL 高 → SDA 高 */
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_11, GPIO_PIN_RESET); /* SDA 低 */
    HAL_Delay(1);
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_10, GPIO_PIN_SET);   /* SCL 高 */
    HAL_Delay(1);
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_11, GPIO_PIN_SET);   /* SDA 高 (STOP) */
    HAL_Delay(1);

    /* 校验两线都为高 */
    if (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_10) != GPIO_PIN_SET ||
        HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_11) != GPIO_PIN_SET) {
        Debug_Print("WARN: I2C2 恢复后总线仍异常\r\n");
    }

    /* 重新初始化 I2C2 */
    MX_I2C2_Init();
    Debug_Print("I2C2 总线恢复\r\n");
}

/* ======================== 通用底层函数 ======================== */
static uint16_t ADC_ReadChannel(uint32_t channel)
{
    static uint16_t last_val = 0; /* 上一次有效值, 故障时返回 */
    ADC_ChannelConfTypeDef sConfig = {0};
    sConfig.Channel = channel;
    sConfig.Rank = ADC_REGULAR_RANK_1;
    sConfig.SamplingTime = ADC_SAMPLETIME_239CYCLES_5;

    if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK) {
        return last_val; /* 通道配置失败, 返回上次值 */
    }

    if (HAL_ADC_Start(&hadc1) != HAL_OK) {
        return last_val;
    }

    if (HAL_ADC_PollForConversion(&hadc1, 20) != HAL_OK) {
        HAL_ADC_Stop(&hadc1);
        return last_val; /* 轮询超时, 返回上次值 */
    }

    uint16_t val = HAL_ADC_GetValue(&hadc1);
    HAL_ADC_Stop(&hadc1);
    last_val = val;
    return val;
}

static uint16_t ADC_ReadFiltered(uint32_t channel, uint8_t count)
{
    uint32_t sum = 0;
    for (uint8_t i = 0; i < count; i++) {
        sum += ADC_ReadChannel(channel);
    }
    return (uint16_t)(sum / count);
}

static float Filter_SlidingAverage(float new_value, float *buf, uint8_t size, uint8_t *index, bool *filled)
{
    float sum = 0.0f;
    uint8_t valid_count;

    buf[*index] = new_value;
    *index = (*index + 1) % size;
    if (*index == 0) *filled = true;

    valid_count = *filled ? size : *index;
    if (valid_count == 0) valid_count = 1;  /* 防止除零 */

    for (uint8_t i = 0; i < valid_count; i++) sum += buf[i];
    return sum / (float)valid_count;
}

/* ======================== 温度通道: 需求计算 ======================== */
/**
 * @brief 计算温度通道的风速需求 (水温激励 + 室温修正)
 * @param delta_t       水温 - 室温 温差 (℃)
 * @param ambient_temp  室内温度 (℃)
 * @return 温度通道需求 PWM 值 (0 ~ PWM_MAX_DUTY)
 */
static float Calc_TempDuty(float delta_t, float ambient_temp)
{
    /* 1. 温差线性映射: ΔT → 占空比百分比 */
    float duty_pct;
    if (delta_t <= TEMP_DELTA_LOW) {
        duty_pct = TEMP_DUTY_LOW_PCT;
    } else if (delta_t >= TEMP_DELTA_HIGH) {
        duty_pct = TEMP_DUTY_HIGH_PCT;
    } else {
        /* 线性插值 */
        duty_pct = TEMP_DUTY_LOW_PCT +
                   (delta_t - TEMP_DELTA_LOW) *
                   (TEMP_DUTY_HIGH_PCT - TEMP_DUTY_LOW_PCT) /
                   (TEMP_DELTA_HIGH - TEMP_DELTA_LOW);
    }

    float duty = (float)PWM_MAX_DUTY * duty_pct / 100.0f;

    /* 2. 室温修正: 高温环境蒸发更猛 → 增大排风 */
    float correction = 1.0f + (ambient_temp - TEMP_CORRECTION_CENTER) * TEMP_CORRECTION_RATE;
    if (correction < TEMP_CORRECTION_MIN) correction = TEMP_CORRECTION_MIN;
    if (correction > TEMP_CORRECTION_MAX) correction = TEMP_CORRECTION_MAX;

    duty *= correction;

    /* 3. 钳位 */
    if (duty < 0.0f) duty = 0.0f;
    if (duty > (float)PWM_MAX_DUTY) duty = (float)PWM_MAX_DUTY;

    return duty;
}

/* ======================== 升降速斜率限制 (基于真实时间) ======================== */
/**
 * @brief 对目标占空比施加升降速斜率限制, 避免需求切换导致风扇抖动
 *
 * 斜率限制作用于 "输出端", 不是传感器。
 * 使用真实时间间隔 dt 换算允许变化量, 与 PID 的时间基准一致,
 * 无论主循环快慢, 升/降速体感都相同。
 *
 * @param target   本次计算的目标占空比 (经过安全约束后)
 * @param current  当前实际输出的占空比 (上一轮 final_duty)
 * @return 斜率限制后的最终占空比
 */
static uint16_t SlewRateLimit(float target, uint16_t current)
{
    /* 记录上一次执行时刻, 用于计算真实 dt */
    static uint32_t last_slew_tick = 0;
    uint32_t now = HAL_GetTick();
    float dt;

    if (last_slew_tick == 0) {
        /* 首次调用: 记录时间基准, 从 current 出发用最小 dt 做一次正常限制 */
        last_slew_tick = now;
        dt = 0.001f; /* 最小 dt, 等效只允许极小变化, 不会跳变 */
    } else {
        dt = (float)(now - last_slew_tick) / 1000.0f;  /* 转为秒 */
        last_slew_tick = now;
    }

    /* dt 钳位: 最小 1ms 防除零, 最大 2s 防长时间挂起后暴冲 */
    if (dt < 0.001f) dt = 0.001f;
    if (dt > 2.0f) dt = 2.0f;

    float max_up   = (float)SLEW_RATE_UP_PER_SEC * dt;
    float max_down = (float)SLEW_RATE_DOWN_PER_SEC * dt;

    float diff = target - (float)current;

    if (diff > 0) {
        /* 升速: 有蒸汽时要尽快拉风量 */
        if (diff > max_up) diff = max_up;
    } else {
        /* 降速: 蒸汽刚减弱时不要立刻掉速 */
        if (diff < -max_down) diff = -max_down;
    }

    float result = (float)current + diff;
    if (result < 0.0f) result = 0.0f;
    if (result > (float)PWM_MAX_DUTY) result = (float)PWM_MAX_DUTY;

    return (uint16_t)result;
}

static void Fan_SetDuty(uint16_t duty)
{
    if (duty > PWM_MAX_DUTY) duty = PWM_MAX_DUTY;
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, duty);
    g_sys.fan_running = (duty > 0);
}

static void Fan_Stop(void)
{
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 0);
    g_sys.fan_running = false;
}

static void Buzzer_On(void)
{
    HAL_GPIO_WritePin(BUZZER_GPIO_Port, BUZZER_Pin, GPIO_PIN_RESET);
}

static void Buzzer_Off(void)
{
    HAL_GPIO_WritePin(BUZZER_GPIO_Port, BUZZER_Pin, GPIO_PIN_SET);
}

static void LED_Set(bool green, bool yellow, bool red)
{
    HAL_GPIO_WritePin(LED_GREEN_GPIO_Port,  LED_GREEN_Pin,  green  ? GPIO_PIN_RESET : GPIO_PIN_SET);
    HAL_GPIO_WritePin(LED_YELLOW_GPIO_Port, LED_YELLOW_Pin, yellow ? GPIO_PIN_RESET : GPIO_PIN_SET);
    HAL_GPIO_WritePin(LED_RED_GPIO_Port,    LED_RED_Pin,    red    ? GPIO_PIN_RESET : GPIO_PIN_SET);
}

static void Debug_Print(const char *fmt, ...)
{
    char buf[128];
    va_list args;
    va_start(args, fmt);
    vsnprintf(buf, sizeof(buf), fmt, args);
    va_end(args);
    HAL_UART_Transmit(&huart1, (uint8_t *)buf, strlen(buf), 100);
}

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
