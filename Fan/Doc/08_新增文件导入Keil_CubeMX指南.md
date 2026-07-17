# 新增文件导入 Keil / CubeMX 指南

本文档说明当你新增 `.c` / `.h` 文件后，如何正确地将它们加入 Keil MDK 和 CubeMX 工程中，确保编译通过。

---

## 一、核心概念

### CubeMX 管理的文件 vs 用户自定义文件

| 类型 | 示例 | CubeMX 生成 | CubeMX 重新生成时 |
|------|------|------------|------------------|
| **CubeMX 管理** | `main.c`, `gpio.c`, `i2c.c`, `tim.c` | 是 | 会覆盖 (但保留 USER CODE 区域) |
| **用户自定义** | `bsp_aht20.c`, `bsp_pid.c` 等 | 否 | **不会覆盖, 但也不会自动加入 Keil** |

关键结论：
- **头文件 (.h)** 只要放在 Include Path 包含的目录下 (如 `Core/Inc/`)，编译器会自动找到，**无需额外操作**
- **源文件 (.c)** 必须**手动**添加到 Keil 工程的编译列表中，否则链接时会报 `undefined symbol`

---

## 二、Keil MDK 添加新文件 — 完整步骤

### 场景：新增 `bsp_xxx.c` 和 `bsp_xxx.h`

#### 步骤 1：将文件放到正确目录

```
Core/Inc/bsp_xxx.h      ← 头文件
Core/Src/bsp_xxx.c      ← 源文件
```

#### 步骤 2：在 Keil 中注册 .c 文件

1. 打开 Keil，菜单 **Project → Manage Project Items**（或快捷键无，使用菜单）

2. 在弹出的窗口中，左侧是 **Groups** (分组)，右侧是 **Files** (文件列表)

3. 选择你要添加到的分组 (例如 `Core/Src` 或 `Application/User/Core`)
   - 如果没有合适的分组，点 **New (Insert)** 创建一个新分组，如 `BSP_Drivers`

4. 选中分组后，点右侧的 **Add Files**

5. 在文件浏览器中定位到 `Core/Src/`，选择 `bsp_xxx.c`

6. 点 **Add**，然后 **Close**

7. 点 **OK** 关闭 Manage Project Items 窗口

#### 步骤 3：确认 Include Path

1. 菜单 **Project → Options for Target** (或按 Alt+F7)
2. 切到 **C/C++** 选项卡
3. 检查 **Include Paths** 是否包含 `Core/Inc`
4. 如果你的 .h 文件放在其他目录，也要添加该目录

```
../Core/Inc
../Drivers/STM32F1xx_HAL_Driver/Inc
../Drivers/CMSIS/Device/ST/STM32F1xx/Include
../Drivers/CMSIS/Include
```

#### 步骤 4：在使用的 .c 文件中 #include 头文件

```c
/* USER CODE BEGIN Includes */
#include "bsp_xxx.h"
/* USER CODE END Includes */
```

#### 步骤 5：编译验证

按 **F7** 编译，确认无 `undefined symbol` 错误。

### 图示：Manage Project Items 界面

```
┌─── Manage Project Items ──────────────────────────┐
│                                                     │
│  Project Targets:  SmartFan_New                     │
│                                                     │
│  ┌─── Groups ───┐  ┌─── Files ──────────────────┐  │
│  │ Core/Src     │  │ main.c                      │  │
│  │              │  │ gpio.c                       │  │
│  │              │  │ adc.c                        │  │
│  │              │  │ i2c.c                        │  │
│  │              │  │ tim.c                        │  │
│  │              │  │ usart.c                      │  │
│  │              │  │ stm32f1xx_it.c               │  │
│  │              │  │ stm32f1xx_hal_msp.c          │  │
│  │              │  │ bsp_aht20.c          ← 已加 │  │
│  │              │  │ bsp_mlx90614.c       ← 已加 │  │
│  │              │  │ bsp_oled.c           ← 已加 │  │
│  │              │  │ bsp_pid.c            ← 需加 │  │
│  ��              │  │ bsp_humidity.c       ← 需加 │  │
│  │              │  │ bsp_flash_storage.c  ← 需加 │  │
│  │              │  │ bsp_fan_tacho.c      ← 需加 │  │
│  │              │  │                              │  │
│  │ Drivers/...  │  │ (HAL 库文件)                 │  │
│  └──────────────┘  └──────────────────────────────┘  │
│                                                     │
│  [New (Insert)]  [Delete]     [Add Files]  [Delete]  │
│                               [OK]  [Cancel]         │
└─────────────────────────────────────────────────────┘
```

---

## 三、本项目当前 Keil 工程缺失的文件

经检查，当前 `SmartFan_New.uvprojx` 中已注册的用户 .c 文件：

| 文件 | 状态 |
|------|------|
| `main.c` | ✅ 已注册 |
| `gpio.c`, `adc.c`, `i2c.c`, `tim.c`, `usart.c` | ✅ 已注册 |
| `stm32f1xx_it.c`, `stm32f1xx_hal_msp.c`, `system_stm32f1xx.c` | ✅ 已注册 |
| `bsp_aht20.c` | ✅ 已注册 |
| `bsp_mlx90614.c` | ✅ 已注册 |
| `bsp_oled.c` | ✅ 已注册 |
| **`bsp_pid.c`** | ❌ **缺失 — 需手动添加** |
| **`bsp_humidity.c`** | ❌ **缺失 — 需手动添加** |
| **`bsp_flash_storage.c`** | ❌ **缺失 — 需手动添加** |
| **`bsp_fan_tacho.c`** | ❌ **缺失 — 需手动添加** |

**修复方法**：按上面"步骤 2"操作，将 4 个缺失的 .c 文件添加到 Keil 工程中。

---

## 四、CubeMX .ioc 缺失的配置

当前 `SmartFan.ioc` 中缺少以下代码实际使用的外设配置：

### 4.1 添加 I2C2 (室外 AHT20)

1. 打开 CubeMX，加载 `SmartFan.ioc`
2. 在 Pinout 视图中点击 **PB10** → 选择 **I2C2_SCL**
3. 点击 **PB11** → 选择 **I2C2_SDA**
4. 左侧 Connectivity → **I2C2** → Mode = **I2C**
5. Configuration → Speed = **100000** (100kHz Standard Mode)

### 4.2 添加 TIM3 CH2 输入捕获 (风扇测速)

1. 在 Pinout 视图中点击 **PA7** → 选择 **TIM3_CH2**
2. 左侧 Timers → **TIM3** → Clock Source = **Internal Clock**
3. Channel2 = **Input Capture direct mode**
4. Configuration → Counter Settings:
   - Prescaler = **71** (72MHz / 72 = 1MHz)
   - Counter Period = **65535** (0xFFFF)
5. Configuration → Input Capture Channel 2:
   - Polarity = **Falling Edge**
   - IC Selection = **Direct**
   - Prescaler = **No division**
   - Filter = **0**

### 4.3 使能 TIM3 中断

1. 左侧 System Core → **NVIC**
2. 找到 **TIM3 global interrupt** → 勾选 **Enabled**
3. 优先级建议设为 1 (低于 SysTick 的 15, 但高于其他)

### 4.4 重新生成代码

1. 点 **GENERATE CODE**
2. CubeMX 会更新 `i2c.c/h`（新增 MX_I2C2_Init）、`tim.c/h`（新增 MX_TIM3_Init）
3. **确认 USER CODE 区域被保留**
4. 回到 Keil，确认 bsp_*.c 仍在工程中

---

## 五、CubeMX 重新生成代码后的保护措施

### 5.1 USER CODE 区域

CubeMX 会保留所有 `/* USER CODE BEGIN xxx */` 和 `/* USER CODE END xxx */` 之间的代码。
因此，**所有用户代码必须写在 USER CODE 标记内**：

```c
/* USER CODE BEGIN Includes */
#include "bsp_config.h"    // ← 安全, 会被保留
/* USER CODE END Includes */

#include "some_lib.h"      // ← 危险! 会被覆盖删除!
```

### 5.2 不受影响的文件

以下文件 CubeMX **完全不会触碰**：
- `bsp_config.h`
- `bsp_aht20.c/h`
- `bsp_mlx90614.c/h`
- `bsp_oled.c/h`
- `bsp_pid.c/h`
- `bsp_humidity.c/h`
- `bsp_flash_storage.c/h`
- `bsp_fan_tacho.c/h`

### 5.3 Keil 工程文件 (.uvprojx)

CubeMX 重新生成时**会覆盖** `.uvprojx` 文件, 这意味着:
- 手动添加的 `bsp_*.c` 可能被从工程中移��
- **每次 CubeMX 重新生成后, 需要重新检查并添加缺失的 .c 文件**

**建议**: 在 `.mxproject` 文件的 `[PreviousUsedKeilFiles]` 区段中, CubeMX 不会管理你手动添加的文件。重新生成后只需再次按步骤 2 添加即可。

---

## 六、快速操作清单

### 新增一个 BSP 模块的完整流程

```
1. 创建文件
   Core/Inc/bsp_新模块.h
   Core/Src/bsp_新模块.c

2. 在 bsp_新模块.h 中:
   #include "bsp_config.h"   (获取 HAL 头文件和全局类型)

3. 在 main.c 的 USER CODE BEGIN Includes 中:
   #include "bsp_新模块.h"

4. 在 main.c 中调用新模块的初始化和功能函数
   (写在 USER CODE 区域内)

5. 在 Keil 中:
   Project → Manage Project Items → 选择分组
   → Add Files → 选择 bsp_新模块.c → OK

6. F7 编译 → 无错误 → F8 下载
```

### 修改 CubeMX 配置后的流程

```
1. 打开 SmartFan.ioc → 修改配置 → GENERATE CODE

2. 回到 Keil, 检查:
   ✅ USER CODE 区域代码是否保留
   ✅ bsp_*.c 是否仍在工程中 (缺失则重新添加)
   ✅ 新生成的外设 .c 是否已自动加入工程

3. F7 编译 → 修复可能的兼容性问题 → F8 下载
```

---

## 七、stm32f1xx_it.c 中断文件的特殊处理

`stm32f1xx_it.c` 是 CubeMX 管理的文件, 但本项目在其中添加了大量用户代码:
- SysTick 按键扫描 (USER CODE BEGIN SysTick_IRQn 1)
- TIM3_IRQHandler (USER CODE BEGIN 1)
- HAL_TIM_IC_CaptureCallback (USER CODE BEGIN 1)

**CubeMX 重新生成后**, 这些代码应会被保留 (因为都在 USER CODE 区域内)。但建议:
1. 重新生成前备份 `stm32f1xx_it.c`
2. 生成后对比确认用户代码完整
3. 特别注意 TIM3_IRQHandler — 如果 CubeMX 自动生成了该函数, 可能与用户手写版冲突, 需要合并

---

## 八、HAL 库模块启用 (stm32f1xx_hal_conf.h)

如果新增功能需要新的 HAL 模块, 需在 `stm32f1xx_hal_conf.h` 中取消对应宏的注释:

```c
#define HAL_ADC_MODULE_ENABLED      // ADC
#define HAL_I2C_MODULE_ENABLED      // I2C
#define HAL_TIM_MODULE_ENABLED      // TIM
#define HAL_UART_MODULE_ENABLED     // UART
#define HAL_FLASH_MODULE_ENABLED    // Flash (bsp_flash_storage 需要)
// #define HAL_SPI_MODULE_ENABLED   // ← 如果新增 SPI 设备, 取消注释
// #define HAL_IWDG_MODULE_ENABLED  // ← 本项目直接操作寄存器, 不需要 HAL IWDG
```

CubeMX 会根据启用的外设自动管理此文件, 通常不需要手动修改。
