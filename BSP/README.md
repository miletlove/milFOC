# BSP — 板级支持包 (Board Support Package)

> **职责**: 封装 STM32G431 硬件外设的底层操作，向上层 (Modules/App) 提供统一接口。本层不包含任何业务逻辑或算法。

---

## 📋 目录

- [架构概述](#架构概述)
- [文件清单与功能说明](#文件清单与功能说明)
- [调用关系](#调用关系)
- [使用示例](#使用示例)

---

## 架构概述

```
BSP/
├── bsp_init.h        # 统一初始化入口 (聚合所有 BSP Init)
├── bsp_dwt.c / .h     # DWT 高精度周期计数器
├── bsp_adc.c / .h     # ADC 注入组 + TIM1 同步触发
├── bsp_can.c / .h     # FDCAN 发送/接收/滤波/中断
├── bsp_usart.c / .h   # USART DMA/IT 多实例抽象
├── bsp_log.c / .h     # 分级日志系统 (依赖 bsp_usart)
├── bsp_flash.c / .h   # 内部 Flash 参数持久化
└── README.md          # 本文件
```

**初始化顺序** (由 `App/robot.c:RobotInit()` 调用)：
```
BSPInit()             → DWT_Init(168)     [必须最先初始化, 用于后续所有延时]
LogInit(&huart1)       → USARTRegister     [日志系统绑定 USART1]
adc_bsp_init()        → ADC1/2 注入组配置 [TIM1 触发配置]
```

---

## 文件清单与功能说明

### `bsp_init.h` — BSP 统一初始化入口

| 属性 | 说明 |
|------|------|
| **头文件** | `bsp_init.h` |
| **包含** | `bsp_dwt.h`, `bsp_adc.h`, `bsp_can.h`, `bsp_usart.h`, `bsp_log.h`, `bsp_flash.h` |
| **核心函数** | `BSPInit(void)` — 仅初始化 DWT (168MHz 核时钟) |
| **调用者** | `App/robot.c:RobotInit()` |
| **注意** | 必须在 RTOS 启动前和任何中断使能前调用。其他 BSP 外设 (CAN/USART) 由各自的模块在注册时初始化 |

---

### `bsp_dwt.c` / `bsp_dwt.h` — DWT 高精度周期计数器

| 属性 | 说明 |
|------|------|
| **硬件** | ARM CoreSight DWT (Data Watchpoint and Trace) 的 CYCCNT 寄存器 |
| **功能** | 启用 CPU 周期计数器 (CYCCNT)，提供纳秒级延时和代码性能分析 |
| **精度** | 1/168MHz ≈ 5.95ns (一个 CPU 时钟周期) |
| **核心函数** | `DWT_Init(SystemCoreCLK)` — 初始化, 传入系统时钟频率 (MHz) |
| | `DWT_Delay(float sec)` — 阻塞延时 (秒), 如 `DWT_Delay(0.016f)` = 16ms |
| | `DWT_GetDeltaT(uint32_t *t0)` — 获取自 t0 以来的时间差 (秒), 用于代码段耗时测量 |
| | `DWT_GetTimeline_ms(void)` — 获取系统上电后的累计时间 (毫秒) |
| **使用场景** | 编码器初始化前的 16ms 等待; ISR 执行时间测量; FOC 计算耗时统计 |
| **来源** | 原作者 Wang Hongxi (FalconFoc) |

---

### `bsp_adc.c` / `bsp_adc.h` — ADC 注入组同步采样

| 属性 | 说明 |
|------|------|
| **硬件** | ADC1 + ADC2 双 ADC 注入组, 由 TIM1 CH4 硬件触发 |
| **通道映射** | ADC1 JDR1=CUR_C(PA2), JDR2=CUR_B(PA1), JDR3=CUR_A(PA0), JDR4=VBUS(PB1) |
| **采样频率** | 20kHz (与 PWM 频率同步) |
| **触发机制** | TIM1 计数器溢出 → TIM1 CH4 OC 事件 → ADC1/2 注入组同步采样 |
| **核心宏** | `FAC_CURRENT` — ADC 原始值 → 电流 (A): `3.3V / 4096 / 0.01Ω` (基于分流电阻和运放增益) |
| | `VOLTAGE_TO_ADC_FACTOR` — ADC 原始值 → 母线电压 (V): `3.3V / 4096 * 11` |
| **核心函数** | `adc_bsp_init(void)` — 配置 ADC 注入组序列, 使能 TIM1 触发 |
| **调用者** | `App/robot.c:RobotInit()` (初始化); `Modules/motor/foc_motor.c` (读数) |
| **设计要点** | 注入组数据直接读取 `ADC1->JDRx` 寄存器, 绕过 HAL 开销, 确保电流环延迟最小 |

---

### `bsp_can.c` / `bsp_can.h` — FDCAN 发送/接收/滤波/中断

| 属性 | 说明 |
|------|------|
| **硬件** | FDCAN1 (PB8=RX, PB9=TX) |
| **模式** | Classic CAN, 标准 11-bit ID, 1Mbps |
| **状态** | ✅ 已板级验证通过 |
| **核心函数** | `bsp_can_init(void)` — CAN 初始化: 配置滤波器 → 启动外设 → 使能 RX FIFO0 中断 |
| | `can_filter_init(void)` — 配置掩码模式滤波器, 接收所有标准 ID 到 FIFO0 |
| | `fdcanx_send_data(FDCAN_HandleTypeDef *, uint16_t id, uint8_t *data, uint32_t len)` — 发送 CAN 帧 (经典 CAN 最大 8 字节) |
| | `fdcanx_receive(FDCAN_HandleTypeDef *, uint16_t *rec_id, uint8_t *buf)` — 从 FIFO0 读取一帧 (含 DLC→字节长度映射) |
| | `fdcan1_rx_callback(void)` — FDCAN1 接收回调 (在 HAL 中断上下文中调用) |
| **全局变量** | `rx_data1[8]` — 接收数据缓冲区 (供上层 can_driver 读取) |
| | `rec_id1` — 接收到的 CAN ID |
| **中断** | `HAL_FDCAN_RxFifo0Callback` — 覆盖 HAL 弱定义, 触发 `fdcan1_rx_callback` |
| **上层调用** | `Modules/comm/can_driver.c` 通过 `FDCANCommInit/Send/Get` 封装使用 |

---

### `bsp_usart.c` / `bsp_usart.h` — USART 多实例抽象

| 属性 | 说明 |
|------|------|
| **硬件** | USART1 (PB6=TX, PB7=RX), 115200bps |
| **最大实例** | 4 个 (可配置) |
| **传输模式** | DMA (优先), IT (中断), Blocking (阻塞) |
| **核心结构** | `USARTInstance` — 一个 USART 实例 (含句柄, 缓冲区, DMA/IT 状态) |
| **核心函数** | `USARTRegister(USART_Init_Config_s *)` — 注册一个 USART 实例 |
| | `USARTSend(USARTInstance *, uint8_t *data, uint16_t len, USART_TRANSFER_MODE)` — 发送数据 |
| **调用者** | `BSP/bsp_log.c` (日志输出); 其他需要串口通信的模块 |

---

### `bsp_log.c` / `bsp_log.h` — 分级日志系统

| 属性 | 说明 |
|------|------|
| **输出** | USART1 DMA (非阻塞) |
| **日志级别** | `LOG_DEBUG`, `LOG_INFO`, `LOG_WARN`, `LOG_ERROR` |
| **格式** | `[LEVEL] <文件:行号> 函数名(): 消息\r\n` |
| **缓冲区大小** | 256 字节 (可配置) |
| **核心函数** | `LogInit(UART_HandleTypeDef *)` — 绑定日志 USART |
| | `LOG_PROTO(fmt, level, file, line, func, ...)` — 核心格式化+发送函数 |
| **使用宏** | `LOGDEBUG(fmt, ...)` / `LOGINFO(...)` / `LOGWARN(...)` / `LOGERROR(...)` |
| **约束** | ❌ 禁止在 JEOC ISR (电流环) 等高优先级中断中使用 |
| | ✅ 可在 App 层、低优先级中断、RTOS 任务中使用 |

---

### `bsp_flash.c` / `bsp_flash.h` — 内部 Flash 参数持久化

| 属性 | 说明 |
|------|------|
| **硬件** | STM32G431 内部 Flash (页大小 2KB) |
| **功能** | 存储校准参数 (编码器偏移、电机 R/L、PID 增益等), 掉电不丢失 |
| **核心函数** | `FlashWrite(uint32_t addr, uint32_t *data, uint16_t len)` — 写入 Flash (写入前自动擦除) |
| | `FlashRead(uint32_t addr, uint32_t *data, uint16_t len)` — 读取 Flash |
| **注意** | 写入前需先擦除整页 (2KB); 写入操作会暂停 CPU |
| **使用场景** | 校准完成后保存参数; 上电时从 Flash 恢复参数 |

---

## 调用关系

```
App/robot.c:RobotInit()
│
├── BSPInit()                          [bsp_init.h]
│   └── DWT_Init(168)                  [bsp_dwt.c]
│
├── DWT_Delay(0.016f)                  [bsp_dwt.c]  ← 等 MT6816 上电
│
├── LogInit(&huart1)                   [bsp_log.c]
│   └── USARTRegister(&config)         [bsp_usart.c]
│
├── adc_bsp_init()                     [bsp_adc.c]
│   └── ADC1/2 注入组 + TIM1 联动
│
└── (后续模块初始化在 cmd_task / motor_task 中)

App/cmd_task.c:RobotCMDInit()
│
└── FDCANCommInit(&config)              [can_driver.c]
    └── bsp_can_init()                 [bsp_can.c]
        ├── can_filter_init()
        ├── HAL_FDCAN_Start(&hfdcan1)
        └── HAL_FDCAN_ActivateNotification(&hfdcan1, ...)

App/motor_task.c:MotorTask()  ← JEOC ISR, 20kHz
│
├── 直接读取 ADC1->JDRx 寄存器          (绕过 BSP, 最小延迟)
│
└── (BSP 层不参与 20kHz 实时计算)
```

---

## 使用示例

```c
/* === 初始化 (在 RobotInit 中调用) === */
#include "bsp_init.h"
#include "bsp_log.h"
#include "bsp_adc.h"

BSPInit();              // DWT 初始化
LogInit(&huart1);       // 日志绑定 USART1
adc_bsp_init();         // ADC 注入组 + TIM1 联动

/* === 高精度延时 (在任意上下文中) === */
#include "bsp_dwt.h"
DWT_Delay(0.016f);      // 阻塞等待 16ms

/* === 日志输出 (非 ISR 上下文) === */
LOGINFO("系统上电完成, Vbus=%.2fV", vbus);
LOGERROR("过流故障! Iq=%.2fA", iq);

/* === CAN 收发 (通过 can_driver 封装, 不直接调用 BSP) === */
#include "can_driver.h"
FDCANCommSend(ins, &tx_data);
cmd_rx = *(Cmd_Rx_s *)FDCANCommGet(ins);

/* === Flash 存储校准参数 === */
#include "bsp_flash.h"
float params[4] = {Rs, Ls, flux, offset};
FlashWrite(FLASH_SAVE_ADDR, (uint32_t*)params, 4);
```
