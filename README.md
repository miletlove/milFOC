# milFOC — 基于 STM32G431 的高性能 FOC 电机驱动器

[![Platform](https://img.shields.io/badge/Platform-STM32G431CBT6-blue)](https://www.st.com/en/microcontrollers-microprocessors/stm32g431.html)
[![Framework](https://img.shields.io/badge/Framework-STM32%20HAL-green)](https://www.st.com/en/embedded-software/stm32cube-mcu-packages.html)
[![Build](https://img.shields.io/badge/Build-CMake%20%2B%20GCC--ARM-orange)](https://developer.arm.com/tools-and-software/open-source-software/developer-tools/gnu-toolchain)
[![License](https://img.shields.io/badge/License-MIT-yellow)](LICENSE)

> **milFOC** 是一款面向高性能 PMSM/BLDC 电机驱动器的开源 FOC 固件项目，基于 **STM32G431CBT6** MCU + **FD6288** 栅极驱动器，采用四层解耦架构设计，支持电流/速度/位置三环级联控制。

---

## 📋 目录

- [硬件平台](#硬件平台)
- [工程架构](#工程架构)
- [层级详解](#层级详解)
- [功能特性](#功能特性)
- [快速开始](#快速开始)
- [完整目录树](#完整目录树)
- [FOC 数据流与时序](#foc-数据流与时序)
- [开发指南](#开发指南)
- [参考项目](#参考项目)
- [许可证](#许可证)

---

## 🔧 硬件平台

| 组件 | 型号/规格 |
|------|-----------|
| **MCU** | STM32G431CBT6 (Arm Cortex-M4F, 170MHz, CORDIC/FMAC) |
| **栅极驱动器** | FD6288 (250V 三相半桥驱动, 内置死区时间) |
| **目标电机** | 5010 750KV BLDC/PMSM |
| **电流采样** | 三电阻下桥采样 (PA0=CUR_A, PA1=CUR_B, PA2=CUR_C) |
| **位置反馈** | MT6816 14-bit 绝对值磁编码器 (SPI1) |
| **通信接口** | USB (CDC), FDCAN, USART1 |
| **母线电压** | 3S LiPo (12.6V max) |

### 引脚映射

```
PWM 输出 (TIM1):
  PA8  → PWMA_H (CH1)    PB13 → PWMA_L (CH1N)
  PA9  → PWMB_H (CH2)    PB14 → PWMB_L (CH2N)
  PA10 → PWMC_H (CH3)    PB15 → PWMC_L (CH3N)

电流采样 (ADC1/2):
  PA0 → CUR_A    PA1 → CUR_B    PA2 → CUR_C    PB1 → VBUS

编码器 (SPI1):
  PA5 → SCK      PA6 → MISO     PA7 → MOSI

通信:
  PA11 → USB_DM    PA12 → USB_DP
  PB8  → FDCAN_RX  PB9  → FDCAN_TX
  PB6  → USART1_TX PB7  → USART1_RX

LED: PC13
```

---

## 🏗️ 工程架构

milFOC 采用**四层解耦架构**，严格隔离硬件抽象、核心算法、应用逻辑：

```
┌──────────────────────────────────────────────────────────────┐
│  🎯 App/          应用层                                      │
│  robot.c       → 系统初始化入口 (RobotInit / RobotTask)       │
│  cmd_task.c    → CAN 命令解析/调度 (~200Hz)                   │
│  motor_task.c  → JEOC 中断回调 (20kHz 电流环)                 │
│  robot_def.h   → CAN ID / 协议帧 / 命令枚举定义               │
├──────────────────────────────────────────────────────────────┤
│  ⚙️ Modules/      功能模块层                                   │
│  motor/          → FOC 算法核心 (Clarke/Park/SVPWM/PID)       │
│  encoder/        → MT6816 编码器 + PLL 速度估算               │
│  controller/     → 抗饱和 PID 控制器                          │
│  comm/           → CAN 协议解析/打包                          │
│  daemon/         → 模块心跳守护                               │
│  led/            → LED 状态指示                               │
│  vofa/           → VOFA+ 数据示波器                           │
│  algorithm/crc   → CRC-8/16 校验                              │
│  general_def.h   → 全局常量/宏/内联函数                       │
├──────────────────────────────────────────────────────────────┤
│  🔌 BSP/          板级支持包                                   │
│  bsp_adc.c     → ADC 注入组 + TIM1 同步触发                  │
│  bsp_can.c     → FDCAN 收发/滤波/中断 (已验证)               │
│  bsp_dwt.c     → DWT 高精度周期计数器 (ns 延时)              │
│  bsp_usart.c   → USART DMA/IT 多实例                         │
│  bsp_log.c     → 非阻塞分级日志                              │
│  bsp_flash.c   → 内部 Flash 参数持久化                       │
│  bsp_init.h    → BSP 统一初始化                               │
├──────────────────────────────────────────────────────────────┤
│  💻 Core/         CubeMX 生成 (禁止手动修改)                   │
│  Core/Inc/      → HAL 头文件 (adc.h, tim.h, gpio.h, ...)     │
│  Core/Src/      → HAL 源文件 (main.c, adc.c, tim.c, ...)     │
│  Drivers/       → CMSIS + HAL 驱动库                          │
│  Middlewares/   → ST USB 协议栈                               │
└──────────────────────────────────────────────────────────────┘
```

### 层级职责与规则

| 层级 | 职责 | 依赖 | 禁止 |
|------|------|------|------|
| **Core/** | CubeMX 生成的 HAL 初始化代码 | CMSIS, HAL | ❌ 手动修改 |
| **BSP/** | 板级外设驱动抽象 | Core HAL | ❌ 包含业务逻辑 |
| **Modules/** | 核心算法与控制模块 | BSP, Core | ❌ 直接操作寄存器 |
| **App/** | 应用层任务调度与系统集成 | Modules, BSP | ❌ 包含算法实现 |

---

## 📊 层级详解

> 各层详细说明请参见对应子目录 README：

| 层级 | README | 说明 |
|------|--------|------|
| BSP | [BSP/README.md](./BSP/README.md) | 每个 BSP 驱动文件的功能、API 接口、硬件映射 |
| Modules | [Modules/README.md](./Modules/README.md) | 各模块的算法原理、数据结构、调用关系 |
| App | [App/README.md](./App/README.md) | 系统初始化流程、任务调度、CAN 协议定义 |

### 层级间的调用关系

```
App/robot.c
├── BSP/bsp_init.h       → DWT 初始化
├── BSP/bsp_log.c        → USART1 日志绑定
├── BSP/bsp_adc.c        → ADC1 注入组配置
├── Modules/motor/       → Foc_Pwm_Start / Foc_Pwm_LowSides
├── Modules/comm/        → FDCANCommInit → BSP/bsp_can.c:bsp_can_init()
├── Modules/led/         → RGB_DisplayColorById
├── App/cmd_task.c       → RobotCMDInit / RobotCMDTask
└── App/motor_task.c     → MotorTask (仅 JEOC ISR 上下文中)

App/cmd_task.c
├── Modules/comm/can_driver.c → FDCANCommGet / FDCANCommSend
│   └── BSP/bsp_can.c         → fdcanx_send_data / rx_data1
├── Modules/motor/bldc_motor.c → Foc_Pwm_LowSides (预充电/急停)
├── Modules/motor/foc_motor.c  → MOTOR_DATA 全局状态
└── Modules/encoder/           → ENCODER_DATA 速度/位置读取

App/motor_task.c (ADC JEOC ISR, 最高优先级, <15µs)
├── Modules/motor/foc_motor.c  → GetMotorADC1PhaseCurrent
├── Modules/motor/bldc_motor.c → Clarke / Park / PID / InvPark / SVPWM / SetPwm
└── Modules/encoder/           → GetMotor_Angle (PLL 更新)
```

---

## ✨ 功能特性

### 控制模式

| 模式 | 枚举值 | 说明 |
|------|--------|------|
| `CONTROL_MODE_OPEN` | 速度开环 | V/F 控制, 用于启动或调试 |
| `CONTROL_MODE_TORQUE` | 力矩闭环 | dq 轴电流 PI 控制 |
| `CONTROL_MODE_VELOCITY` | 速度闭环 | 速度环 → 电流环级联 |
| `CONTROL_MODE_POSITION` | 位置闭环 | 位置环 → 速度环 → 电流环 |
| `CONTROL_MODE_VELOCITY_RAMP` | 速度斜坡 | 带加速度限制的速度控制 |
| `CONTROL_MODE_POSITION_RAMP` | 位置梯形轨迹 | 带梯形轨迹规划的位置控制 |

### 保护机制

| 保护类型 | 触发条件 | 动作 |
|----------|----------|------|
| 过流保护 | Iq > 1.5× 电流限幅 | 关闭 PWM, 进入 GUARD 状态 |
| 过压保护 | Vbus > 1.2× 额定电压 | 关闭 PWM |
| 欠压保护 | Vbus < 额定电压 / 1.5 | 关闭 PWM |
| 过热保护 | NTC > 65°C (预留) | 关闭 PWM |
| FD6288 Fault | 硬件 Break 输入 | 硬件自动关闭 PWM |

### 核心算法

- ✅ Clarke / Park / InvPark / SVPWM 全链路变换
- ✅ 七段式 SVPWM + 中点注入 (bipolar → unipolar)
- ✅ 位置式 PID (抗积分饱和 + 输出限幅)
- ✅ 软件 PLL 编码器速度估算 (2000 rad/s 带宽)
- ✅ 梯形轨迹规划器 (加速 → 匀速 → 减速)
- ✅ 三电阻下桥电流采样 + ADC 注入组同步触发
- ✅ FD6288 自举电容预充电管理
- 🚧 CORDIC 硬件加速 sin/cos (待迁移)
- 🚧 FMAC 硬件滤波器 (待迁移)
- 🚧 电机参数自动辨识 (R/L/极对数/编码器偏移)
- 🚧 无感 FOC (SMO/EKF)
- 🚧 死区补偿

---

## 🚀 快速开始

### 前置条件

- **ARM GCC 工具链**: `arm-none-eabi-gcc` (推荐 12.3+)
- **CMake**: 3.22+
- **Ninja** (推荐) 或 Make
- **STM32CubeMX** 6.x (用于修改 .ioc 配置)
- **VS Code** + STM32 插件 (推荐)

### 构建

```bash
# 克隆仓库
git clone https://github.com/miletlove/milFOC.git
cd milFOC

# 配置 CMake (Debug 模式)
cmake --preset Debug

# 编译
cmake --build build/Debug

# 烧录 (使用 ST-LINK / J-Link)
STM32_Programmer_CLI -c port=SWD -w build/Debug/milFOC.elf -v
```

### CubeMX 配置要点

1. **TIM1**: 中心对齐模式 PWM, 20kHz, 死区时间 1µs, CH4 作为 ADC 触发源
2. **ADC1**: 注入组 4 通道 (JDR1~JDR4), 触发源 TIM1 TRGO
3. **SPI1**: 全双工主模式, 10Mbps, 用于 MT6816
4. **FDCAN1**: Classic CAN 模式, 1Mbps
5. **USART1**: 115200bps, 用于调试日志
6. **USB**: CDC 虚拟串口
7. **CORDIC**: 使能 (用于 sin/cos 硬件加速)

### 中断优先级配置

| 中断 | 优先级 (Preempt/Sub) | 用途 |
|------|---------------------|------|
| ADC1 JEOC | 0/0 (最高) | 20kHz 电流环 FOC 计算 |
| TIM1 Break | 0/1 | FD6288 硬件故障保护 |
| TIM3/TIM2 | 1/0 | 1kHz 速度环调度 |
| USART1 DMA | 3/0 | 日志非阻塞输出 |
| FDCAN1 | 4/0 | CAN 通信 |
| USB CDC | 5/0 | USB 虚拟串口 |

---

## 📁 完整目录树

```
milFOC/
├── .gitignore                   # Git 忽略规则 (build/ 输出, IDE 配置)
├── .github/agents/              # GitHub Copilot Agent 配置
├── CMakeLists.txt               # 顶层 CMake 构建脚本 (自动扫描用户目录)
├── CMakePresets.json            # CMake 预设 (Debug/Release)
├── milFOC.ioc                   # STM32CubeMX 项目配置文件
├── startup_stm32g431xx.s        # 启动汇编 (向量表, 堆栈)
├── STM32G431XX_FLASH.ld         # 链接脚本 (FLASH/RAM 布局)
├── README.md                    # 📖 本文件
│
├── Core/                        # 💻 CubeMX 生成 — 禁止手动修改
│   ├── Inc/                     # HAL 头文件
│   │   ├── main.h               #   主头文件 (引脚定义, 句柄声明)
│   │   ├── adc.h / tim.h / gpio.h
│   │   ├── spi.h / fdcan.h / usart.h
│   │   ├── cordic.h / crc.h / dma.h / rng.h
│   │   ├── stm32g4xx_hal_conf.h #   HAL 模块使能配置
│   │   └── stm32g4xx_it.h       #   中断服务程序声明
│   └── Src/                     # HAL 源文件
│       ├── main.c               #   main() 入口
│       ├── adc.c / tim.c / gpio.c / spi.c / fdcan.c / usart.c
│       ├── cordic.c / crc.c / dma.c / rng.c
│       ├── stm32g4xx_hal_msp.c  #   外设 MSP 初始化
│       ├── stm32g4xx_it.c       #   中断向量表入口
│       ├── system_stm32g4xx.c   #   系统时钟配置
│       ├── syscalls.c / sysmem.c
│
├── Drivers/                     # STM32 官方驱动库
│   ├── CMSIS/
│   │   ├── Include/             #   CMSIS Core (core_cm4.h, cmsis_gcc.h)
│   │   └── Device/ST/STM32G4xx/ #   设备头文件 (stm32g431xx.h, system_)
│   └── STM32G4xx_HAL_Driver/   #   HAL 驱动库 (hal_adc.c, hal_tim.c, ...)
│
├── Middlewares/                 # ST 中间件
│   └── ST/
│       └── STM32_USB_Device_Library/  # USB CDC 协议栈
│
├── USB_Device/                  # USB 应用层 (CubeMX 生成)
│   ├── App/
│   │   ├── usb_device.c         #   USB 设备初始化
│   │   ├── usbd_desc.c          #   设备描述符
│   │   └── usbd_cdc_if.c        #   CDC 接口回调 (接收/发送)
│   └── Target/
│       └── usbd_conf.c          #   USB 配置 (内存, 端点)
│
├── BSP/                         # 🔌 板级支持包 — 详见 BSP/README.md
│   ├── bsp_init.h               # BSP 统一初始化入口
│   ├── bsp_dwt.c / .h           # DWT 周期计数器 (ns 级延时/性能剖析)
│   ├── bsp_adc.c / .h           # ADC 注入组 + TIM1 同步触发
│   ├── bsp_can.c / .h           # FDCAN 收发/滤波/中断 (已验证)
│   ├── bsp_usart.c / .h         # USART DMA/IT 多实例抽象
│   ├── bsp_log.c / .h           # 非阻塞分级日志 (DEBUG/INFO/WARN/ERROR)
│   ├── bsp_flash.c / .h         # 内部 Flash 参数持久化
│   └── README.md                # BSP 层详细文档
│
├── Modules/                     # ⚙️ 功能模块层 — 详见 Modules/README.md
│   ├── general_def.h            # 全局数学常量 / 工具宏 / 数据类型转换
│   ├── controller/
│   │   └── pid.c / .h           # 位置式 PID (抗饱和 + 输出限幅)
│   ├── motor/                   # ★ FOC 核心 ★
│   │   ├── bldc_motor.c / .h    # FOC 数学变换 (Clarke/Park/InvPark/SVPWM)
│   │   ├── foc_motor.c / .h     # 电机状态机 / 多环级联 / 故障保护
│   │   ├── motor_adc.c / .h     # ADC→物理量映射 / 偏置校准
│   │   └── trap_traj.c / .h     # 梯形轨迹规划器
│   ├── encoder/
│   │   └── mt6816_encoder.c/.h  # MT6816 14-bit 绝对值编码器 + PLL
│   ├── comm/
│   │   └── can_driver.c / .h    # CAN 协议层 (命令帧解析 / 遥测打包)
│   ├── daemon/
│   │   └── daemon.c / .h        # 模块心跳守护 (离线检测 + 回调)
│   ├── led/
│   │   └── led.c / .h           # RGB LED 状态指示
│   ├── vofa/
│   │   └── vofa.c / .h          # VOFA+ JustFloat 实时数据流 (USB CDC)
│   ├── algorithm/
│   │   └── crc/
│   │       ├── crc8.c / .h      # CRC-8 校验 (预计算查找表)
│   │       └── crc16.c / .h     # CRC-16 Modbus 校验
│   └── README.md                # Modules 层详细文档
│
├── App/                         # 🎯 应用层 — 详见 App/README.md
│   ├── robot.c / .h             # 系统初始化入口 (RobotInit / RobotTask)
│   ├── robot_def.h              # 核心定义: CAN ID / 协议帧 / 命令枚举
│   ├── cmd_task.c / .h          # CAN 指令解析与调度 (~200Hz)
│   ├── motor_task.c / .h        # ADC JEOC 中断回调 (20kHz 电流环)
│   └── README.md                # App 层详细文档
│
├── cmake/                       # CMake 构建配置
│   ├── gcc-arm-none-eabi.cmake  # ARM GCC 工具链配置
│   ├── starm-clang.cmake        # ARM Clang 工具链 (备选)
│   └── stm32cubemx/
│       └── CMakeLists.txt       # 自动扫描 Core/Drivers/Middlewares 源文件
│
├── Docs/                        # 📚 参考工程与学习资料
│   ├── FalconFoc/               # FalconFoc 参考工程 (本项目架构原型)
│   │   ├── APP/ / BSP/ / MODULES/ / Core/ / Drivers/
│   │   └── learningMD/          #   架构分析 / FOC 流程 / 数据结构文档
│   └── MotorConrol/             # ODrive C++ FOC 移植参考 (不参与编译)
│       ├── foc.cpp / .hpp       #   FieldOrientedController 实现
│       ├── controller.cpp/.hpp  #   位置/速度/力矩级联控制器
│       ├── motor.cpp / .hpp     #   电机参数/校准/电流管理
│       ├── encoder.cpp / .hpp   #   编码器抽象层
│       ├── sensorless_estimator.* # 无感估算器 (SMO/EKF)
│       ├── trapTraj.cpp / .hpp  #   梯形轨迹规划器
│       ├── utils.cpp / .hpp     #   数学工具 (sin/cos/sincos 近似)
│       └── low_level.cpp / .h   #   硬件抽象层
│
└── build/                       # 构建输出 (已 gitignore)
    └── Debug/
        ├── milFOC.elf           #   可执行文件
        ├── milFOC.map           #   内存映射
        ├── compile_commands.json#   编译数据库
        └── CMakeFiles/          #   CMake 构建缓存
```

---

## 🔄 FOC 数据流与时序

### 单次电流环执行流程 (20kHz)

```
TIM1 计数器溢出 (中心对齐模式, 周期 50µs)
        │
        ▼
TIM1 CH4 OC 触发 ADC1 注入组
  → ADC1 同步采样: JDR1=CUR_C, JDR2=CUR_B, JDR3=CUR_A, JDR4=VBUS
        │
        ▼  (~2µs ADC 转换时间)
ADC JEOC 中断 (优先级 0/0, 最高)
  │
  └── motor_task.c:MotorTask()  ─────── 总执行时间 < 15µs ───────
        │
        ├─ 1. GetMotorADC1PhaseCurrent()
        │     读取 ADC1->JDR3/2/1 → Ia/Ib/Ic [A]
        │     读取 ADC1->JDR4 → Vbus [V]
        │
        ├─ 2. Clarke(Ia,Ib,Ic) → Iα, Iβ
        │     Iα = Ia
        │     Iβ = (Ib - Ic) / √3
        │
        ├─ 3. GetMotor_Angle() → θ_e, ω_m
        │     SPI 读 MT6816 14-bit 角度 → LUT 非线性校正
        │     → PLL 跟踪 → pos_estimate_, vel_estimate_, phase_
        │
        ├─ 4. Park(Iα,Iβ, θ_e) → Id, Iq
        │     Id = Iα·cosθ + Iβ·sinθ
        │     Iq = -Iα·sinθ + Iβ·cosθ
        │
        ├─ 5. PID(Id_ref-Id) → Vd
        │     PID(Iq_ref-Iq) → Vq
        │     (带积分抗饱和 + 输出限幅)
        │
        ├─ 6. InvPark(Vd,Vq, θ_e) → Vα, Vβ
        │     Vα = Vd·cosθ - Vq·sinθ
        │     Vβ = Vd·sinθ + Vq·cosθ
        │
        ├─ 7. Svpwm_Midpoint(Vα,Vβ) → dtc_a, dtc_b, dtc_c
        │     扇区判定 → 矢量时间 → 三相占空比 [0~1] + 0.5 中点偏置
        │
        └─ 8. SetPwm(dtc_a,b,c) → TIM1->CCR1/2/3
              直接写寄存器 (最小延迟)
```

### 多环级联分频

```
每 1 次 ADC JEOC    → 电流环 (20kHz)
每 20 次 ADC JEOC   → 速度环 (1kHz)   — VelPID 更新 Iq_setpoint
每 200 次 ADC JEOC  → 位置环 (100Hz)  — PosPID 更新 Vel_setpoint + 梯形轨迹步进
```

### 电机状态机

```
IDLE ──(校准完成或跳过)──→ DETECTING
                              │
                     ┌───────[自动校准]────────┐
                     │  CURRENT_CALIBRATING     │  ADC 偏置采集
                     │  → RSLS_CALIBRATING      │  电阻/电感辨识
                     │  → FLUX_CALIBRATING      │  磁链/极对数辨识
                     │  → ENCODER_CALIBRATING   │  编码器偏移校准
                     │  → REPORT_OFFSET_LUT     │  非线性 LUT 生成
                     └──────────┬───────────────┘
                                ▼
                             RUNNING ◄──────── 正常 FOC 运行
                                │
                     ┌─────────┴─────────┐
                     ▼                   ▼
              过流/过压/欠压/过热     用户 STOP 指令
                     │                   │
                     └────────┬──────────┘
                              ▼
                            GUARD  ────  PWM 关闭, 等待清除
```

---

## 👨‍💻 开发指南

### 分支策略

- `main` — 稳定分支，必须编译通过
- `dev/*` — 功能开发分支
- `test/*` — 板级测试分支

> ⚠️ 请在非 main 分支上进行修改和测试，验证通过后再合并！

### 编码规范

1. **BSP 层** — 仅封装硬件操作，函数名以 `bsp_` 或硬件名开头
2. **Modules 层** — 实现纯算法和控制逻辑，通过 BSP 接口访问硬件
3. **Core 层** — CubeMX 生成，**禁止手动修改**
4. **中断函数** — 仅放必要逻辑，保持 < 15µs 执行时间
5. **日志使用** — ISR 中禁止使用 `LOG_*` 宏；使用 `LOG_PROTO` 格式化

### Git 提交规范

```
<type>: <subject>

<body> — 变更原因、影响范围

类型: feat(功能) / fix(修复) / refactor(重构) / docs(文档) / chore(杂项)
示例: fix: 修复 bsp_can.c 中 can_filter_init 隐式声明警告
```

---

## 📚 参考项目

| 项目 | 路径 | 用途 |
|------|------|------|
| **FalconFoc** | `Docs/FalconFoc/` | 本项目架构原型 (基于 STM32G431 的完整 FOC 工程) |
| **ODrive** | `Docs/MotorConrol/` | C++ FOC 移植参考 (算法分析与架构借鉴, 不参与编译) |

---

## 📄 许可证

本项目采用 MIT 许可证。详见 [LICENSE](./LICENSE) 文件。
