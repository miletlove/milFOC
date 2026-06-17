# Modules — 功能模块层

> **职责**: 实现纯算法和控制逻辑，包含 FOC 全链路变换、PID 控制、编码器估算、CAN 协议解析等。该层通过 BSP 接口访问硬件，不直接操作寄存器。

---

## 📋 目录

- [架构概述](#架构概述)
- [子模块详解](#子模块详解)
  - [general_def.h — 全局通用定义](#general_defh--全局通用定义)
  - [motor/ — FOC 电机控制核心](#motor--foc-电机控制核心)
  - [encoder/ — MT6816 编码器 + PLL](#encoder--mt6816-编码器--pll)
  - [controller/ — PID 控制器](#controller--pid-控制器)
  - [comm/ — CAN 协议驱动](#comm--can-协议驱动)
  - [trap_traj/ (位于 motor/) — 梯形轨迹规划器](#motor--梯形轨迹规划器)
  - [daemon/ — 模块心跳守护](#daemon--模块心跳守护)
  - [led/ — RGB LED 状态指示](#led--rgb-led-状态指示)
  - [vofa/ — VOFA+ 数据示波器](#vofa--vofa-数据示波器)
  - [algorithm/crc/ — CRC 校验](#algorithmcrc--crc-校验)
- [模块间调用关系](#模块间调用关系)

---

## 架构概述

```
Modules/
├── general_def.h          # ★ 全局定义 (数学常量 / 宏 / 内联函数)
├── motor/                 # ★ FOC 核心算法
│   ├── bldc_motor.c/.h    #   数学变换 (Clarke/Park/InvPark/SVPWM)
│   ├── foc_motor.c/.h     #   状态机 / 多环级联调度 / 故障保护
│   ├── motor_adc.c/.h     #   ADC→物理量映射 / 偏置校准
│   └── trap_traj.c/.h     #   梯形轨迹规划器
├── encoder/
│   └── mt6816_encoder.c/.h#   MT6816 编码器 + PLL 速度估算
├── controller/
│   └── pid.c/.h           #   位置式 PID (抗积分饱和)
├── comm/
│   └── can_driver.c/.h    #   CAN 协议层 (命令帧解析 / 遥测帧打包)
├── daemon/
│   └── daemon.c/.h        #   模块心跳守护 (离线检测)
├── led/
│   └── led.c/.h           #   RGB LED 状态指示
├── vofa/
│   └── vofa.c/.h          #   VOFA+ JustFloat 实时数据流
├── algorithm/
│   └── crc/
│       ├── crc8.c/.h      #   CRC-8 校验
│       └── crc16.c/.h     #   CRC-16 Modbus 校验
└── README.md              #   本文件
```

---

## 子模块详解

### `general_def.h` — 全局通用定义

| 属性 | 说明 |
|------|------|
| **路径** | `Modules/general_def.h` |
| **依赖** | `<math.h>`, `<stdint.h>`, `main.h` |
| **功能分类** | |

#### 数学常量
| 宏 | 值 | 用途 |
|---|---|---|
| `M_PI` | 3.14159265f | 圆周率 (带 `#ifndef` 保护) |
| `M_2PI` | 6.28318530f | 2π |
| `_SQRT3` | 1.73205080f | √3 |
| `ONE_BY_SQRT3` | 0.57735026f | 1/√3 (Park/Clarke 系数) |
| `TWO_BY_SQRT3` | 1.15470053f | 2/√3 (SVPWM 系数) |

#### 单位转换宏
| 宏 | 公式 | 用途 |
|---|---|---|
| `DEG2RAD_f(d)` | d × π/180 | 角度→弧度 |
| `RPM2RADPS_f(r)` | r × 2π/60 | 转速→角速度 |
| `RADPS2RPM_f(r)` | r × 60/(2π) | 角速度→转速 |

#### FOC 核心变换函数 (static inline)

| 函数 | 输入 | 输出 | 实现 |
|---|---|---|---|
| `clarke_transform()` | Ia, Ib, Ic | Iα, Iβ | Iα=Ia; Iβ=(Ib-Ic)/√3 |
| `park_transform()` | Iα, Iβ, θ | Id, Iq | 使用 `fast_sincos` |
| `inverse_park()` | Vd, Vq, θ | Vα, Vβ | 反 Park 变换 |
| `svm()` | Vα, Vβ | tA, tB, tC | 七段式 SVPWM (6扇区 switch/case) |
| `fast_sincos()` | θ | sinθ, cosθ | 抛物线拟合 + 修正 (误差 < 0.001) |

#### 角度数学函数 (static inline)

| 函数 | 功能 |
|---|---|
| `wrap_pm(x, range)` | 角度折叠到 [-range, range] |
| `wrap_pm_pi(theta)` | 角度折叠到 [-π, π] |
| `fmodf_pos(x, y)` | 保证结果非负的浮点取模 |
| `mod(dividend, divisor)` | 支持负数的整数取模 |

#### 数据类型转换 (for CAN/serial protocols)

| 函数 | 功能 |
|---|---|
| `int_to_data()` / `data_to_int()` | int ↔ 4字节 (小端序) |
| `float_to_data()` / `data_to_float()` | float ↔ 4字节 (小端序) |
| `float4_to_data7()` | float → data[4~7] (偏移写入) |

#### 滤波器宏

| 宏 | 说明 |
|---|---|
| `UTILS_LP_FAST(v,s,k)` | 一阶低通滤波: v -= k*(v-s) |
| `UTILS_LP_MOVING_AVG_APPROX(v,s,N)` | 滑动平均近似: k=2/(N+1) |

---

### `motor/` — FOC 电机控制核心

#### `bldc_motor.c` / `bldc_motor.h` — FOC 数学变换

| 属性 | 说明 |
|------|------|
| **类型** | 纯数学模块, 无硬件依赖 |
| **核心结构** | `FOC_DATA` — 单个 PWM 周期的完整中间计算数据 |
| **核心函数** | `Clarke(FOC_DATA *)` — 三相电流 → α-β 静止坐标系 |
| | `Park(FOC_DATA *)` — α-β 静止 → d-q 旋转坐标系 |
| | `Inv_Park(FOC_DATA *)` — d-q 旋转 → α-β 静止 (电压) |
| | `Inv_Clarke(FOC_DATA *)` — α-β 静止 → 三相电压 |
| | `Svpwm_Midpoint(FOC_DATA *)` — α-β 电压 → 三相 PWM 占空比 (七段式 + 中点注入) |
| | `Sin_Cos_Val(FOC_DATA *)` — 计算 sinθ/cosθ (当前用 `sinf`/`cosf`) |
| **PWM 控制** | `Foc_Pwm_Start()` — 启动所有 PWM 通道 (50% 安全态) |
| | `Foc_Pwm_Stop()` — 停止所有 PWM |
| | `Foc_Pwm_LowSides()` — 下桥全导通 (FD6288 自举电容预充电) |
| | `SetPwm(FOC_DATA *)` — 直接写 TIM1->CCR1/2/3 |
| **电机参数** | 通过 `bldc_motor.h` 中的宏配置: `MPTOR_P` (极对数), `MOTOR_RS` (相电阻), `MOTOR_LS` (相电感), `PWM_FREQUENCY` (20kHz) 等 |
| **待优化** | 将 `sinf`/`cosf` 迁移至 CORDIC 硬件加速器 |

#### `foc_motor.c` / `foc_motor.h` — 电机状态机与级联控制

| 属性 | 说明 |
|------|------|
| **核心结构** | `MOTOR_DATA` — 顶层电机数据结构 (聚合 Components/State/Parameters/Controller/PID) |
| **全局实例** | `motor_data` (单例) |
| **状态机** | IDLE → DETECTING (自动校准) → RUNNING → GUARD (故障) |
| **控制模式** | OPEN / TORQUE / VELOCITY / POSITION / VELOCITY_RAMP / POSITION_RAMP |
| **核心函数** | `Init_Motor_Calib(MOTOR_DATA *)` — 全自动校准入口 |
| | `Init_Motor_No_Calib(MOTOR_DATA *)` — 使用预校准参数初始化 |
| | `GetMotorADC1PhaseCurrent(MOTOR_DATA *)` — 从 ADC 注入组读取三相电流+母线电压 |
| | `FOC_update_current_gain(MOTOR_DATA *)` — 根据电机参数自动计算 PI 增益 |
| **PID 实例** | `IqPID` (Q轴电流 PI), `IdPID` (D轴电流 PI), `VelPID` (速度环 PI), `PosPID` (位置环 PID) |
| **故障保护** | 过流 / 过压 / 欠压 / 过热 / 超速 — 触发后进入 GUARD 状态并关闭 PWM |

#### `motor_adc.c` / `motor_adc.h` — ADC 物理量映射

| 属性 | 说明 |
|------|------|
| **核心结构** | `CURRENT_DATA` — ADC 偏置 + 原始数据 |
| **全局实例** | `current_data` (单例) |
| **通道映射** | JDR3=CUR_A(PA0), JDR2=CUR_B(PA1), JDR1=CUR_C(PA2), JDR4=VBUS(PB1) |
| **核心函数** | `GetTempNtc(uint16_t adc_value, float *temp)` — NTC 热电偶转换 (预留) |

#### `motor/trap_traj.c` / `trap_traj.h` — 梯形轨迹规划器

| 属性 | 说明 |
|------|------|
| **功能** | 生成三阶段速度梯形: 加速 → 匀速 → 减速 |
| **核心结构** | `TRAPEZOIDAL` — 轨迹状态 (位置/速度/加速度/时间/阶段) |
| **核心函数** | `traj_plan(TRAPEZOIDAL *, float start, float end, float max_vel, float accel, float decel)` — 规划新轨迹 |
| | `traj_update(TRAPEZOIDAL *)` — 步进一个周期, 输出当前期望 pos/vel/acc |
| **用于** | `CONTROL_MODE_POSITION_RAMP` 模式下的工业 PTP 运动 |

---

### `encoder/` — MT6816 编码器 + PLL

#### `mt6816_encoder.c` / `mt6816_encoder.h`

| 属性 | 说明 |
|------|------|
| **传感器** | MT6816 14-bit 绝对值磁编码器, 16384 CPR |
| **接口** | SPI1 (PA5=SCK, PA6=MISO, PA7=MOSI) |
| **核心结构** | `ENCODER_DATA` — 编码器全部状态 (原始角/校正角/位置/速度/电角度) |
| **全局实例** | `encoder_data` (单例) |
| **核心函数** | `GetMotor_Angle(ENCODER_DATA *)` — 每次调用执行: SPI 读角度 → 非线性 LUT 校正 → PLL 跟踪 → 输出 pos/vel/phase |
| | `Theta_ADD(ENCODER_DATA *)` — 开环角度累加器 |
| | `low_pass_filter(float)` — 编码器读数低通滤波 |
| | `normalize_angle(float)` — 角度归一化到 [0, 2π) |
| **PLL** | 软件锁相环 (PLL): `pll_kp = 2 * bw`, `pll_ki = 0.25 * kp²`, 默认带宽 2000 rad/s |
| **关键输出** | `phase_` — 电角度 [rad] (Park/InvPark 直接使用); `vel_estimate_` — 速度 [turn/s]; `pos_estimate_` — 位置 [turn] |
| **非线性校正** | 128 点查找表 (LUT), 校准后自动补偿传感器非线性误差 |

---

### `controller/` — PID 控制器

#### `pid.c` / `pid.h`

| 属性 | 说明 |
|------|------|
| **类型** | 位置式 PID (离散域) |
| **核心结构** | `PidTypeDef` — PID 参数 + 积分状态 + 输出限幅 |
| **核心函数** | `PID_Init(PidTypeDef *, ...)` — 初始化 PID (Kp/Ki/Kd + 限幅值) |
| | `PID_Calculate(PidTypeDef *, float ref, float fdb)` — 一次 PID 计算, 返回控制量 |
| **抗饱和** | 积分钳位 (clamping): 输出超限时停止积分累加 |
| **微分处理** | "测量值微分" (derivative on measurement): 避免设定值阶跃时的微分冲击 |
| **使用实例** | `motor_data.IqPID` / `IdPID` — 20kHz 电流环 |
| | `motor_data.VelPID` — 1kHz 速度环 |
| | `motor_data.PosPID` — 100Hz 位置环 |

---

### `comm/` — CAN 协议驱动

#### `can_driver.c` / `can_driver.h`

| 属性 | 说明 |
|------|------|
| **核心结构** | `FDCANCommInstance` — CAN 通信实例 (句柄/TX ID/RX ID/数据长度) |
| **核心函数** | `FDCANCommInit(FDCANComm_Init_Config_s *)` — 初始化 CAN 实例 (调用 `bsp_can_init`) |
| | `FDCANCommGet(FDCANCommInstance *)` — 获取接收数据 (指向 BSP `rx_data1`) |
| | `FDCANCommSend(FDCANCommInstance *, void *)` — 发送数据 (调用 BSP `fdcanx_send_data`) |
| **协议帧** | CMD 帧: `[cmd_id:4B][data:4B]`; TELEM 帧: `[state:4B][vel:2B][pos:2B]` (定义在 `robot_def.h`) |
| **上层调用** | `App/cmd_task.c` |

---

### `daemon/` — 模块心跳守护

#### `daemon.c` / `daemon.h`

| 属性 | 说明 |
|------|------|
| **功能** | 监控各模块的健康状态, 每个模块注册一个守护实例 (带重载计数) |
| **核心函数** | `DaemonRegist(void *, uint32_t, DaemonCallback)` — 注册模块 (id, 超时计数, 回调) |
| | `DaemonReload(void *)` — 重载计数器 (模块调用表示存活) |
| | `DaemonTask(void)` — 守护任务: 递减所有计数器, 归零则触发回调 |
| **使用场景** | 检测编码器离线、CAN 通信超时、PWM 输出异常等 |

---

### `led/` — RGB LED 状态指示

#### `led.c` / `led.h`

| 属性 | 说明 |
|------|------|
| **硬件** | PC13, WS2812B 兼容 RGB LED |
| **核心函数** | `LED_Init(void)` — LED 初始化 |
| | `RGB_DisplayColorById(int color_id)` — 按颜色 ID 设置 LED |
| | `RGB_DisplayColor(uint8_t R, uint8_t G, uint8_t B)` — 设置指定颜色 |
| **颜色约定** | 0=红色(故障), 1=绿色(运行), 2=蓝色(校准中), 3=黄色(空闲) |
| **待完成** | TIM PWM+DMA 驱动 WS2812B 精确时序 |

---

### `vofa/` — VOFA+ 数据示波器

#### `vofa.c` / `vofa.h`

| 属性 | 说明 |
|------|------|
| **功能** | 通过 USB CDC 向上位机 VOFA+ 发送实时 FOC 变量, 用于调参和波形观察 |
| **协议** | JustFloat: 每个浮点数 4 字节 (小端序), 帧尾 `0x00 0x00 0x80 0x7F` |
| **核心函数** | `vofa_send_data(uint8_t ch, float data)` — 发送单个通道数据 |
| | `vofa_sendframetail(void)` — 发送帧尾 |
| | `Vofa_Packet(void)` — 打包发送所有 7 个 FOC 通道 |
| **数据通道** | CH0=i_d, CH1=i_q, CH2=v_d, CH3=v_q, CH4=theta, CH5=velocity, CH6=vbus |
| **每帧大小** | 7×4B + 4B tail = 32 字节 |
| **待完成** | USB CDC 发送函数的底层实现 |

---

### `algorithm/crc/` — CRC 校验

#### `crc8.c` / `crc8.h`

| 属性 | 说明 |
|------|------|
| **功能** | CRC-8 校验 (多项式 0x07), 使用预计算查找表 (256 字节) |
| **核心函数** | `crc8_calc(uint8_t *data, uint32_t len)` — 计算 CRC-8 |
| **使用场景** | 编码器通信校验、小数据包完整性检查 |

#### `crc16.c` / `crc16.h`

| 属性 | 说明 |
|------|------|
| **功能** | CRC-16 Modbus 校验 (多项式 0x8005), 使用预计算查找表 (512 字节) |
| **核心函数** | `crc16_calc(uint8_t *data, uint32_t len)` — 计算 CRC-16 |
| **使用场景** | Modbus RTU 通信、CAN 帧数据完整性校验 |

---

## 模块间调用关系

```
App 层调用
├── cmd_task.c ─────→ comm/can_driver.c ──→ BSP/bsp_can.c
├── motor_task.c ───→ motor/foc_motor.c ──→ motor/bldc_motor.c
│                                  └──────→ encoder/mt6816_encoder.c
│                                  └──────→ motor/motor_adc.c
│                                  └──────→ controller/pid.c
│                                  └──────→ motor/trap_traj.c
│
├── robot.c ────────→ motor/bldc_motor.c  (Foc_Pwm_Start/LowSides)
│                  └─→ led/led.c          (RGB_DisplayColorById)
│                  └─→ comm/can_driver.c
│                  └─→ motor/motor_adc.c
│
└── App 层独立任务
    ├── daemon/daemon.c  (心跳检测, 独立调度)
    └── vofa/vofa.c      (数据示波器, 按需调用)

跨层调用 (通过 general_def.h 共享)
    ALL ───────→ general_def.h  (数学常量 / FOC 变换 / 数据转换)
```
