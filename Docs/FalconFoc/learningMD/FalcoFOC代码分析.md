

***

# **基于STM32G431的FOC电机驱动板开源工程项目 技术分析报告**

## **0. 项目功能分析**

本项目基于 STM32G431 微控制器，构建了完整的磁场定向控制（FOC）架构。系统功能设计完善，涵盖了从底层驱动到上层轨迹规划的完整闭环控制。

### **0.1 支持的电机控制模式**
该项目实现了完整的FOC控制架构，支持以下控制模式（定义于 `FOCMotor.h:L27-L34`）：

| 控制模式 | 枚举值 | 说明 | 控制精度与响应特性 |
|---------|--------|------|----------------|
| **速度开环** | `CONTROL_MODE_OPEN` | 速度开环控制，直接输出电压 | 无传感器反馈，适用于启动或简单测试 |
| **力矩控制** | `CONTROL_MODE_TORQUE` | 电流/力矩闭环控制 | 高频响应，依赖准确的相电流采样与 Clarke/Park 变换 |
| **速度闭环** | `CONTROL_MODE_VELOCITY` | 速度 PID 闭环控制 | 依赖编码器微分数据，响应平滑 |
| **位置闭环** | `CONTROL_MODE_POSITION` | 位置 PID 闭环控制 | 稳态误差小，适用于云台或机械臂关节 |
| **速度梯形曲线** | `CONTROL_MODE_VELOCITY_RAMP` | 带速度斜率限制的速度控制 | 限制加速度，防止电机瞬态过载失步 |
| **位置梯形曲线** | `CONTROL_MODE_POSITION_RAMP` | 带梯形轨迹的位置控制 | 柔性启停，符合工业点位运动（PTP）规范 |

### **0.2 保护与故障诊断机制**
项目实现了多层级、高实时性的保护机制（`FOCMotor.c:L840-L890`）：

| 保护类型 | 阈值设定 | 故障状态码 |
|---------|------|-----------|
| **过流保护** | `MOTOR_CURRENT_LIMIT` | `FAULT_STATE_OVER_CURRENT` |
| **过压保护** | `BATVEL_MAX_LIMIT` (4.2V×3S×1.2) | `FAULT_STATE_OVER_VOLTAGE` |
| **欠压保护** | `BATVEL_MIN_LIMIT` (4.0V×3S/1.5) | `FAULT_STATE_UNDER_VOLTAGE` |
| **过热保护** | `TEMP_MAX_LIMIT` (65°C) | `FAULT_STATE_OVER_TEMPERATURE` |
| **过速保护** | `SPEED_MAX_LIMIT` | `FAULT_STATE_SPEEDING` |

### **0.3 支持的电机类型及参数范围**
系统支持多极对数无刷直流电机（BLDC/PMSM），需预先标定关键参数（定义于 `BLDCMotor.h:L1-L180`）：

```c
// 支持电机配置示例
#if MOTOR_PM3510
    MOTOR_RS = 1.736~1.747Ω           // 相电阻 (影响死区补偿与前馈)
    MOTOR_LS = 0.00062~0.00065H       // 相电感 (决定电流环带宽)
    MOTOR_VEL_LIMIT = 15.75 turn/s    // 速度限制
    MOTOR_CURRENT_LIMIT = 0.55A       // 电流限制
#elif MOTOR_DJI2312
    MOTOR_RS = 0.135Ω
    MOTOR_LS = 24.6μH
    MOTOR_VEL_LIMIT = 168 turn/s
    MOTOR_CURRENT_LIMIT = 6.0A
#endif
```

---

## **1. 文件架构分析**

工程采用了清晰的四层架构设计，严格隔离了底层硬件与核心算法。

### **1.1 工程文件结构树**

```text
FalconFoc/
├── Core/                        ← ★排除项：STM32CubeMX生成的MCU基本配置 (时钟/引脚)
│   ├── Inc/                     ← HAL库头文件 (gpio.h, tim.h, adc.h, etc.)
│   └── Src/                     ← HAL库源文件 (main.c, gpio.c, tim.c, etc.)
│
├── Drivers/                     ← STM32官方驱动 (CMSIS及STM32G4xx_HAL_Driver)
│
├── BSP/                         ← 板级支持包 (硬件抽象)
│   ├── adc/bsp_adc.c/h          ← ADC底层配置与注入组触发逻辑
│   ├── can/bsp_can.c/h          ← CAN FD总线通信底层封装
│   ├── dwt/bsp_dwt.c/h          ← DWT高精度周期计数器驱动
│   ├── flash/bsp_flash.c/h      ← 内部Flash读写驱动
│   └── usart/bsp_usart.c/h      ← 串口外设抽象
│
├── MODULES/                     ← 功能模块层 ★核心业务代码★
│   ├── algorithm/crc/           ← 基础算法：CRC8/CRC16校验
│   ├── can_comm/can_driver.c/h  ← CAN协议帧打包/解包逻辑
│   ├── controller/pid.c/h       ← 核心控制器：离散位置式PID计算
│   ├── encoder/MT6816/          ← 编码器：MT6816磁编码器读取与PLL解算
│   ├── motor/                   ← ★FOC电机控制核心算法★
│   │   ├── FOCMotor.c/h         ← FOC主控逻辑、状态机与环路嵌套
│   │   ├── BLDCMotor.c/h        ← 数学核心 (Clarke/Park/InvPark/SVPWM)
│   │   ├── Motor_ADC.c/h        ← 物理量映射 (ADC转电流/电压实值)
│   │   └── trapTraj.c/h         ← 运动学核心：梯形轨迹规划
│   └── vofa/vofa.c/h            ← VOFA+ 上位机数据观测器接口
│
└── APP/                         ← 应用层
    ├── cmd_task/cmd_task.c/h    ← 指令调度任务 (解析上位机命令)
    ├── motor_task/motor_task.c/h← 强实时任务 (ADC注入中断回调映射)
    ├── robot.c/h                ← 机器人应用主入口与对象实例化
    └── robot_def.h              ← 核心对象数据结构定义
```

### **1.2 关键数据结构体解析**

系统状态依赖于高度封装的数据结构，实现了面向对象（OOP）的 C 语言设计：

**1. `FOC_DATA` (数学核心参数集)** - `BLDCMotor.h`
记录了 FOC 算法全流程的中间变量。
```c
typedef struct {
    float vbus;           // 母线电压
    float theta;          // 当前电角度 (用于坐标变换)
    float i_a, i_b, i_c;  // 实际相电流 (A)
    float i_alpha, i_beta;// Clarke变换后静止坐标系电流
    float i_d, i_q;       // Park变换后旋转坐标系电流
    float v_d, v_q;       // PID输出的电压指令
    float dtc_a, dtc_b, dtc_c; // 最终PWM占空比 (0.0~1.0)
    float id_set, iq_set; // 电流环目标设定值
} FOC_DATA;
```

**2. `MOTOR_DATA` (电机对象实例)** - `FOCMotor.h`
包含子组件指针及多级闭环控制器。
```c
typedef struct {
    MOTOR_COMPIONENTS components; // 指向编码器、ADC数据的指针集合
    MOTOR_STATE state;            // 当前状态机节点 (空闲/校准/运行/故障)
    PidTypeDef IqPID, IdPID;      // 电流环控制器
    PidTypeDef VelPID, PosPID;    // 速度环、位置环控制器
} MOTOR_DATA;
```

---

## **2. FOC算法实现分析**

算法基于经典磁场定向控制（Field Oriented Control），保证定子磁场与转子磁场始终保持正交（90°），从而实现最大转矩电流比（MTPA）。

### **2.1 核心数学变换与代码映射**

*   **Clarke 变换 (`Clarke()`, `BLDCMotor.c:L138`)**
    将三相静止坐标系（$a, b, c$）降维到两相正交静止坐标系（$\alpha, \beta$）。由于三相电流之和为零（$I_a + I_b + I_c = 0$），公式简化为：
    $i_\alpha = I_a$
    $i_\beta = \frac{I_b - I_c}{\sqrt{3}}$

*   **Park 变换 (`Park()`, `BLDCMotor.c:L155`)**
    利用编码器获取的电角度 $\theta$，将（$\alpha, \beta$）静止坐标系投影到随转子同步旋转的（$d, q$）坐标系：
    $i_d = i_\alpha \cos(\theta) + i_\beta \sin(\theta)$
    $i_q = i_\beta \cos(\theta) - i_\alpha \sin(\theta)$

*   **控制与逆变换 (`Inv_Park()`, `Svpwm_Sector()`)**
    电流环 PID 输出旋转坐标系下的电压指令 $v_d, v_q$ 后，通过**逆 Park 变换**转回静止坐标系 $v_\alpha, v_\beta$。
    最后进入**空间矢量脉宽调制 (SVPWM)**：根据 $v_\alpha, v_\beta$ 所在的六边形扇区，计算出相邻非零矢量及零矢量的作用时间。值得注意的是，在将计算得到的双极性电压指令映射到微控制器定时器的单极性 PWM 寄存器时，工程中在占空比计算环节引入了 **0.5f 的零点偏移量 (Duty Cycle Offset)**，这一处理完美解决了 STM32 高级定时器中心对齐模式下 bipolar 到 unipolar 的信号平移问题。

### **2.2 PI调节器实现逻辑**
系统采用串级控制，底层频率最高：
*   **电流环 (`PID_Calc()` 20kHz)**：位于 `FOCMotor.c` 的 `MotorControlTask` 中，分别对 $i_d$（通常设定为0）和 $i_q$（目标转矩）进行极速闭环，输出电压。
*   **速度环/位置环 (5kHz)**：采用降频处理（每 4 次 PWM 中断执行 1 次）。位置环 PID 计算输出速度设定值（带梯形限幅），速度环 PID 计算输出电流指令（$i_{q\_set}$）。

---

## **3. 功能实现架构图表**

为保证高频控制逻辑的极低延迟，本工程利用了 STM32 特定硬件架构：**DWT 周期计数器**与**ADC 注入组采样**。

### **3.1 核心原理解释补充**
*   **STM32 DWT (Data Watchpoint and Trigger)**：在 RTOS 系统中，基于 SysTick 的系统延时通常精度为 1ms，这对于 FOC 微秒级的算法耗时统计是无效的。工程中引入 `bsp_dwt.c`，利用内核级的 DWT 计数器，实现了纳秒级分辨率的非阻塞延时和算法耗时 Profiling。
*   **ADC 注入组采样 (Injected Conversion)**：FOC 电流采样的核心在于“同步”。工程利用 TIM1 的 CH4(或TRGO) 触发 ADC 转换，且配置为**注入组**。注入组中断优先级极高，能在常规后台 ADC 转换时强行“插队”，确保在 PWM 发波的中心对称点（此时 IGBT 动作引起的开关噪声最小）精准捕获相电流。

### **3.2 电流采样与计算流程 (20kHz 触发)**

```text
┌─────────────────────────────────────────────────────────────────────────────┐
│                         CURRENT SAMPLING FLOW                               │
│                 (严格基于 TIM1 20kHz 硬件触发 + 注入转换)                   │
└─────────────────────────────────────────────────────────────────────────────┘

    ┌───────────────────┐
    │ TIM1 Center Align │◄────── 发波中点产生硬件触发信号 (TRGO)
    │ PWM Period (20kHz)│
    └─────────┬─────────┘
              │
              ▼
    ┌───────────────────┐
    │ ADC1 Injected     │       强制打断常规转换，消除开关噪声干扰
    │ Conversion Start  │
    └─────────┬─────────┘
              │
              ▼
    ┌───────────────────┐
    │ ADC JDR1/2/3 (Ia/b/c)◄───► 滤波处理 (adc1_median_filter)
    │ ADC JDR4 (Vbus)   │
    └─────────┬─────────┘
              │
              ▼
    ┌─────────────────────────────────────────┐
    │     GetMotorADC1PhaseCurrent()          │
    │ ┌─────────────────────────────────────┐ │
    │ │ Ia = (JDR3 - Ia_offset) × FAC       │ │
    │ │ Ib = (JDR2 - Ib_offset) × FAC       │ │
    │ │ Ic = (JDR1 - Ic_offset) × FAC       │ │
    │ └─────────────────────────────────────┘ │
    └────────────────────┬────────────────────┘
                         │
                         ▼
             进入 MotorControlTask() FOC 闭环
```

### **3.3 位置/速度检测流程 (PLL解算)**

```text
┌─────────────────────────────────────────────────────────────────────────────┐
│                     ENCODER & POSITION FLOW                                 │
└─────────────────────────────────────────────────────────────────────────────┘

    ┌───────────────────┐
    │ SPI3 Interface    │◄─── MT6816 Magnetic Encoder (14-bit 绝对值)
    └─────────┬─────────┘
              │
              ▼
    ┌─────────────────────────────────────────┐
    │        GetMotor_Angle() (20kHz)         │
    │ ┌─────────────────────────────────────┐ │
    │ │ 1. 查表补偿: raw - offset_lut[raw]  │ │
    │ │ 2. 软件锁相环 (PLL) 降噪与微分:     │ │
    │ │    pos_est += vel × Ts              │ │
    │ │    vel_est += ki × Δpos             │ │
    │ │ 3. 计算电角度:                      │ │
    │ │    phase = pole_pairs × 2π × pos    │ │
    │ └─────────────────────────────────────┘ │
    └────────────────────┬────────────────────┘
                         │
                         ▼
             输出到位置环与 Park 变换矩阵
```

### **3.4 控制环层级嵌套关系与状态机**

```text
┌─────────────────────────────────────────────────────────────────────────────┐
│                    FOC CONTROL LOOPS & STATE MACHINE                        │
└─────────────────────────────────────────────────────────────────────────────┘

 [System Boot] ──► STATE_MODE_DETECTING (系统自检与标定)
                           │
                           ├── 1. 电压/电流零点偏移校准 (Current_Calibrating)
                           ├── 2. 电机 R/L 参数辨识 (RSLS_Calibrating)
                           └── 3. 编码器线性化查表生成 (Encoder_Calibrating)
                           │
                           ▼
                   STATE_MODE_RUNNING (正常运行态)
                           │
  TIM3 Interrupt (5kHz) ─┼──► 速度环/位置环 (每 4 次 PWM 周期降频执行)
                         │       │
                         │       ├── 位置环 PID_Calc(&PosPID) ──► 输出 vel_set
                         │       └── 速度环 PID_Calc(&VelPID) ──► 输出 iq_set
                         │
  PWM Interrupt (20kHz) ─┼──► 电流环 (极速响应)
                         │       │
                         │       ├── Clarke Transform
                         │       ├── Park Transform (依赖 PLL phase)
                         │       ├── Current PID (计算 v_q, v_d)
                         │       ├── Inv Park Transform
                         │       └── SVPWM (含 0.5f 零点偏移) ──► 写入 TIM1 CCR
                         │
  Daemon Task (5kHz)    ─┼──► MotorGuardTask
                                 (监测 Vbus、温度、失步，触发 FAULT 状态)
```

*(文档结束)*