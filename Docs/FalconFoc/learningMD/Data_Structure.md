## 项目电机控制数据结构及初始化流程分析

本项目是一个基于 **STM32G431** 的 **FOC（磁场定向控制）** 无刷直流电机驱动固件，采用了串级控制架构：**位置环 → 速度环 → 电流环**。下面我从数据结构定义、全局实例化、初始化流程三个层面进行系统分析。

---

### 一、核心数据结构全景图

整个电机控制系统的数据结构分为 **6 层**，顶层统一由 `MOTOR_DATA` 聚合：

```
MOTOR_DATA (顶层电机对象)
├── components (MOTOR_COMPIONENTS)  →  硬件组件指针
│   ├── foc     → FOC_DATA          →  FOC算法中间变量
│   ├── encoder → ENCODER_DATA      →  磁编码器数据
│   └── current → CURRENT_DATA      →  ADC电流采样数据
├── state (MOTOR_STATE)             →  状态机
├── parameters (MOTOR_PARAMETERS)   →  电机物理参数(Rs, Ls, flux)
├── Controller (MOTOR_CONTROLLER)   →  控制参数与设定值
├── IqPID (PidTypeDef)              →  Q轴电流PI控制器
├── IdPID (PidTypeDef)              →  D轴电流PI控制器
├── VelPID (PidTypeDef)             →  速度环PI控制器
└── PosPID (PidTypeDef)             →  位置环PI控制器
```

---

### 二、各数据结构详细分析

#### 1. `FOC_DATA` — FOC 核心计算数据（BLDCMotor.h）

这是 FOC 算法的"工作台"，存储每一帧 PWM 周期中所有坐标变换和 PI 计算的中间量：

| 字段 | 类型 | 物理意义 |
|------|------|----------|
| `vbus` | `float` | 实测母线电压 (V) |
| `inv_vbus` | `float` | 母线电压倒数，用于电压归一化 |
| `theta` | `float` | 当前电角度 (rad) |
| `i_a, i_b, i_c` | `float` | **三相采样电流** (A)，ADC注入转换后赋值的入口 |
| `i_alpha, i_beta` | `float` | Clarke 变换后的 αβ 轴电流 |
| `i_d, i_q` | `float` | Park 变换后的 dq 轴电流（直流分量） |
| `v_alpha, v_beta` | `float` | 反Park变换后的 αβ 轴电压 |
| `v_a, v_b, v_c` | `float` | 反Clarke变换后的三相电压 |
| `v_d, v_q` | `float` | **电流环PI输出的 dq 轴电压** |
| `vd_set, vq_set` | `float` | dq 轴电压设定值 |
| `id_set, iq_set` | `float` | **dq 轴电流设定值**（速度环/PI输出） |
| `dtc_a, dtc_b, dtc_c` | `float` | 三相 PWM 占空比 [0,1] |
| `sin_val, cos_val` | `float` | 当前电角度的三角函数值 |
| `sector` | `int` | SVPWM 当前扇区号(1~6) |
| `i_q_filt, i_d_filt` | `float` | 低通滤波后的 dq 电流 |
| `current_ctrl_integral_d/q` | `float` | 电流环积分累加器（用于抗饱和） |
| `current_ctrl_p_gain/i_gain` | `float` | 电流环PI增益（可自动计算） |

**FOC 数据流**（一帧20kHz周期内）：

```
ADC注入采样 → i_a/i_b/i_c
    ↓
Clarke变换 → i_alpha/i_beta
    ↓
Park变换   → i_d/i_q  (结合 theta 和 sin/cos)
    ↓
电流环PI   → v_d/v_q  (目标 id_set/iq_set vs 实测 i_d/i_q)
    ↓
反Park变换  → v_alpha/v_beta
    ↓
SVPWM      → dtc_a/dtc_b/dtc_c
    ↓
写入TIM1 CCR → PWM输出
```

---

#### 2. `ENCODER_DATA` — MT6816 磁编码器数据（mt6816_encoder.h）

| 字段 | 类型 | 物理意义 |
|------|------|----------|
| `hspi` | `SPI_HandleTypeDef*` | SPI 句柄 |
| `angle` | `uint32_t` | 从 MT6816 读取的原始角度值(14位) |
| `raw` | `int` | 方向修正后的原始计数值 [0, 16383] |
| `cnt` | `int` | **非线性校正后的编码器计数值** |
| `shadow_count_` | `int` | 多圈累计计数值 |
| `count_in_cpr_` | `int` | 单圈内计数值 [0, 16383] |
| `pos_estimate_` | `float` | **PLL估算的转子位置 (圈数)** |
| `vel_estimate_` | `float` | **PLL估算的转子速度 (圈/秒, turn/s)** |
| `pos_cpr_` | `float` | PLL估算的单圈位置 [0, 1) |
| `phase_` | `float` | **电角度 (rad)**，范围 [-π, π] |
| `elec_angle` | `float` | 电角度 [0, 2π) |
| `mec_angle` | `float` | 机械角度 [0, 2π) |
| `pole_pairs` | `uint8_t` | 电机极对数 |
| `dir` | `int` | 旋转方向 (CW=1 / CCW=-1) |
| `encoder_offset` | `float` | **编码器零位偏移**，校准获得 |
| `offset_lut[128]` | `int32_t` | **非线性校正查找表**，128点 FIR滤波结果 |
| `interpolation_` | `float` | 编码器计数间插值系数 [0,1] |
| `calib_valid` | `int` | 校准有效标志 |
| `theta_acc` | `float` | 开环控制时的电角度累加步长 |

编码器内部使用了 **PLL（锁相环）** 对原始计数值进行滤波和速度估算，`pll_kp_` 和 `pll_ki_` 由 `ENCODER_PLL_BANDWIDTH(2000 rad/s)` 计算：

```c
pll_kp_ = 2.0f * ENCODER_PLL_BANDWIDTH;  // 4000
pll_ki_ = 0.25f * SQ(pll_kp_);            // 4e6
```

---

#### 3. `CURRENT_DATA` — ADC 电流采样数据（Motor_ADC.h）

| 字段 | 类型 | 物理意义 |
|------|------|----------|
| `hadc` | `ADC_HandleTypeDef*` | ADC 句柄 (hadc1) |
| `Temp_Result` | `float` | NTC 温度采样结果 (°C) |
| `Ia_offset/b/c` | `float` | **三相电流偏置**（校准时获取） |
| `current_offset_sum_a/b/c` | `float` | 偏置校准累加器 |

电流采样使用 **ADC1 注入模式（Injected Mode）**，由 TIM1 触发同步采样，采样完成后进入中断回调 `HAL_ADCEx_InjectedConvCpltCallback`。

---

#### 4. `MOTOR_PARAMETERS` — 电机物理参数

| 字段 | 物理意义 |
|------|----------|
| `Rs` | 相电阻 (Ω)，可通过 RSLS 校准自动获取 |
| `Ls` | 相电感 (H)，可通过 RSLS 校准自动获取 |
| `flux` | 永磁体磁链 (Wb) |

---

#### 5. `MOTOR_CONTROLLER` — 控制参数与设定值

| 字段 | 物理意义 |
|------|----------|
| `inertia` | 转动惯量 [A/(turn/s²)]，用于前馈补偿 |
| `vel_ramp_rate` | 速度斜坡速率 [(turn/s)/s] |
| `traj_vel/accel/decel` | 梯形轨迹规划的速度、加速度、减速度 |
| `torque_const` | 转矩常数 [Nm/A] |
| `torque_limit` | 转矩限制 [Nm] |
| `current_limit` | 电流限制 [A] |
| `voltage_limit` | 电压限制 [V] |
| `current_ctrl_bandwidth` | 电流环带宽 [rad/s] (默认230) |
| `input_position/velocity/torque/current` | 用户设定的目标值 |
| `pos_setpoint/vel_setpoint/torque_setpoint` | 经斜坡/轨迹处理后的设定值 |
| `input_updated` | 位置更新标志，触发梯形轨迹重规划 |

---

#### 6. `MOTOR_STATE` — 状态机

**状态模式** (`STATE_MODE`)：
| 状态 | 含义 |
|------|------|
| `IDLE` | 空闲，进入后自动切到 DETECTING |
| `DETECTING` | 检测/校准中（电流偏置校准 → RSLS校准） |
| `RUNNING` | 正常运行，执行 FOC 控制 |
| `GUARD` | 保护模式，关闭 PWM |

**控制模式** (`CONTROL_MODE`)：
| 模式 | 控制链路 |
|------|----------|
| `OPEN` | 开环电压控制（V/F） |
| `TORQUE` | 电流闭环（仅 Iq/Id PI） |
| `VELOCITY` | 速度环 → 电流环 |
| `POSITION` | 位置环 → 速度环 → 电流环 |
| `VELOCITY_RAMP` | 带速度斜坡的速度环 |
| `POSITION_RAMP` | 带梯形轨迹的位置环 |

**子状态** (`SUB_STATE`)：
- `CURRENT_CALIBRATING`：电流偏置校准
- `RSLS_CALIBRATING`：电阻/电感/方向/极对数/编码器校准

---

#### 7. `PidTypeDef` — PID 控制器（pid.h）

| 字段 | 物理意义 |
|------|----------|
| `mode` | 位置式(`PID_POSITION`) 或 增量式(`PID_DELTA`) |
| `Kp, Ki, Kd` | PID 增益 |
| `max_out` | 输出限幅 |
| `max_iout` | 积分项输出限幅（抗积分饱和） |
| `set, fdb` | 设定值、反馈值 |
| `out, Pout, Iout, Dout` | 总输出、比例项、积分项、微分项 |
| `Dbuf[3]` | 微分缓冲（当前/上次/上上次） |
| `error[3]` | 误差缓冲 |

本项目4个 PID 实例均使用**位置式 PID**模式。

---

#### 8. `tTraj` — 梯形轨迹规划器（trapTraj.h）

| 字段 | 物理意义 |
|------|----------|
| `Y` | 当前位置 (turn) |
| `Yd` | 当前速度 (turn/s) |
| `Ydd` | 当前加速度 (turn/s²) |
| `Tf_` | 总运动时间 (s) |
| `t` | 当前时间戳 |
| `trajectory_done` | 轨迹完成标志 |

---

### 三、全局变量初始化

所有核心数据结构均通过 **C99 指定初始化器** 在编译时静态初始化，无需运行时调用 init 函数：

#### `FOC_DATA foc_data`（`BLDCMotor.c`）

```c
FOC_DATA foc_data = {
    .inv_vbus                = INVBATVEL,                 // 1/(4*3.0) ≈ 0.0833
    .vd_set = .vq_set        = 0.0f,
    .id_set                  = 0.0f,
    .iq_set                  = MOTOR_INPUT_CURRENT,       // 0.4A (PM3510)
    .current_ctrl_integral_d/q = 0.0f,
    .current_ctrl_p_gain     = MOTOR_CURRENT_CTRL_P_GAIN, // 0 (Auto)
    .current_ctrl_i_gain     = MOTOR_CURRENT_CTRL_I_GAIN, // 0 (Auto)
};
```

#### `ENCODER_DATA encoder_data`（mt6816_encoder.c）

```c
ENCODER_DATA encoder_data = {
    .hspi            = &hspi3,
    .pole_pairs      = MPTOR_P,         // 11 (PM3510)
    .encoder_offset  = MOTOR_OFFSET,    // 8001 (rb电机)
    .dir             = MOTOR_DIRECTION, // CCW
    .calib_valid     = false,
    .theta_acc       = 0.01f,
};
```

#### `CURRENT_DATA current_data`（Motor_ADC.c）

```c
CURRENT_DATA current_data = {
    .hadc = &hadc1,
    .Ia_offset = .Ib_offset = .Ic_offset = 0.0f,
    .current_offset_sum_a/b/c = 0.0f,
};
```

#### `MOTOR_DATA motor_data`（FOCMotor.c）— **顶层聚合对象**

```c
MOTOR_DATA motor_data = {
    .components = {
        .foc     = &foc_data,        // 指针链接
        .encoder = &encoder_data,
        .current = &current_data,
    },
    .state = {
        .State_Mode   = STATE_MODE_IDLE,
        .Control_Mode = CONTROL_MODE_VELOCITY_RAMP,  // 默认速度斜坡模式
        .Fault_State  = FAULT_STATE_NORMAL,
    },
    .parameters = {
        .Rs   = MOTOR_RS,    // 1.736Ω (rb)
        .Ls   = MOTOR_LS,    // 0.622mH (rb)
        .flux = MOTOR_FLUX,  // 0
    },
    .Controller = {
        .inertia     = MOTOR_INERTIA,
        .vel_limit   = MOTOR_VEL_LIMIT,       // 15.75 turn/s
        .current_limit = MOTOR_CURRENT_LIMIT, // 0.11/0.2=0.55A
        .input_updated = true,
        // ...其余从宏定义初始化
    },
    .IqPID = { .mode = PID_POSITION, .Kp = 0, .Ki = 0, .Kd = 0,
               .max_out = IQ_PID_MAX_OUT, .max_iout = IQ_PID_MAX_IOUT },
    .IdPID = { /* 类似 */ },
    .VelPID = { .Kp = 0.12f, .Ki = 0.0001f, .Kd = 0,
                .max_out = VEL_PID_MAX_OUT, .max_iout = VEL_PID_MAX_IOUT },
    .PosPID = { .Kp = 120.0f, .Ki = 0, .Kd = 0,
                .max_out = POS_PID_MAX_OUT, .max_iout = POS_PID_MAX_IOUT },
};
```

#### `tTraj Traj`（trapTraj.c）

```c
tTraj Traj;  // 零初始化（BSS段）
```

---

### 四、系统初始化流程

整个系统从 `main()` 开始，分为 **硬件初始化 → 应用初始化 → RTOS调度 → 中断驱动运行** 四个阶段：

#### 阶段 1: `main()` — MCU 级初始化

```
HAL_Init()
SystemClock_Config()          →  HSE 8MHz → PLL 168MHz
MX_GPIO_Init()
MX_DMA_Init()
MX_ADC1_Init()               →  注入模式，TIM1触发
MX_TIM1_Init()               →  中心对齐PWM, 20kHz
MX_ADC2_Init()               →  NTC温度采样+DMA
MX_SPI3_Init()                →  MT6816编码器
// ...其他外设
RobotInit()                   →  应用初始化
MX_FREERTOS_Init()            →  创建3个RTOS线程
osKernelStart()               →  启动调度器
```

#### 阶段 2: `RobotInit()` — 应用层初始化

```c
void RobotInit(void)
{
    __disable_irq();                    // 关闭所有中断
    DWT_Delay(0.016f);                 // 等待MT6816上电稳定(16ms)
    BSPInit();                          // DWT_Init(168) — 微秒级延时
    LogInit(&huart1);                   // 日志串口
    adc_bsp_init();                     // ADC校准 + 启动注入中断 + ADC2 DMA
    Foc_Pwm_Start();                    // 启动TIM1 PWM输出(50%占空比)
    RGB_DisplayColorById(0);           // LED指示
    __enable_irq();                     // 开启中断 → ADC注入转换开始
}
```

关键点：`adc_bsp_init()` 启动 ADC1 注入中断后，TIM1 的更新事件（即 PWM 周期到达峰值/谷值）会**硬件触发** ADC1 注入通道转换，转换完成后进入 `ADC1_2_IRQHandler`。

#### 阶段 3: 中断驱动 — 20kHz FOC 控制循环

整个 FOC 控制是由 **TIM1 → ADC1 注入中断** 这条硬件链路驱动的，无需 RTOS 参与实时控制：

```
TIM1 更新事件 (20kHz)
    ↓ (硬件触发)
ADC1 注入通道开始转换 (JDR3=A相, JDR2=B相, JDR1=C相, JDR4=母线电压)
    ↓ (转换完成)
ADC1_2_IRQHandler
    → HAL_ADC_IRQHandler(&hadc1)
    → HAL_ADCEx_InjectedConvCpltCallback(&hadc1)
        → GetMotorADC1PhaseCurrent(&motor_data)  // 读取ADC结果寄存器→foc->i_a/b/c/vbus
        → GetMotor_Angle(motor_data.components.encoder)  // SPI读取MT6816 → PLL位置/速度估算
        → MotorStateTask(&motor_data)            // 状态机调度
```

#### 阶段 4: `MotorStateTask()` — 状态机调度

这是 FOC 控制的顶层调度函数，根据 `STATE_MODE` 分发：

```text
┌──────────────────────────────────────────────────────┐
│  STATE_MODE_IDLE                                     │
│  动作: 清PID、FOC_reset、PWM置低                        │
│  自动切换到: STATE_MODE_DETECTING + CURRENT_CALIBRATING│
└──────────────┬───────────────────────────────────────┘
               ↓
┌──────────────────────────────────────────────────────┐
│  STATE_MODE_DETECTING                                │
│  ┌─ SUB_STATE: CURRENT_CALIBRATING                  │
│  │  1000次累加ADC原始值 → 计算Ia/Ib/Ic_offset          │
│  │  完成后 → RSLS_CALIBRATING (或直接RUNNING, 取决于PRE_CALIBRATED) │
│  │                                                    │
│  └─ SUB_STATE: RSLS_CALIBRATING                     │
│       ├─ CS_MOTOR_R_START/LOOP/END  → 测相电阻 Rs   │
│       ├─ CS_MOTOR_L_START/LOOP/END  → 测相电感 Ls   │
│       ├─ CS_DIR_PP_START/LOOP/END   → 判断转向、测极对数│
│       ├─ CS_ENCODER_START/CW_LOOP/CCW_LOOP → 编码器非线性校正 LUT│
│       └─ CS_ENCODER_END → FIR滤波生成128点offset_lut  │
│         完成后 → STATE_MODE_RUNNING                   │
└──────────────┬───────────────────────────────────────┘
               ↓
┌──────────────────────────────────────────────────────┐
│  STATE_MODE_RUNNING                                  │
│  MotorControlTask(motor) → 根据 CONTROL_MODE 执行:    │
│                                                       │
│  CONTROL_MODE_OPEN → 开环V/F（无传感器反馈）            │
│  CONTROL_MODE_TORQUE → 电流闭环                       │
│  CONTROL_MODE_VELOCITY → 速度环+电流环 (5kHz/20kHz)  │
│  CONTROL_MODE_POSITION → 位置环+速度环+电流环         │
│  CONTROL_MODE_VELOCITY_RAMP → 带斜坡的速度环          │
│  CONTROL_MODE_POSITION_RAMP → 梯形轨迹位置环          │
└──────────────────────────────────────────────────────┘
```

---

### 五、关键设计要点总结

1. **面向对象的数据聚合思想**：`MOTOR_DATA` 作为顶层对象，通过指针 (`components.foc/encoder/current`) 组合各硬件模块数据，避免了全局变量的散乱。每个硬件模块（编码器、ADC、FOC算法）的数据独立封装，模块间通过指针耦合。

2. **电流环增益自动计算**：在 `FOC_update_current_gain()` 中，电流环 PI 参数根据电机参数自动计算：
   - `Kp = Ls × vel_limit × pole_pairs × 2π`
   - `Ki = Rs × vel_limit × pole_pairs × 2π × Ts`

3. **编码器 PLL 估算**：使用二阶锁相环（`pll_kp_`/`pll_ki_`）对编码器计数值进行滤波和速度估算，带宽 2000 rad/s，在极低速度下自动锁定零速防止抖动。

4. **电流环积分抗饱和**：在 `FOC_current()` 中，当调制电压超过 SVPWM 最大输出范围时，积分器乘以 0.99 衰减而不是继续累积，防止积分饱和导致的超调。

5. **多速率控制**：电流环运行在 20kHz（与 PWM 频率一致），速度环和位置环以 5kHz 运行（每4个电流周期执行一次），通过 `loop_count` 计数器分频实现。

6. **配置与代码分离**：通过 `MOTOR_PM3510` / `MOTOR_DJI2312` 宏切换电机型号，通过 `lf/rf/lb/rb` 宏选择具体电机（4个电机各有独立的校准参数），`PRE_CALIBRATED` 宏控制是否在校准后直接运行。