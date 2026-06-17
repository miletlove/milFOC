# App — 应用层

> **职责**: 系统集成、任务调度与状态管理。该层连接 BSP 和 Modules 层，实现完整的电机控制应用逻辑。

---

## 📋 目录

- [架构概述](#架构概述)
- [文件清单与功能说明](#文件清单与功能说明)
- [系统初始化流程](#系统初始化流程)
- [主循环调度](#主循环调度)
- [CAN 协议定义](#can-协议定义)

---

## 架构概述

```
App/
├── robot.c/.h          # ★ 系统初始化入口 + 主循环调度
├── robot_def.h         # 核心定义: CAN ID / 协议帧结构 / 命令枚举
├── cmd_task.c/.h       # CAN 命令解析与调度 (~200Hz)
├── motor_task.c/.h     # ADC JEOC 中断回调 (20kHz 电流环)
└── README.md           # 本文件
```

**调用层级**:

```
robot.c (裸机主循环)
├── RobotInit()          # 一次性硬件初始化 (关中断)
│   ├── BSPInit()        # DWT
│   ├── LogInit()        # 日志
│   ├── adc_bsp_init()   # ADC
│   ├── Foc_Pwm_Start()  # PWM 50% 安全态
│   ├── RobotCMDInit()   # CAN 实例
│   └── LED 初始化
│
└── RobotTask()          # 主循环 (~1kHz)
    ├── RobotCMDTask()   # CAN 收/发/指令执行
    └── 守护任务、VOFA 等

motor_task.c (仅 JEOC ISR 上下文中, 20kHz)
└── MotorTask()          # 电流环 FOC: 采样→变换→PID→SVPWM→PWM
```

---

## 文件清单与功能说明

### `robot.c` / `robot.h` — 系统初始化入口

| 属性 | 说明 |
|------|------|
| **头文件** | `robot.c`, `robot.h` |
| **包含** | `bsp_init.h`, `bsp_log.h`, `robot_def.h`, `cmd_task.h`, `motor_task.h`, `led.h` |
| **核心函数** | |

#### `RobotInit(void)` — 硬件初始化

```
执行顺序 (关键, 不可随意调换):

1. __disable_irq()           — 关全局中断
2. DWT_Delay(0.016f)         — 等待 MT6816 编码器上电 (16ms 无输出期)
3. BSPInit()                 — DWT 周期计数器初始化 (168MHz)
4. LogInit(&huart1)          — 日志系统绑定 USART1
5. adc_bsp_init()            — ADC1 注入组配置 + TIM1 触发同步
6. Foc_Pwm_Start()           — 启动 PWM (50% 占空比, 安全态)
7. RobotCMDInit()            — CAN 通信实例注册
8. LED 指示初始化完成
9. __enable_irq()            — 开全局中断
```

#### `RobotTask(void)` — 主循环任务

```
被 main() 或 FreeRTOS 以 ~1kHz 周期调用:

1. RobotCMDTask()            — 解析 CAN 命令, 更新 motor_data 状态
2. 可扩展: 守护任务、VOFA 数据发送
```

---

### `cmd_task.c` / `cmd_task.h` — 命令调度

| 属性 | 说明 |
|------|------|
| **频率** | ~200Hz (或新 CAN 帧到达时触发) |
| **通信接口** | CAN (通过 `can_driver.c`), 未来可扩展 USB |
| **核心函数** | |

#### `RobotCMDInit(void)` — 命令系统初始化

```c
// 注册 CAN 通信实例
FDCANComm_Init_Config_s config = {
    .fdcan_handle  = &hfdcan1,
    .tx_id         = FDCAN_M4_ID + FC_MOTOR_ID,
    .rx_id         = FDCAN_M4_ID,
    .send_data_len = sizeof(Cmd_Tx_s),   // 20 字节遥测帧
    .recv_data_len = sizeof(Cmd_Rx_s),   // 16 字节命令帧
};
ins = FDCANCommInit(&config);
```

#### `RobotCMDTask(void)` — 命令处理循环

| 步骤 | 操作 |
|------|------|
| 1 | `FDCANCommGet(ins)` → 读取接收到的 CAN 帧 |
| 2 | 根据 `cmd_rx.cmd_id` 的 switch 分支执行对应操作 |
| 3 | `FDCANCommSend(ins, &cmd_tx)` → 回复遥测帧 |
| 4 | 清零接收缓冲区 |

**支持的命令**:

| 命令 ID | 操作 |
|---------|------|
| `CMD_ID_GET_POSITION` (0x01) | 位置梯形轨迹控制: 更新 target_pos/vel → 触发 traj_plan |
| `CMD_ID_GET_VELOCITY` (0x02) | 速度斜坡控制: 更新 target_vel |
| `CMD_ID_GET_TORQUE` (0x03) | 力矩控制: 更新 target_torque |
| `CMD_ID_CLEAR_ERRORS` (0x04) | 清除故障: 复位 STATE_MODE + Fault_State → LED 变绿 |
| `CMD_ID_GET_ENABLED` (0x05) | 使能电机: Foc_Pwm_LowSides() 自举电容预充电 |
| `CMD_ID_GET_STOP` (0x06) | 紧急停止: 全下桥导通 → 电机刹车 + LED 变红 |

**遥测回复字段**:

| 字段 | 含义 | 来源 |
|------|------|------|
| `cmd_tx_state` | 故障状态码 | `motor_data.state.Fault_State` |
| `cmd_tx_vel` | 当前速度 [turn/s] | `encoder->vel_estimate_` |
| `cmd_tx_pos` | 当前位置 [turn] | `encoder->pos_estimate_` |
| `cmd_tx_iq` | Q轴电流 [A] | `foc->i_q` |
| `cmd_tx_vbus` | 母线电压 [V] | `foc->vbus` |

---

### `motor_task.c` / `motor_task.h` — 实时中断任务

| 属性 | 说明 |
|------|------|
| **执行上下文** | ADC1/2 JEOC 中断 (注入组转换完成) |
| **频率** | 20kHz (PWM 开关频率, 周期 50µs) |
| **优先级** | 抢占优先级 0 (全系统最高) |
| **执行时间约束** | < 15µs |
| **核心函数** | `MotorTask(void)` — JEOC 中断回调中直接调用 |

#### 中断约束

| 禁止 | 原因 |
|------|------|
| ❌ `printf` / `malloc` | 阻塞/不确定延时 |
| ❌ `LOG_DEBUG` / `LOG_INFO` 等 | USART DMA 竞争 |
| ❌ 浮点 printf 格式化 | 极慢 (数百 µs) |
| ❌ HAL 延迟函数 | 阻塞 |
| ✅ 直接寄存器读取 (`ADC1->JDRx`, `TIM1->CCRx`) | 零开销 |
| ✅ `fast_sincos` / `sinf` / `cosf` | 硬件 FPU |

---

### `robot_def.h` — 核心应用定义

| 属性 | 说明 |
|------|------|
| **功能** | 集中定义 CAN ID、命令协议帧和电机标识符 |
| **编辑指南** | 修改此文件以配置 CAN ID、多电机 ID 和协议数据布局 |

#### CAN ID 定义

```c
#define FC_MOTOR_ID  0x001   // milFOC 电机 CAN 标识符

typedef enum {
    FDCAN_M1_ID = 0x100,  // 电机1 (前左?)
    FDCAN_M2_ID = 0x1FF,  // 电机2 (前右?)
    FDCAN_M3_ID = 0x200,  // 电机3 (后左?)
    FDCAN_M4_ID = 0x2FF,  // 电机4 (后右?)
} FDCAN_ID_e;
```

#### 命令 ID 枚举

```c
typedef enum {
    CMD_ID_GET_POSITION = 0x01,  // 设定位置目标
    CMD_ID_GET_VELOCITY = 0x02,  // 设定速度目标
    CMD_ID_GET_TORQUE   = 0x03,  // 设定力矩目标
    CMD_ID_CLEAR_ERRORS = 0x04,  // 清除故障状态
    CMD_ID_GET_ENABLED  = 0x05,  // 使能电机 (预充电)
    CMD_ID_GET_STOP     = 0x06,  // 紧急停止
    CMD_ID_SET_PID      = 0x07,  // 设置 PID 参数
    CMD_ID_GET_STATUS   = 0x08,  // 查询电机状态
} Cmd_Id_e;
```

#### 协议帧结构

```c
#pragma pack(1)

// 接收: 主机 → 电机 (16 bytes)
typedef struct {
    int    cmd_id;          //  4B  命令 ID
    float  cmd_rx_vel;      //  4B  速度设定 [turn/s]
    float  cmd_rx_pos;      //  4B  位置设定 [turn]
    float  cmd_rx_torque;   //  4B  力矩设定 [Nm]
} Cmd_Rx_s;

// 发送: 电机 → 主机 (20 bytes)
typedef struct {
    int    cmd_tx_state;    //  4B  电机状态码
    float  cmd_tx_vel;      //  4B  当前速度 [turn/s]
    float  cmd_tx_pos;      //  4B  当前位置 [turn]
    float  cmd_tx_iq;       //  4B  Q轴电流 [A]
    float  cmd_tx_vbus;     //  4B  母线电压 [V]
} Cmd_Tx_s;

#pragma pack()
```

> ⚠️ CAN 经典帧最大载荷为 8 字节。上述结构体超过 8 字节时, 实际仅发送前 8 字节 (CAN DLC=8)。多帧传输或 CAN FD 待后续支持。

---

## 系统初始化流程

```
上电
  │
  ▼
Reset_Handler (startup_stm32g431xx.s)
  │
  ├── SystemInit()                  # CMSIS 系统初始化
  ├── HAL_Init()                    # HAL 库初始化
  ├── SystemClock_Config()          # 时钟树配置 (HSE/PLL/SysClk)
  ├── MX_GPIO_Init()                # GPIO 初始化
  ├── MX_DMA_Init()                 # DMA 初始化
  ├── MX_ADC1_Init() / MX_ADC2_Init()
  ├── MX_TIM1_Init()                # TIM1 PWM (20kHz)
  ├── MX_SPI1_Init()                # SPI1 (MT6816)
  ├── MX_FDCAN1_Init()              # FDCAN1
  ├── MX_USART1_UART_Init()         # USART1 (日志)
  ├── MX_USB_PCD_Init()             # USB CDC
  ├── MX_CORDIC_Init()              # CORDIC 加速器
  │
  ├── RobotInit()                   # ★ 应用层初始化 (关中断)
  │
  ├── while(1) { RobotTask(); }     # 主循环 (~1kHz)
  │
  └── (ADC JEOC 中断异步触发 MotorTask, 20kHz)
```

---

## 主循环调度

```
main() while(1)  [裸机模式, ~1kHz]
  │
  ├── RobotTask()
  │   ├── RobotCMDTask()            ─── 解析 CAN / 执行指令 / 回复遥测
  │   ├── (可选) DaemonTask()       ─── 模块心跳检测
  │   └── (可选) Vofa_Packet()      ─── 实时数据发送至上位机
  │
  └── (后台) HAL_USB_CDC_xxx        ─── USB 收发处理

ADC1 JEOC ISR  [硬件触发, 20kHz, 独立于主循环]
  │
  └── MotorTask()                   ─── 完整电流环 FOC (< 15µs)
```
