让我先查看项目中的 FOC 相关代码，了解完整的电压计算流程。

Read [](file:///e%3A/STM32CubeMX/Project/milFOC/MotorConrol/controller.hpp)

Read [](file:///e%3A/STM32CubeMX/Project/milFOC/MotorConrol/foc.hpp)

Read [](file:///e%3A/STM32CubeMX/Project/milFOC/MotorConrol/foc.cpp)

Read [](file:///e%3A/STM32CubeMX/Project/milFOC/MotorConrol/motor.hpp)

Read [](file:///e%3A/STM32CubeMX/Project/milFOC/MotorConrol/controller.cpp#1-1), lines 1 to 200

Read [](file:///e%3A/STM32CubeMX/Project/milFOC/MotorConrol/axis.cpp#1-1), lines 1 to 200

Read [](file:///e%3A/STM32CubeMX/Project/milFOC/MotorConrol/motor.cpp#1-1), lines 1 to 200

Read [](file:///e%3A/STM32CubeMX/Project/milFOC/MotorConrol/encoder.hpp#1-1), lines 1 to 100

Read [](file:///e%3A/STM32CubeMX/Project/milFOC/MotorConrol/low_level.cpp#1-1), lines 1 to 200

现在我已经完整阅读了算法库的全部 FOC 链代码，以下是完整的解析。

---

## FOC 完整电压输出计算过程

整个算法库的 FOC 电压计算可以分解为 **6 个步骤**，从用户发出指令到三相 PWM 输出，形成一个闭环。

```
  ┌──────────────┐
  │  用户指令     │  (位置/速度/力矩设定)
  └──────┬───────┘
         ▼
  ┌──────────────┐
  │  Controller  │  (位置环→速度环→力矩环 PID)
  │  → torque    │
  └──────┬───────┘
         ▼  torque_setpoint
  ┌──────────────┐
  │  Motor       │  (力矩→电流 Idq_setpoint 转换)
  │  → Idq_set   │  或者直接 Vdq_setpoint (电压模式)
  └──────┬───────┘
         ▼ Idq_setpoint / Vdq_setpoint
  ┌──────────────────────────────┐
  │  FieldOrientedController     │  FOC 核心
  │                              │
  │  ① Clarke 变换: Ia,Ib,Ic→Iα,Iβ│
  │  ② Park 变换: Iα,Iβ→Id,Iq    │
  │  ③ PI 电流环: Idq_err→Vdq    │
  │  ④ 反 Park 变换: Vd,Vq→Vα,Vβ │
  │  ⑤ SVPWM: Vα,Vβ→tA,tB,tC   │
  └──────────┬───────────────────┘
             ▼ tA, tB, tC (占空比)
  ┌──────────────────┐
  │  PWM 定时器      │  → 三相逆变器 → 电机
  └──────────────────┘
             │
             ▼ (反馈)
  ┌──────────────────┐
  │  电流传感器 ADC   │  → Ia, Ib, Ic (回到 ①)
  └──────────────────┘
```

---

### 第 0 步：Controller 层 — 位置/速度/力矩控制

文件 controller.cpp 中的 `Controller::update()` 根据控制模式计算 `torque_setpoint_`：

| 控制模式 | 计算方式 |
|---|---|
| **位置控制** | `torque = (pos_ref - pos) × Kp + vel_integrator` |
| **速度控制** | `torque = (vel_ref - vel) × Kp + vel_integrator` |
| **力矩控制** | `torque = torque_ref`（直通）|

结果输出到 `torque_output_`，该端口连接到 `Motor` 的 `torque_setpoint_src_`。

---

### 第 0.5 步：Motor 层 — 力矩转电流

在 motor.cpp 中，`Motor::update()` 将力矩转换为电流指令：

$$
\begin{aligned}
I_{q\_setpoint} &= \frac{\text{torque\_setpoint}}{\text{torque\_constant}} \\
I_{d\_setpoint} &= 0 \quad (\text{对于表贴式 PMSM，MTPA 模式下另有计算})
\end{aligned}
$$

结果写入 `Idq_setpoint_`，送入 `FieldOrientedController`。

---

### 第 1 步：Clarke 变换 (三相→两相静止)

**代码位置**：general_def.h 中的 `clarke_transform()`，以及 foc.cpp 中 `AlphaBetaFrameController::on_measurement()`。

**输入**：ADC 采样得到的三相电流 $(I_a, I_b, I_c)$

**计算**：

$$
\boxed{\begin{aligned}
I_\alpha &= I_a \\[2pt]
I_\beta &= \frac{1}{\sqrt{3}}(I_b - I_c)
\end{aligned}}
$$

**物理意义**：将三相 120° 坐标系投影到正交的 $(\alpha, \beta)$ 坐标系。

**代码片段**（foc.cpp 第 15-18 行）：
```cpp
Ialpha_beta = {
    (*currents)[0],
    one_by_sqrt3 * ((*currents)[1] - (*currents)[2])
};
```

---

### 第 2 步：Park 变换 (静止→旋转 dq)

**代码位置**：foc.cpp 中 `FieldOrientedController::get_alpha_beta_output()` 第 106-110 行。

**输入**：$I_\alpha, I_\beta$ 和电角度 $\theta_e$

**计算**：

首先需要知道当前时刻的精确电角度，代码中通过**相位预测**补偿控制延迟：

```cpp
float I_phase = phase + phase_vel * 
    ((float)(int32_t)(i_timestamp_ - ctrl_timestamp_) / (float)TIM_1_8_CLOCK_HZ);
```

然后用 `our_arm_cos_f32` / `our_arm_sin_f32`（或 `fast_sincos`）计算正余弦：

$$
\boxed{\begin{aligned}
I_d &= I_\alpha \cos\theta_e + I_\beta \sin\theta_e \\[2pt]
I_q &= -I_\alpha \sin\theta_e + I_\beta \cos\theta_e
\end{aligned}}
$$

**物理意义**：
- $I_d$（直轴电流）→ 控制磁通。通常控制为 0，弱磁时变为负值
- $I_q$（交轴电流）→ 控制转矩。与输出转矩成正比

**代码**（foc.cpp 第 109-110 行）：
```cpp
Idq = {
    c_I * Ialpha + s_I * Ibeta,   // Id
    c_I * Ibeta - s_I * Ialpha    // Iq
};
```

---

### 第 3 步：PI 电流环控制 (Idq→Vdq)

**代码位置**：foc.cpp 第 124-156 行。

**输入**：$I_{d\_ref}, I_{q\_ref}$（来自 Controller/Motor）和 $I_d, I_q$（来自 Park 变换反馈）

**计算**：

电流误差：

$$
\begin{aligned}
I_{err\_d} &= I_{d\_ref} - I_d \\
I_{err\_q} &= I_{q\_ref} - I_q
\end{aligned}
$$

PI 控制器输出（带前馈）：

$$
\boxed{\begin{aligned}
V_d &= K_p \cdot I_{err\_d} + I_{integ\_d} + V_{d\_ff} \\[2pt]
V_q &= K_p \cdot I_{err\_q} + I_{integ\_q} + V_{q\_ff}
\end{aligned}}
$$

其中积分项：

$$
\begin{aligned}
I_{integ\_d} &\mathrel{+}= K_i \cdot I_{err\_d} \cdot \Delta t \\
I_{integ\_q} &\mathrel{+}= K_i \cdot I_{err\_q} \cdot \Delta t
\end{aligned}
$$

**增益计算公式**（在 `update_current_controller_gains()` 中设置）：

$$
\begin{aligned}
K_p &= L \cdot \omega_{bw} \quad &(\text{比例增益，单位 V/A})\\
K_i &= R \cdot \omega_{bw} \quad &(\text{积分增益，单位 V/(A·s)})
\end{aligned}
$$

其中 $L$ 为相电感，$R$ 为相电阻，$\omega_{bw}$ 为电流环带宽（默认 1000 rad/s）。

---

### 第 4 步：反 Park 变换 (Vd,Vq→Vα,Vβ)

**代码位置**：foc.cpp 第 162-166 行，以及 general_def.h 中的 `inverse_park()`。

**输入**：$V_d, V_q$（PI 输出）和电角度 $\theta_e$（含补偿预测）

**计算**：

同样需要相位预测补偿 PWM 输出延迟：

```cpp
float pwm_phase = phase + phase_vel * 
    ((float)(int32_t)(output_timestamp - ctrl_timestamp_) / (float)TIM_1_8_CLOCK_HZ);
```

$$
\boxed{\begin{aligned}
V_\alpha &= V_d\cos\theta_e - V_q\sin\theta_e \\[2pt]
V_\beta &= V_d\sin\theta_e + V_q\cos\theta_e
\end{aligned}}
$$

**注意**：电压先归一化为调制比 `mod_d, mod_q`，再反 Park：

```cpp
float mod_to_V = (2.0f / 3.0f) * vbus_voltage;  // 母线电压→最大可输出电压
float V_to_mod = 1.0f / mod_to_V;

mod_d = V_to_mod * Vd;  // [0~1] 归一化
mod_q = V_to_mod * Vq;

// 反 Park
mod_alpha = c_p * mod_d - s_p * mod_q;
mod_beta = c_p * mod_q + s_p * mod_d;
```

**为什么除 $(2/3)V_{bus}$**：SVPWM 的线性调制区最大不失真输出电压为 $V_{dc}/\sqrt{3}$，但归一化到 $[0,1]$ 范围时使用 $(2/3)V_{bus}$ 作为基准，使得调制比与 SVPWM 的占空比计算兼容。

---

### 第 5 步：SVPWM (Vα,Vβ→tA,tB,tC)

**代码位置**：foc.cpp 第 32-34 行调用 `SVM()`，实现在 general_def.h 的 `svm()` 函数中。

这是 **α-β 坐标系到三相 PWM 占空比** 的最后一步，也是**6 个扇区的核心作用所在**。

#### 5.1 扇区判定

根据 $(V_\alpha, V_\beta)$ 落在哪个 60° 扇区：

```cpp
if (beta >= 0.0f) {
    if (alpha >= 0.0f) {
        if (ONE_BY_SQRT3 * beta > alpha)  Sextant = 2;
        else                                Sextant = 1;
    } else {
        if (-ONE_BY_SQRT3 * beta > alpha) Sextant = 3;
        else                                Sextant = 2;
    }
} else {
    // ... 类似，共 6 个扇区
}
```

等价于在复平面上用直线 $\beta = \pm\sqrt{3}\alpha$ 和 $\beta = 0$ 划分扇区：

```
判定条件                对应的扇区分界线
β = 0                  水平轴
β = √3 · α             60° 斜线
β = -√3 · α            -60° 斜线
```

#### 5.2 各扇区矢量作用时间

每个扇区用相邻两个基本矢量合成目标矢量。以扇区 1 为例：

$$
\begin{aligned}
t_1 &= V_\alpha - \frac{1}{\sqrt{3}}V_\beta \quad (\text{V1 作用时间})\\
t_2 &= \frac{2}{\sqrt{3}}V_\beta \quad\quad\quad\; (\text{V2 作用时间})
\end{aligned}
$$

**6 个扇区的公式循环轮换**：

| 扇区 | 相邻矢量 | $t_x$ 公式 | $t_y$ 公式 |
|---|---|---|---|
| 1 | V1-V2 | $V_\alpha - \frac{\beta}{\sqrt{3}}$ | $\frac{2\beta}{\sqrt{3}}$ |
| 2 | V2-V3 | $V_\alpha + \frac{\beta}{\sqrt{3}}$ | $-V_\alpha + \frac{\beta}{\sqrt{3}}$ |
| 3 | V3-V4 | $\frac{2\beta}{\sqrt{3}}$ | $-V_\alpha - \frac{\beta}{\sqrt{3}}$ |
| 4 | V4-V5 | $-V_\alpha + \frac{\beta}{\sqrt{3}}$ | $-\frac{2\beta}{\sqrt{3}}$ |
| 5 | V5-V6 | $-V_\alpha - \frac{\beta}{\sqrt{3}}$ | $V_\alpha - \frac{\beta}{\sqrt{3}}$ |
| 6 | V6-V1 | $-\frac{2\beta}{\sqrt{3}}$ | $V_\alpha + \frac{\beta}{\sqrt{3}}$ |

**仔细观察可以发现，6 个公式本质上是同一个公式在 6 个不同角度区间上的投影**。这就是为什么必须用 `switch/case`——每个扇区的 $(\alpha,\beta)$ 到 $(t_x,t_y)$ 的线性组合系数不同。

#### 5.3 三相占空比计算

以七段式对称发波，零矢量 $V_0(000)$ 和 $V_7(111)$ 平均分配剩余时间：

扇区 1 的例子：

$$
\begin{aligned}
t_A &= \frac{1 - t_1 - t_2}{2} \\
t_B &= t_A + t_1 \\
t_C &= t_B + t_2
\end{aligned}
$$

每个扇区中 $t_A, t_B, t_C$ 的大小顺序轮换：

| 扇区 | 占空比大小顺序 | 对应 PWM 波形 |
|---|---|---|
| 1 | $t_A \le t_B \le t_C$ | A 相导通时间最短，C 相最长 |
| 2 | $t_B \le t_A \le t_C$ | B 相最短，C 相最长 |
| 3 | $t_B \le t_C \le t_A$ | B 相最短，A 相最长 |
| 4 | $t_C \le t_B \le t_A$ | C 相最短，A 相最长 |
| 5 | $t_C \le t_A \le t_B$ | C 相最短，B 相最长 |
| 6 | $t_A \le t_C \le t_B$ | A 相最短，B 相最长 |

#### 5.4 有效性检查

```cpp
int result_valid = 
    *tA >= 0.0f && *tA <= 1.0f && 
    *tB >= 0.0f && *tB <= 1.0f && 
    *tC >= 0.0f && *tC <= 1.0f;
```

如果 $|V_{\text{ref}}| > V_{dc}/\sqrt{3}$（过调制），占空比会超出 $[0,1]$，返回 `-1`。

---

### 第 6 步：PWM 定时器输出

foc.cpp 中的 `get_output()` 将 SVPWM 计算的占空比写入 `pwm_timings[3]`，经过 low_level.cpp 中的 `apply_pwm_timings()` 写入定时器 CCR 寄存器，生成三相 PWM 波形驱动逆变器。

---

## α, β, 6 扇区在整个算法中的作用总结

| 量 | 角色 | 在算法中的位置 |
|---|---|---|
| $I_\alpha, I_\beta$ | **电流反馈**在静止坐标系中的表示 | Clarke 变换的输出，Park 变换的输入 |
| $V_\alpha, V_\beta$ | **电压指令**在静止坐标系中的表示 | 反 Park 变换的输出，SVPWM 的输入 |
| **6 个扇区** | 将 $(\alpha,\beta)$ 平面划分为 6 个 60° 区域 | SVPWM 中决定用哪两个基本矢量合成目标电压 |
| $t_A, t_B, t_C$ | 三相 PWM 占空比 | SVPWM 输出，直接写入定时器 |

**α-β 坐标系是连接电流环（旋转 dq 坐标系）和 PWM 调制（三相静止坐标系）之间的桥梁**。6 个扇区则是从连续电压向量到离散开关状态的分段线性映射。

---

如需更深入的某个环节（如 PI 增益整定、弱磁控制、MTPA、死区补偿等），可以继续展开。