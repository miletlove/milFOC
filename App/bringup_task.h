/**
 ******************************************************************************
 * @file    bringup_task.h
 * @author  milFOC Team
 * @brief   FOC Bring-Up State Machine & Command Interface.
 *
 *          Implements the industrial-standard staged Bring-Up flow:
 *            Stage 0: IDLE          — ADC sampling, no PWM (safety check)
 *            Stage 1: PWM_TEST      — 50% duty output (scope verification)
 *            Stage 2: LOCK          — Fixed-angle rotor alignment
 *            Stage 3: OPEN_LOOP     — Rotating voltage vector
 *            Stage 4: CURRENT       — Closed-loop current control (future)
 *            Stage 5: SPEED         — Closed-loop speed control (future)
 *
 *          USB CDC serial command interface (115200-8N1):
 *            mode idle|pwm|lock|open
 *            set uq <volts>        — Set Uq voltage [0~24V]
 *            set speed <Hz>        — Set electrical speed [Hz]
 *            set angle <deg>       — Set lock angle [degrees]
 *            status                — Print system status
 *            help                  — Command list
 *
 *          Telemetry @ 100Hz (VOFA+ FireWater protocol):
 *            CH1=Vbus CH2=Ia CH3=Ib CH4=Ic CH5=ΣI CH6=Mode
 *            CH7=Uq CH8=Theta CH9=Speed CH10=Time CH11=Faults CH12=ADCraw
 ******************************************************************************
 */

#ifndef BRINGUP_TASK_H
#define BRINGUP_TASK_H

#include "general_def.h"
#include "bldc_motor.h"
#include "virtual_encoder.h"
#include <stdbool.h>

/* ======================== FOC Timing ====================================== */

#define BRINGUP_FOC_FREQ    20000.0f        /* FOC execution frequency [Hz] */
#define BRINGUP_TS           (1.0f / BRINGUP_FOC_FREQ)  /* 50µs */
#define BRINGUP_VOFA_DIV     100u            /* VOFA output divider */

/* ======================== Motor Parameters ================================ */

#define BRINGUP_VBUS         24.0f           /* Bus voltage [V] */
#define BRINGUP_POLE_PAIRS   7u              /* Motor pole pairs (5010 series) */

/* ==================== Bring-Up Stage Selection (compile-time) ================
 *
 *  Change BRINGUP_STAGE to switch test phase. Recompile & flash each time.
 *
 *  STAGE 0 (IDLE):      ADC sampling, no PWM. Verify Vbus/Ia/Ib/Ic.
 *  STAGE 1 (PWM_TEST):  50% duty output. Verify 3-phase PWM on scope.
 *  STAGE 2 (LOCK):      Fixed-angle rotor alignment. Verify magnetic lock.
 *  STAGE 3 (OPEN_LOOP): Rotating voltage vector. Verify smooth rotation.
 *
 *  For STAGE 3, adjust SPEED and UQ below for smoother motion:
 *    - Higher UQ  → stronger torque, smoother at low speed
 *    - Higher SPEED → less cogging visible (motor inertia helps)
 *    V/F 曲线模式 (BRINGUP_VF_MODE=1):
 *      Uq = Uq_base + ψ_f × ω_elec
 *      Uq_base = 0.3V  — 克服摩擦的最低电压 (~2.5A @ 0.12Ω)
 *      ψ_f = 0.0021 Wb  — 磁链常数，决定反电动势斜率
 *      例如 8Hz → Uq = 0.3 + 0.0021×50.3 = 0.41V ✅ 低温运行
 * ========================================================================== */

#define BRINGUP_STAGE           3       /* 0=IDLE | 1=PWM | 2=LOCK | 3=OPEN_LOOP */

#define BRINGUP_VF_MODE          1       /* 1=V/F自动电压, 0=固定Uq */
#define BRINGUP_INIT_UQ          2.5f    /* 固定Uq [V] — 仅 VF_MODE=0 时生效 */
#define BRINGUP_UQ_BASE          0.3f    /* V/F 基础电压 [V] — VF_MODE=1 时生效 */
#define BRINGUP_INIT_SPEED       8.0f    /* Electrical speed [Hz] */
#define BRINGUP_INIT_ANGLE       0.0f    /* Lock angle [deg] — only STAGE 2 */

/* ======================== FOC Bring-Up Mode ================================ */

typedef enum
{
    FOC_MODE_IDLE       = 0,    /* PWM off, ADC monitoring only */
    FOC_MODE_PWM_TEST   = 1,    /* 50% fixed duty, scope check */
    FOC_MODE_LOCK       = 2,    /* Fixed angle + configurable Uq */
    FOC_MODE_OPEN_LOOP  = 3,    /* Rotating voltage vector */
    FOC_MODE_CURRENT    = 4,    /* Current closed-loop (future) */
    FOC_MODE_SPEED      = 5,    /* Speed closed-loop (future) */
} FOC_Mode_t;

/* ======================== Protection Configuration ========================= */

#define BRINGUP_VBUS_MAX        30.0f   /* Over-voltage threshold [V] */
#define BRINGUP_VBUS_MIN        6.0f    /* Under-voltage threshold [V] — relaxed for testing */
#define BRINGUP_OVER_CURRENT    20.0f   /* OC threshold [A] — 5010 rated 23A */
#define BRINGUP_ADC_SUM_THRESH  0.5f    /* Ia+Ib+Ic zero-sum check [A] */
#define BRINGUP_MAX_UQ_STAGE0   2.0f    /* Max Uq for STAGE 0~2 [V] */
#define BRINGUP_MAX_UQ_STAGE3   8.0f    /* Max Uq for STAGE 3 [V] */
#define BRINGUP_DEFAULT_UQ      0.5f    /* Default Uq for serial commands [V] */
#define BRINGUP_DEFAULT_SPEED   1.0f    /* Default speed for serial commands [Hz] */
#define BRINGUP_UQ_RAMP         5.0f    /* Uq ramp rate [V/s] — 0→2.5V in 0.5s */
#define BRINGUP_SPEED_RAMP      10.0f   /* Speed ramp rate [Hz/s] */
#define BRINGUP_SKIP_RAMP       1       /* 1=instant — torque beats OC filter lag */
#define BRINGUP_TELEMETRY_DIV   200u    /* Telemetry every N FOC steps → 100Hz */
#define BRINGUP_CMD_POLL_DIV    400u    /* Command poll every N FOC steps → 50Hz */
#define BRINGUP_ADC_CALIB_SAMPLES 2000u /* ADC offset calibration samples (~0.1s) */
#define BRINGUP_ADC_LPF_ALPHA    0.05f  /* IIR filter coefficient (~160Hz @20kHz) */

/* ======================== ADC Reading ====================================== */

typedef struct
{
    /* Raw ADC readings (12-bit, 0-4095) */
    uint16_t i_a_raw;
    uint16_t i_b_raw;
    uint16_t i_c_raw;
    uint16_t vbus_raw;

    /* Calibrated offsets (updated during first ~0.1s of IDLE) */
    float Ia_offset;
    float Ib_offset;
    float Ic_offset;

    /* Non-blocking calibration state */
    uint32_t calib_cnt;
    uint8_t  calib_done;
    uint64_t calib_sum_a;
    uint64_t calib_sum_b;
    uint64_t calib_sum_c;

    /* Currents [A] */
    float i_a;
    float i_b;
    float i_c;

    /* Filtered currents (IIR) */
    float i_a_filt;
    float i_b_filt;
    float i_c_filt;
    uint8_t filter_init;

    /* Bus voltage [V] */
    float vbus;
} BringUp_ADC_t;

/* ======================== Open-Loop Controller ============================= */

typedef struct
{
    float   theta_elec;        /* Electrical angle [rad], [0, 2π) */
    float   speed_elec_hz;     /* Target electrical speed [Hz] */
    float   speed_elec_radps;  /* Target electrical speed [rad/s] */
    float   speed_current_hz;  /* Current (ramped) speed [Hz] */
    float   sin_val;
    float   cos_val;
    float   uq_target;         /* Target Uq [V] */
    float   uq_current;        /* Current (ramped) Uq [V] */
    uint8_t pole_pairs;
} OpenLoop_Ctrl_t;

/* ======================== Bring-Up Instance ================================ */

typedef struct
{
    FOC_Mode_t       mode;             /* Current operating mode */
    FOC_Mode_t       prev_mode;        /* Previous mode (for transitions) */
    uint32_t         step_cnt;         /* FOC step counter */
    uint32_t         run_time_ms;      /* Runtime [ms] */

    BringUp_ADC_t    adc;              /* ADC readings */
    OpenLoop_Ctrl_t  ol;               /* Open-loop controller state */
    FOC_DATA         foc;              /* FOC math data (local) */

    /* Protection state */
    bool             fault_ov;         /* Over-voltage fault */
    bool             fault_uv;         /* Under-voltage fault */
    bool             fault_oc;         /* Over-current fault */
    bool             fault_adc_sum;    /* ADC zero-sum fault */
    bool             pwm_active;       /* PWM outputs enabled */

    /* USB CDC command buffer */
    char             cmd_buf[64];
    uint8_t          cmd_len;
    bool             cmd_ready;

} BringUp_t;

/* ======================== Global Instance ================================== */

extern BringUp_t bringup;

/* ======================== Public API ======================================= */

/**
 * @brief  Initialize the Bring-Up module
 *         Sets mode=IDLE, initializes ADC, starts TIM1 for ADC trigger.
 */
void BringUp_Init(void);

/**
 * @brief  Execute one Bring-Up step (call @ 20kHz from main loop)
 *
 *         Pipeline:
 *         1. Read ADC (Vbus, Ia, Ib, Ic) — always
 *         2. Protection checks — always
 *         3. Mode-specific action:
 *            IDLE:      set PWM duty=50% neutral (or disable outputs)
 *            PWM_TEST:  set PWM duty=50% on all phases
 *            LOCK:      fixed θ + Uq → SVPWM
 *            OPEN_LOOP: OpenLoop_Update → θ advance → SVPWM
 *         4. Increment step counter / runtime
 */
void BringUp_Step(void);

/**
 * @brief  Process one USB CDC command line (call @ 50Hz from main loop)
 *         Parses "mode xxx", "set xxx", "status", "help" commands.
 */
void BringUp_ProcessCmd(void);

/**
 * @brief  Send telemetry to USB CDC (call @ 100Hz from main loop)
 *         Format: "Time Vbus Ia Ib Ic Mode Uq Theta\r\n"
 */
void BringUp_Telemetry(void);

/**
 * @brief  Feed received USB data into the command parser
 * @param  data  Received byte
 *
 * @note   Called from CDC_Receive_FS callback or main loop polling.
 *         Accumulates characters until '\n' or '\r', then sets cmd_ready.
 */
void BringUp_FeedChar(uint8_t data);

/**
 * @brief  Emergency stop — disable PWM, set mode=IDLE
 */
void BringUp_EmergencyStop(void);

#endif /* BRINGUP_TASK_H */
