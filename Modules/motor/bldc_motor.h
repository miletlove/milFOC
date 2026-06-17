/**
 ******************************************************************************
 * @file    bldc_motor.h
 * @author  milFOC Team
 * @brief   BLDC/PMSM FOC mathematical core module.
 *          Contains Clarke, Park, Inverse Park, SVPWM transforms and
 *          the FOC_DATA structure for all intermediate calculation values.
 *
 * @note    This module is pure math - no hardware dependencies.
 *          All transforms operate on the FOC_DATA structure.
 *          For STM32G431, sin/cos should use CORDIC accelerator when possible.
 *
 * Hardware-specific motor parameters are configured via macros.
 * Target motor: 5010 750KV BLDC
 ******************************************************************************
 */

#ifndef BLDC_MOTOR_H
#define BLDC_MOTOR_H

#include "general_def.h"
#include "tim.h"

/* ======================== Motor Selection ================================= */
#define MOTOR_5010_750KV 1     /* Target: 5010 750KV PMSM/BLDC */
#define MOTOR_PM3510     0
#define MOTOR_DJI2312    0

/* ======================== Battery Configuration ============================ */
#define N_BASE       3.0f             /* 3S battery */
#define BATVEL       (4.0f * N_BASE)  /* Nominal battery voltage */
#define INVBATVEL    (1.0f / BATVEL)  /* Reciprocal for normalization */

/* Pre-calibration flag: set 0 for first boot (auto-calibrate), then 1 */
#define PRE_CALIBRATED 0

/* ======================== Motor 5010 750KV Parameters ====================== */
#if MOTOR_5010_750KV
#if PRE_CALIBRATED
/* TODO: Fill in calibrated values after running auto-calibration */
#define MPTOR_P         7u               /* Pole pairs */
#define MOTOR_RS        0.2f             /* Phase resistance (Ohm) - TODO: calibrate */
#define MOTOR_LS        0.00005f         /* Phase inductance (H) - TODO: calibrate */
#define MOTOR_FLUX      0.0f             /* Rotor flux linkage (Wb) */
#define MOTOR_OFFSET    0                /* Encoder offset */
#define MOTOR_DIRECTION UNKNOWN          /* CW=1, CCW=-1, UNKNOWN=0 */
#else
#define MPTOR_P         0u               /* Auto-detect */
#define MOTOR_RS        0.0f
#define MOTOR_LS        0.0f
#define MOTOR_FLUX      0.0f
#define MOTOR_OFFSET    0
#define MOTOR_DIRECTION UNKNOWN
#endif

/* Control parameters for 5010 750KV */
#define MOTOR_INERTIA              0.0001f     /* Rotor inertia [A/(turn/s^2)] */
#define MOTOR_CURRENT_RAMP_RATE    0.001f      /* Torque ramp rate [Nm/s] */
#define MOTOR_VEL_RAMP_RATE        15.0f       /* Velocity ramp rate [(turn/s)/s] */
#define MOTOR_TRAJ_VEL             15.0f       /* Max trajectory velocity [turn/s] */
#define MOTOR_TRAJ_ACCEL           5.0f        /* Trajectory acceleration [(turn/s)/s] */
#define MOTOR_TRAJ_DECEL           5.0f        /* Trajectory deceleration [(turn/s)/s] */
#define MOTOR_TORQUE_CONST         0.05f       /* Torque constant [Nm/A] - TODO: measure */
#define MOTOR_TORQUE_LIMIT         0.1f        /* Torque limit [Nm] */
#define MOTOR_VEL_LIMIT            50.0f       /* Velocity limit [turn/s] (750KV*12V) */
#define MOTOR_VOLTAGE_LIMIT        12.0f       /* Voltage limit [V] */
#define MOTOR_CURRENT_LIMIT        2.0f        /* Current limit [A] */
#define MOTOR_CURRENT_CTRL_P_GAIN  0.0f        /* Auto-calculated after calibration */
#define MOTOR_CURRENT_CTRL_I_GAIN  0.0f        /* Auto-calculated after calibration */
#define MOTOR_CURRENT_CTRL_BANDWIDTH 230.0f    /* Current loop bandwidth [rad/s] */
#define MOTOR_INPUT_CURRENT        0.5f        /* Default input current [A] */
#define MOTOR_INPUT_TORQUE         0.0f
#define MOTOR_INPUT_VELOCITY       0.0f
#define MOTOR_INPUT_POSITION       0.0f

/* PID gains */
#define MOTOR_IQ_PID_KP  0.0f    /* Auto-calculated */
#define MOTOR_IQ_PID_KI  0.0f
#define MOTOR_IQ_PID_KD  0.0f
#define MOTOR_ID_PID_KP  0.0f
#define MOTOR_ID_PID_KI  0.0f
#define MOTOR_ID_PID_KD  0.0f
#define MOTOR_VEL_PID_KP 0.12f
#define MOTOR_VEL_PID_KI 0.001f
#define MOTOR_VEL_PID_KD 0.0f
#define MOTOR_POS_PID_KP 30.0f
#define MOTOR_POS_PID_KI 0.0f
#define MOTOR_POS_PID_KD 0.0f

#endif /* MOTOR_5010_750KV */

/* ======================== PID Output Limits =============================== */
#define CURRENT_PID_MAX_OUT  ((BATVEL) / _SQRT3)
#define IQ_PID_MAX_OUT       CURRENT_PID_MAX_OUT
#define IQ_PID_MAX_IOUT      CURRENT_PID_MAX_OUT
#define ID_PID_MAX_OUT       CURRENT_PID_MAX_OUT
#define ID_PID_MAX_IOUT      CURRENT_PID_MAX_OUT
#define VEL_PID_MAX_OUT      MOTOR_CURRENT_LIMIT
#define VEL_PID_MAX_IOUT     MOTOR_CURRENT_LIMIT
#define POS_PID_MAX_OUT      (MOTOR_VEL_LIMIT - 3.0f)
#define POS_PID_MAX_IOUT     (MOTOR_VEL_LIMIT - 3.0f)

/* ======================== Calibration Parameters =========================== */
#define CURRENT_MAX_CALIB     0.5f    /* Max current during calibration [A] */
#define VOLTAGE_MAX_CALIB     3.0f    /* Max voltage during calibration [V] */
#define OFFSET_LUT_NUM        128     /* Encoder offset LUT size */
#define COGGING_MAP_NUM       3000    /* Cogging torque map size */
#define CALB_SPEED            M_2PI   /* Calibration speed [rad/s] */
#define MOTOR_POLE_PAIRS_MAX  20      /* Auto-detect max pole pairs */
#define SAMPLES_PER_PPAIR     OFFSET_LUT_NUM

/* ======================== Protection Limits =============================== */
#define BATVEL_MAX_LIMIT   (BATVEL * 1.2f)          /* Over-voltage threshold */
#define BATVEL_MIN_LIMIT   (BATVEL / 1.5f)          /* Under-voltage threshold */
#define SPEED_MAX_LIMIT    (MOTOR_VEL_LIMIT * 1.2f) /* Over-speed threshold */
#define TEMP_MAX_LIMIT     65.0f                    /* Over-temperature [degC] */

/* ======================== PWM Timing ====================================== */
#define TIMER1_CLK_MHz      168                     /* TIM1 clock [MHz] */
#define PWM_FREQUENCY       20000                   /* PWM frequency [Hz] */
#define PWM_MEASURE_PERIOD  (1.0f / (float)PWM_FREQUENCY)  /* PWM period [s] */
#define CURRENT_MEASURE_PERIOD  PWM_MEASURE_PERIOD

/* ======================== FOC Data Structure ============================== */

/**
 * @brief FOC calculation intermediate data (one frame per PWM cycle)
 *
 * Data flow per 20kHz cycle:
 *   ADC -> i_a/i_b/i_c -> Clarke -> i_alpha/i_beta -> Park -> i_d/i_q
 *   -> Current PI -> v_d/v_q -> InvPark -> v_alpha/v_beta
 *   -> SVPWM -> dtc_a/dtc_b/dtc_c -> TIM1 CCRx
 */
typedef struct
{
    /* Bus voltage */
    float vbus;             /* Measured bus voltage [V] */
    float inv_vbus;         /* 1/vbus for normalization */

    /* Electrical angle */
    float theta;            /* Current electrical angle [rad] */
    float sin_val;          /* sin(theta) */
    float cos_val;          /* cos(theta) */

    /* Phase currents [A] */
    float i_a, i_b, i_c;

    /* Clarke transform output (alpha-beta stationary frame) */
    float i_alpha, i_beta;

    /* Park transform output (d-q rotating frame) */
    float i_d, i_q;

    /* Filtered d-q currents */
    float i_d_filt, i_q_filt;

    /* Bus current and power */
    float i_bus;
    float i_bus_filt;
    float power_filt;

    /* Current controller (PI) output voltages (d-q frame) */
    float v_d, v_q;

    /* Voltage setpoints (d-q frame) */
    float vd_set, vq_set;

    /* Current setpoints (d-q frame) */
    float id_set, iq_set;

    /* Inverse Park transform output (alpha-beta frame) */
    float v_alpha, v_beta;

    /* Three-phase voltages (after Inverse Clarke) */
    float v_a, v_b, v_c;

    /* Final PWM duty cycles [0.0, 1.0] */
    float dtc_a, dtc_b, dtc_c;

    /* SVPWM sector (1~6) */
    int sector;

    /* Current controller integrator accumulators */
    float current_ctrl_integral_d;
    float current_ctrl_integral_q;

    /* Current controller gains (auto-calculated) */
    float current_ctrl_p_gain;
    float current_ctrl_i_gain;

} FOC_DATA;

/* ======================== Global FOC Data Instance ========================= */
extern FOC_DATA foc_data;

/* ======================== Public API ====================================== */

/* --- PWM Control Functions --- */

/** Start PWM output on TIM1 CH1/CH2/CH3 + complementary channels */
void Foc_Pwm_Start(void);

/** Stop all PWM outputs */
void Foc_Pwm_Stop(void);

/** Set all duty cycles to 0% (low-side conduction for bootstrap pre-charge) */
void Foc_Pwm_LowSides(void);

/** Write computed duty cycles to TIM1 CCR registers */
void SetPwm(FOC_DATA *foc);

/* --- Trigonometric Functions --- */

/** Compute sin(theta) and cos(theta) for current electrical angle */
void Sin_Cos_Val(FOC_DATA *foc);

/* --- Core FOC Transforms --- */

/** Clarke transform: (a,b,c) -> (alpha, beta) */
void Clarke(FOC_DATA *foc);

/** Inverse Clarke transform: (alpha, beta) -> (a,b,c) */
void Inv_Clarke(FOC_DATA *foc);

/** Park transform: (alpha, beta) -> (d, q) using current sin/cos */
void Park(FOC_DATA *foc);

/** Inverse Park transform: (d, q) -> (alpha, beta) using current sin/cos */
void Inv_Park(FOC_DATA *foc);

/* --- SVPWM Modulation --- */

/**
 * @brief 7-segment SVPWM modulation with midpoint injection
 *        Converts v_alpha/v_beta to duty cycles dtc_a/b/c [0, 1].
 *        Includes bipolar-to-unipolar offset (0.5) for TIM1 center-aligned mode.
 */
void Svpwm_Midpoint(FOC_DATA *foc);

/* --- FOC Lifecycle --- */

/** Reset FOC integrators and state */
void FOC_reset(FOC_DATA *foc);

#endif /* BLDC_MOTOR_H */
