/**
 ******************************************************************************
 * @file    foc_motor.h
 * @author  milFOC Team
 * @brief   FOC Motor state machine and multi-loop cascade controller.
 *          Manages motor lifecycle: IDLE -> DETECTING -> RUNNING -> GUARD.
 *          Orchestrates current loop (20kHz), speed loop (1kHz), and
 *          position loop (100Hz) with proper cascade architecture.
 *
 * @note    Control loop execution frequencies:
 *          - Current loop:  20 kHz (ADC JEOC interrupt)
 *          - Speed loop:    1 kHz  (every 20th current loop)
 *          - Position loop: 100 Hz (every 10th speed loop)
 ******************************************************************************
 */

#ifndef FOC_MOTOR_H
#define FOC_MOTOR_H

#include "general_def.h"
#include "pid.h"
#include "bldc_motor.h"

/* ======================== Control Mode Enum ================================ */

/**
 * @brief Motor control modes (cascade loop configuration)
 */
typedef enum
{
    CONTROL_MODE_OPEN           = 0,    /* Speed open-loop (V/F control) */
    CONTROL_MODE_TORQUE         = 1,    /* Torque/current closed-loop */
    CONTROL_MODE_VELOCITY       = 2,    /* Velocity closed-loop */
    CONTROL_MODE_POSITION       = 3,    /* Position closed-loop */
    CONTROL_MODE_VELOCITY_RAMP  = 4,    /* Velocity ramp (limited accel) */
    CONTROL_MODE_POSITION_RAMP  = 5,    /* Position trapezoidal trajectory */
} CONTROL_MODE;

/* ======================== State Machine Enums ============================== */

/** Motor calibration sub-states */
typedef enum
{
    SUB_STATE_IDLE          = 0,
    CURRENT_CALIBRATING,        /* ADC offset calibration */
    RSLS_CALIBRATING,           /* Resistance/inductance identification */
    FLUX_CALIBRATING,           /* Flux linkage identification */
} SUB_STATE;

/** Calibration sequence states */
typedef enum
{
    CS_STATE_IDLE           = 0,
    CS_MOTOR_R_START,
    CS_MOTOR_R_LOOP,
    CS_MOTOR_R_END,
    CS_MOTOR_L_START,
    CS_MOTOR_L_LOOP,
    CS_MOTOR_L_END,
    CS_DIR_PP_START,
    CS_DIR_PP_LOOP,
    CS_DIR_PP_END,
    CS_ENCODER_START,
    CS_ENCODER_CW_LOOP,
    CS_ENCODER_CCW_LOOP,
    CS_ENCODER_END,
    CS_REPORT_OFFSET_LUT,
} CS_STATE;

/** Top-level motor states */
typedef enum
{
    STATE_MODE_IDLE     = 0,    /* Idle, transition to DETECTING */
    STATE_MODE_DETECTING,       /* Auto-calibration in progress */
    STATE_MODE_RUNNING,         /* Normal FOC operation */
    STATE_MODE_GUARD,           /* Fault protection, PWM disabled */
} STATE_MODE;

/** Fault/error states */
typedef enum
{
    FAULT_STATE_NORMAL          = 0,
    FAULT_STATE_OVER_CURRENT,       /* Over-current detected */
    FAULT_STATE_OVER_VOLTAGE,       /* Bus over-voltage */
    FAULT_STATE_UNDER_VOLTAGE,      /* Bus under-voltage */
    FAULT_STATE_OVER_TEMPERATURE,   /* Over-temperature */
    FAULT_STATE_SPEEDING,           /* Over-speed */
} FAULT_STATE;

/* ======================== Data Structures ================================== */

/** Forward declarations */
struct ENCODER_DATA;
struct CURRENT_DATA;

/** Component aggregation (pointers to sub-module data) */
typedef struct
{
    FOC_DATA *foc;                  /* FOC math data */
    struct ENCODER_DATA *encoder;   /* Encoder data */
    struct CURRENT_DATA *current;   /* ADC current data */
} MOTOR_COMPONENTS;

/** Motor physical parameters (identified via calibration) */
typedef struct
{
    float Rs;       /* Phase resistance [Ohm] */
    float Ls;       /* Phase inductance [H] */
    float flux;     /* Rotor flux linkage [Wb] */
} MOTOR_PARAMETERS;

/** Motor controller configuration and setpoints */
typedef struct
{
    /* Mechanical parameters */
    float inertia;              /* Rotor inertia [A/(turn/s^2)] */
    float torque_ramp_rate;     /* Torque ramp rate [Nm/s] */
    float vel_ramp_rate;        /* Velocity ramp rate [(turn/s)/s] */

    /* Trajectory planner parameters */
    float traj_vel;             /* Max trajectory velocity [turn/s] */
    float traj_accel;           /* Trajectory acceleration [(turn/s)/s] */
    float traj_decel;           /* Trajectory deceleration [(turn/s)/s] */

    /* Limits */
    float vel_limit;            /* Velocity limit [turn/s] */
    float torque_const;         /* Torque constant [Nm/A] */
    float torque_limit;         /* Torque limit [Nm] */
    float current_limit;        /* Current limit [A] */
    float voltage_limit;        /* Voltage limit [V] */

    /* Current loop gains (auto-calculated or manual) */
    float current_ctrl_p_gain;
    float current_ctrl_i_gain;
    int current_ctrl_bandwidth; /* [rad/s], range 100~2000 */

    /* User inputs */
    float input_position;
    float input_velocity;
    float input_torque;
    float input_current;

    /* Processed setpoints */
    float pos_setpoint;
    float vel_setpoint;
    float torque_setpoint;

    volatile bool input_updated;    /* Position input update flag */
} MOTOR_CONTROLLER;

/** Motor state aggregation */
typedef struct
{
    STATE_MODE State_Mode;
    CONTROL_MODE Control_Mode;
    SUB_STATE Sub_State;
    CS_STATE Cs_State;
    FAULT_STATE Fault_State;
} MOTOR_STATE;

/**
 * @brief Top-level motor data structure (singleton)
 *
 * Aggregates all motor-related data: components, state, parameters,
 * controller config, and four PID instances.
 */
typedef struct
{
    MOTOR_COMPONENTS components;
    MOTOR_STATE state;
    MOTOR_PARAMETERS parameters;
    MOTOR_CONTROLLER Controller;

    /* PID controllers for cascade loops */
    PidTypeDef IqPID;       /* Q-axis current PI */
    PidTypeDef IdPID;       /* D-axis current PI */
    PidTypeDef VelPID;      /* Velocity PI */
    PidTypeDef PosPID;      /* Position PID */

} MOTOR_DATA;

/* ======================== Global Motor Instance ============================ */
extern MOTOR_DATA motor_data;

/* ======================== Public API ====================================== */

/**
 * @brief  Initialize motor without calibration (use pre-calibrated params)
 */
void Init_Motor_No_Calib(MOTOR_DATA *motor);

/**
 * @brief  Initialize motor with full auto-calibration sequence
 */
void Init_Motor_Calib(MOTOR_DATA *motor);

/**
 * @brief  Read ADC1 injected group and convert to phase currents
 * @note   Called from ADC JEOC interrupt (20 kHz, highest priority)
 */
void GetMotorADC1PhaseCurrent(MOTOR_DATA *motor);

/**
 * @brief  Read temperature from NTC thermistor
 */
void TempResultTask(MOTOR_DATA *motor);

/**
 * @brief  Main FOC state machine task
 * @note   Called from ADC JEOC interrupt (20 kHz).
 *         Executes state machine, calibration sequences, and FOC control loops.
 */
void MotorStateTask(MOTOR_DATA *motor);

/**
 * @brief  Motor guard/protection task
 * @note   Called at lower frequency (e.g. 1 kHz).
 *         Checks voltage, temperature, over-speed, and triggers faults.
 */
void MotorGuardTask(MOTOR_DATA *motor);

#endif /* FOC_MOTOR_H */
