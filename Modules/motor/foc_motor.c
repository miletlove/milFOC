/**
 ******************************************************************************
 * @file    foc_motor.c
 * @author  milFOC Team
 * @brief   FOC Motor state machine implementation.
 *          Manages motor lifecycle, calibration sequences, and cascade
 *          control loop scheduling.
 *
 * @note    This is the "brain" of the FOC driver. It orchestrates:
 *          1. State transitions (IDLE -> DETECTING -> RUNNING -> GUARD)
 *          2. Control mode dispatch (Open/Torque/Velocity/Position)
 *          3. Multi-rate cascade loop execution
 *          4. Fault detection and protection
 ******************************************************************************
 */

#include "foc_motor.h"
#include "bsp_dwt.h"
#include "bsp_log.h"
#include "led.h"
#include "trap_traj.h"
#include "motor_adc.h"
#include "mt6816_encoder.h"

/* ======================== Global Motor Instance ============================ */

MOTOR_DATA motor_data = {
    .components = {
        .foc     = &foc_data,
        .encoder = &encoder_data,
        .current = &current_data,
    },
    .state = {
        .State_Mode   = STATE_MODE_IDLE,
        .Control_Mode = CONTROL_MODE_VELOCITY_RAMP,
        .Sub_State    = SUB_STATE_IDLE,
        .Cs_State     = CS_STATE_IDLE,
        .Fault_State  = FAULT_STATE_NORMAL,
    },
    .parameters = {
        .Rs   = MOTOR_RS,
        .Ls   = MOTOR_LS,
        .flux = MOTOR_FLUX,
    },
    .Controller = {
        .inertia                = MOTOR_INERTIA,
        .torque_ramp_rate       = MOTOR_CURRENT_RAMP_RATE,
        .vel_ramp_rate          = MOTOR_VEL_RAMP_RATE,
        .traj_vel               = MOTOR_TRAJ_VEL,
        .traj_accel             = MOTOR_TRAJ_ACCEL,
        .traj_decel             = MOTOR_TRAJ_DECEL,
        .torque_const           = MOTOR_TORQUE_CONST,
        .torque_limit           = MOTOR_TORQUE_LIMIT,
        .vel_limit              = MOTOR_VEL_LIMIT,
        .voltage_limit          = MOTOR_VOLTAGE_LIMIT,
        .current_limit          = MOTOR_CURRENT_LIMIT,
        .current_ctrl_p_gain    = MOTOR_CURRENT_CTRL_P_GAIN,
        .current_ctrl_i_gain    = MOTOR_CURRENT_CTRL_I_GAIN,
        .current_ctrl_bandwidth = MOTOR_CURRENT_CTRL_BANDWIDTH,
        .input_current          = MOTOR_INPUT_CURRENT,
        .input_torque           = MOTOR_INPUT_TORQUE,
        .input_velocity         = MOTOR_INPUT_VELOCITY,
        .input_position         = MOTOR_INPUT_POSITION,
        .input_updated          = true,
    },
    .IqPID = {
        .mode     = PID_POSITION,
        .Kp       = MOTOR_IQ_PID_KP,
        .Ki       = MOTOR_IQ_PID_KI,
        .Kd       = MOTOR_IQ_PID_KD,
        .max_out  = IQ_PID_MAX_OUT,
        .max_iout = IQ_PID_MAX_IOUT,
    },
    .IdPID = {
        .mode     = PID_POSITION,
        .Kp       = MOTOR_ID_PID_KP,
        .Ki       = MOTOR_ID_PID_KI,
        .Kd       = MOTOR_ID_PID_KD,
        .max_out  = ID_PID_MAX_OUT,
        .max_iout = ID_PID_MAX_IOUT,
    },
    .VelPID = {
        .mode     = PID_POSITION,
        .Kp       = MOTOR_VEL_PID_KP,
        .Ki       = MOTOR_VEL_PID_KI,
        .Kd       = MOTOR_VEL_PID_KD,
        .max_out  = VEL_PID_MAX_OUT,
        .max_iout = VEL_PID_MAX_IOUT,
    },
    .PosPID = {
        .mode     = PID_POSITION,
        .Kp       = MOTOR_POS_PID_KP,
        .Ki       = MOTOR_POS_PID_KI,
        .Kd       = MOTOR_POS_PID_KD,
        .max_out  = POS_PID_MAX_OUT,
        .max_iout = POS_PID_MAX_IOUT,
    },
};

/* Calibration data buffer (allocated on first calibration) */
static int *p_error_arr = NULL;

/* ======================== Internal Helpers ================================= */

/**
 * @brief  Update current loop PI gains from motor parameters
 *
 *         P_gain = Ls * bandwidth
 *         I_gain = Rs * bandwidth * Ts
 *
 *         Or auto-calculate from velocity limit:
 *         P_gain = Ls * vel_limit * pole_pairs * 2*PI
 *         I_gain = Rs * vel_limit * pole_pairs * 2*PI * Ts
 */
#define CURRENT_AUTO_CALIBRATION 1
static void FOC_update_current_gain(MOTOR_DATA *motor)
{
#if CURRENT_AUTO_CALIBRATION
    motor->Controller.current_ctrl_p_gain =
        motor->parameters.Ls * motor->Controller.vel_limit *
        motor->components.encoder->pole_pairs * M_2PI;
    motor->Controller.current_ctrl_i_gain =
        motor->parameters.Rs * motor->Controller.vel_limit *
        motor->components.encoder->pole_pairs * M_2PI * CURRENT_MEASURE_PERIOD;
#else
    motor->Controller.current_ctrl_p_gain =
        motor->parameters.Ls * motor->Controller.current_ctrl_bandwidth;
    motor->Controller.current_ctrl_i_gain =
        motor->parameters.Rs * motor->Controller.current_ctrl_bandwidth *
        CURRENT_MEASURE_PERIOD;
#endif
    motor->IdPID.Kp = motor->Controller.current_ctrl_p_gain;
    motor->IdPID.Ki = motor->Controller.current_ctrl_i_gain;
    motor->IqPID.Kp = motor->Controller.current_ctrl_p_gain;
    motor->IqPID.Ki = motor->Controller.current_ctrl_i_gain;
}

/**
 * @brief  Adjust PID output limits based on bus voltage
 */
static void SetPIDLimit(MOTOR_DATA *motor,
                        float current_max_out, float current_max_iout,
                        float vel_max_out, float vel_max_iout,
                        float pos_limit)
{
    motor->IdPID.max_out   = CURRENT_PID_MAX_OUT;
    motor->IdPID.max_iout  = CURRENT_PID_MAX_OUT;
    motor->IqPID.max_out   = current_max_out;
    motor->IqPID.max_iout  = current_max_iout;
    motor->VelPID.max_out  = vel_max_out;
    motor->VelPID.max_iout = vel_max_iout;
    motor->PosPID.max_out  = pos_limit;
    motor->PosPID.max_iout = pos_limit;
}

/**
 * @brief  Speed open-loop control (V/F)
 *         Advances electrical angle at a fixed rate, outputs constant voltage.
 */
static void OpenControlMode(MOTOR_DATA *motor, float target_velocity)
{
    float Ts = 0.001f;
    if (Ts <= 0 || Ts > 0.5f) Ts = 1e-3f;

    /* Advance angle at constant speed */
    motor->components.foc->theta =
        wrap_pm_pi(motor->components.foc->theta + target_velocity * Ts);

    /* Output fixed voltage vector in open-loop */
    Sin_Cos_Val(motor->components.foc);
    motor->components.foc->v_d = 0.0f;
    motor->components.foc->v_q = 0.5f;  /* 50% voltage */
    Inv_Park(motor->components.foc);
    Svpwm_Midpoint(motor->components.foc);
    SetPwm(motor->components.foc);
}

/* ======================== Public API ====================================== */

/**
 * @brief  Read ADC and convert to phase currents
 */
void GetMotorADC1PhaseCurrent(MOTOR_DATA *motor)
{
    /* Read ADC1 injected group registers directly for minimum latency */
    motor->components.foc->i_a =
        ((float)motor->components.current->hadc->Instance->JDR3 -
         motor->components.current->Ia_offset) * FAC_CURRENT;
    motor->components.foc->i_b =
        ((float)motor->components.current->hadc->Instance->JDR2 -
         motor->components.current->Ib_offset) * FAC_CURRENT;
    motor->components.foc->i_c =
        ((float)motor->components.current->hadc->Instance->JDR1 -
         motor->components.current->Ic_offset) * FAC_CURRENT;
    motor->components.foc->vbus =
        ((float)motor->components.current->hadc->Instance->JDR4 *
         VOLTAGE_TO_ADC_FACTOR);
}

/**
 * @brief  Read NTC temperature
 */
void TempResultTask(MOTOR_DATA *motor)
{
    /* TODO: Implement NTC thermistor temperature lookup */
    (void)motor;
}

/**
 * @brief  Initialize motor without calibration
 */
void Init_Motor_No_Calib(MOTOR_DATA *motor)
{
    motor->state.Sub_State  = SUB_STATE_IDLE;
    motor->state.Cs_State   = CS_STATE_IDLE;
    motor->state.State_Mode = STATE_MODE_RUNNING;
    FOC_update_current_gain(motor);
    LOGINFO("[FOC] Motor initialized (no calibration)");
}

/**
 * @brief  Initialize motor with full auto-calibration
 */
void Init_Motor_Calib(MOTOR_DATA *motor)
{
    if (p_error_arr == NULL)
    {
        p_error_arr = malloc(SAMPLES_PER_PPAIR * MOTOR_POLE_PAIRS_MAX * sizeof(int));
    }

    motor->components.encoder->calib_valid = false;
    motor->state.Sub_State                 = RSLS_CALIBRATING;
    motor->state.Cs_State                  = CS_MOTOR_R_START;
    LOGINFO("[FOC] Motor calibration started");
}

/**
 * @brief  Main FOC state machine task (called from JEOC interrupt @ 20kHz)
 *
 *         This is the highest-priority real-time task. It must complete
 *         within ~15us to meet 20kHz timing requirements.
 *
 *         Execution flow:
 *         1. Check state machine
 *         2. If RUNNING: execute FOC control based on Control_Mode
 *         3. If DETECTING: execute calibration sequence
 *         4. Update PWM outputs
 */
void MotorStateTask(MOTOR_DATA *motor)
{
    /* --- State: GUARD (fault protection) --- */
    if (motor->state.State_Mode == STATE_MODE_GUARD)
    {
        /* PWM outputs are disabled by hardware break; hold safe state */
        return;
    }

    /* --- State: DETECTING (auto-calibration) --- */
    if (motor->state.State_Mode == STATE_MODE_DETECTING)
    {
        /* TODO: Implement calibration sequence state machine
         * - CURRENT_CALIBRATING: sample ADC offsets
         * - RSLS_CALIBRATING: inject test voltage, measure R and L
         * - ENCODER_CALIBRATING: rotate motor, build LUT
         */
        return;
    }

    /* --- State: RUNNING (normal FOC operation) --- */
    if (motor->state.State_Mode != STATE_MODE_RUNNING) return;

    /* --- FOC Core Calculation --- */

    /* Step 1: Compute sin/cos for current electrical angle */
    Sin_Cos_Val(motor->components.foc);

    /* Step 2: Clarke transform (abc -> alpha-beta) */
    Clarke(motor->components.foc);

    /* Step 3: Park transform (alpha-beta -> d-q) */
    Park(motor->components.foc);

    /* Step 4: Current PI control based on control mode */

    /* Default: Id = 0 (MTPA for SPMSM), Iq = torque-producing current */
    float id_ref = 0.0f;
    float iq_ref = motor->Controller.torque_setpoint / motor->Controller.torque_const;

    /* Clamp current references */
    iq_ref = CLAMP(iq_ref, -motor->Controller.current_limit,
                   motor->Controller.current_limit);

    /* Run D-axis current PI */
    motor->components.foc->v_d = PID_Calc(&motor->IdPID,
        motor->components.foc->i_d, id_ref);

    /* Run Q-axis current PI */
    motor->components.foc->v_q = PID_Calc(&motor->IqPID,
        motor->components.foc->i_q, iq_ref);

    /* Step 5: Inverse Park transform (d-q -> alpha-beta) */
    Inv_Park(motor->components.foc);

    /* Step 6: SVPWM modulation (alpha-beta -> duty cycles) */
    Svpwm_Midpoint(motor->components.foc);

    /* Step 7: Write duty cycles to TIM1 CCR registers */
    SetPwm(motor->components.foc);
}

/**
 * @brief  Motor guard/protection task
 * @note   Run at lower frequency (e.g. 1 kHz) to check protection limits.
 *         Triggers STATE_MODE_GUARD on fault detection.
 */
void MotorGuardTask(MOTOR_DATA *motor)
{
    /* Over-voltage check */
    if (motor->components.foc->vbus > BATVEL_MAX_LIMIT)
    {
        motor->state.Fault_State = FAULT_STATE_OVER_VOLTAGE;
        motor->state.State_Mode  = STATE_MODE_GUARD;
        Foc_Pwm_Stop();
        LOGERROR("[FOC] Over-voltage fault! Vbus=%.2fV", motor->components.foc->vbus);
        return;
    }

    /* Under-voltage check */
    if (motor->components.foc->vbus < BATVEL_MIN_LIMIT)
    {
        motor->state.Fault_State = FAULT_STATE_UNDER_VOLTAGE;
        motor->state.State_Mode  = STATE_MODE_GUARD;
        Foc_Pwm_Stop();
        LOGERROR("[FOC] Under-voltage fault! Vbus=%.2fV", motor->components.foc->vbus);
        return;
    }

    /* Over-current check */
    if (ABS(motor->components.foc->i_q) > MOTOR_CURRENT_LIMIT * 1.5f)
    {
        motor->state.Fault_State = FAULT_STATE_OVER_CURRENT;
        motor->state.State_Mode  = STATE_MODE_GUARD;
        Foc_Pwm_Stop();
        LOGERROR("[FOC] Over-current fault! Iq=%.2fA", motor->components.foc->i_q);
        return;
    }
}
