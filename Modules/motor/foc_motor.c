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
#include "bsp_adc.h"
#include "bsp_log.h"
#include "led.h"
#include "trap_traj.h"
#include "motor_adc.h"
#include "mt6816_encoder.h"

/* ==================== Open-Loop Demo Parameters ========================= */
/*
 * CONTROL_MODE_OPEN: fixed-speed rotating voltage vector.
 * Speed is electrical (mech speed = elec_speed / pole_pairs).
 * Adjust DEMO_VOLTAGE to control current (start low, increase if motor
 * doesn't move; too high may over-current).
 */
#define DEMO_SPEED_RPS   2.0f   /* Electrical revs per second */
#define DEMO_VOLTAGE     0.35f  /* Voltage magnitude [0~1] — 0.35×24=8.4V @24V */

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
 *         State transitions:
 *           IDLE → DETECTING(CURRENT_CALIBRATING, 0.1s) → RUNNING(OPEN_LOOP)
 *
 *         In open-loop: electrical angle advances at fixed speed,
 *         constant voltage vector applied → motor rotates.
 */
void MotorStateTask(MOTOR_DATA *motor)
{
    /* ================================================================
     * STATE: IDLE — auto-transition to DETECTING
     * ================================================================ */
    if (motor->state.State_Mode == STATE_MODE_IDLE)
    {
        /* Clear all PID integrators */
        PID_Clear(&motor->IqPID);
        PID_Clear(&motor->IdPID);
        PID_Clear(&motor->VelPID);
        PID_Clear(&motor->PosPID);

        /* Enter detection state → run current offset calibration */
        motor->state.State_Mode = STATE_MODE_DETECTING;
        motor->state.Sub_State  = CURRENT_CALIBRATING;
        motor->state.Control_Mode = CONTROL_MODE_OPEN;

        LOGINFO("[FOC] IDLE → DETECTING (current offset calibration)");
        return;
    }

    /* ================================================================
     * STATE: GUARD (fault) — stop PWM, hold
     * ================================================================ */
    if (motor->state.State_Mode == STATE_MODE_GUARD)
    {
        return;
    }

    /* ================================================================
     * STATE: DETECTING — run current offset calibration
     *          Collect ADC raw samples for 0.1s → compute offsets
     *          → transition to RUNNING
     * ================================================================ */
    if (motor->state.State_Mode == STATE_MODE_DETECTING)
    {
        static uint32_t calib_cnt = 0;
        static uint64_t sum_a = 0, sum_b = 0, sum_c = 0;

        /* Accumulate ADC raw values */
        sum_a += motor->components.current->hadc->Instance->JDR3;
        sum_b += motor->components.current->hadc->Instance->JDR2;
        sum_c += motor->components.current->hadc->Instance->JDR1;
        calib_cnt++;

        /* Use 50% duty as safe output during calibration */
        FOC_DATA *f = motor->components.foc;
        f->dtc_a = 0.50f;
        f->dtc_b = 0.50f;
        f->dtc_c = 0.50f;
        SetPwm(f);

        /* Init VOFA display channels (zero during calibration) */
        f->v_a = f->v_b = f->v_c = 0.0f;
        f->i_alpha = f->i_beta = 0.0f;
        f->i_d = f->i_q = 0.0f;

        /* After ~0.1s (2000 samples @ 20kHz), compute offsets */
        if (calib_cnt >= 2000)
        {
            motor->components.current->Ia_offset =
                (float)sum_a / (float)calib_cnt;
            motor->components.current->Ib_offset =
                (float)sum_b / (float)calib_cnt;
            motor->components.current->Ic_offset =
                (float)sum_c / (float)calib_cnt;

            /* Update current PI gains from motor parameters */
            FOC_update_current_gain(motor);

            /* Transition to RUNNING */
            motor->state.State_Mode = STATE_MODE_RUNNING;
            calib_cnt = 0;
            sum_a = sum_b = sum_c = 0;

            LOGINFO("[FOC] Calib done → RUNNING (open-loop, %.1f rps elec)",
                    DEMO_SPEED_RPS);
        }
        return;
    }

    /* ================================================================
     * STATE: RUNNING — open-loop FOC rotation
     *          Advance electrical angle at constant speed,
     *          apply fixed Vd=0, Vq=voltage → InvPark → SVPWM → PWM
     * ================================================================ */
    if (motor->state.State_Mode != STATE_MODE_RUNNING) return;

    FOC_DATA *f = motor->components.foc;

    /* ---- Open-loop angle advance ---- */
    /* Ts = 1/20000 = 50us; theta += omega_elec * Ts */
    float omega_elec = DEMO_SPEED_RPS * M_2PI;  /* rad/s electrical */
    f->theta += omega_elec * CURRENT_MEASURE_PERIOD;
    f->theta = wrap_pm_pi(f->theta);            /* wrap to [-π, π] */

    /* ---- Read currents for VOFA display ---- */
    /* (offsets already calibrated, so i_a/i_b/i_c ≈ 0 at idle) */
    /* Already done in motor_task.c ISR before this call */

    /* ---- Compute sin/cos for display + transforms ---- */
    Sin_Cos_Val(f);

    /* ---- Clarke + Park (for VOFA display only) ---- */
    Clarke(f);
    Park(f);

    /* ---- Open-loop voltage command ---- */
    f->v_d = 0.0f;
    f->v_q = DEMO_VOLTAGE;   /* 0.15 = 15% bus voltage */

    /* ---- Inverse Park: dq → αβ ---- */
    Inv_Park(f);

    /* ---- Reconstruct phase voltages for VOFA ---- */
    f->v_a = f->v_alpha;
    f->v_b = -0.5f * f->v_alpha + _SQRT3_2 * f->v_beta;
    f->v_c = -0.5f * f->v_alpha - _SQRT3_2 * f->v_beta;

    /* ---- SVPWM: αβ → duty cycles ---- */
    Svpwm_Sector(f);

    /* ---- Write to PWM ---- */
    SetPwm(f);
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
