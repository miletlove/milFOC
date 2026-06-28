/**
 ******************************************************************************
 * @file    bldc_motor.c
 * @author  milFOC Team
 * @brief   BLDC/PMSM FOC mathematical core implementation.
 *          Clarke, Park, Inverse Park, Inverse Clarke, and SVPWM transforms.
 *
 * @note    All functions operate on the FOC_DATA structure.
 *          sin/cos are currently computed via CMSIS-DSP arm_sin_f32/arm_cos_f32.
 *          TODO: Migrate to STM32G431 CORDIC hardware accelerator for
 *          zero-wait-state sin/cos computation.
 ******************************************************************************
 */

#include "bldc_motor.h"
#include "arm_math.h"

/* ======================== Global FOC Data Instance ========================= */
FOC_DATA foc_data = {
    .inv_vbus                = INVBATVEL,
    .vd_set                  = 0.0f,
    .vq_set                  = 0.0f,
    .id_set                  = 0.0f,
    .iq_set                  = MOTOR_INPUT_CURRENT,
    .i_d_filt                = 0.0f,
    .i_q_filt                = 0.0f,
    .i_bus                   = 0.0f,
    .i_bus_filt              = 0.0f,
    .power_filt              = 0.0f,
    .current_ctrl_integral_d = 0.0f,
    .current_ctrl_integral_q = 0.0f,
    .current_ctrl_p_gain     = MOTOR_CURRENT_CTRL_P_GAIN,
    .current_ctrl_i_gain     = MOTOR_CURRENT_CTRL_I_GAIN,
};

/* ======================== PWM Control ====================================== */

/* Direct CCR register write macros */
#define set_dtc_a(x) (TIM1->CCR1 = (uint16_t)(x))
#define set_dtc_b(x) (TIM1->CCR2 = (uint16_t)(x))
#define set_dtc_c(x) (TIM1->CCR3 = (uint16_t)(x))
#define PWM_ARR       (TIM1->ARR)

/**
 * @brief  Start all PWM channels and complementary outputs
 *         CH1/CH1N, CH2/CH2N, CH3/CH3N + CH4 for ADC trigger
 */
void Foc_Pwm_Start(void)
{
    /* Initialize duty cycles to 50% (safe state) */
    set_dtc_a(PWM_ARR >> 1);
    set_dtc_b(PWM_ARR >> 1);
    set_dtc_c(PWM_ARR >> 1);

    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_4);

    HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_1);
    HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_2);
    HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_3);
}

/**
 * @brief  Stop all PWM outputs (motor free-wheeling)
 */
void Foc_Pwm_Stop(void)
{
    HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_1);
    HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_2);
    HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_3);
    HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_4);

    HAL_TIMEx_PWMN_Stop(&htim1, TIM_CHANNEL_1);
    HAL_TIMEx_PWMN_Stop(&htim1, TIM_CHANNEL_2);
    HAL_TIMEx_PWMN_Stop(&htim1, TIM_CHANNEL_3);
}

/**
 * @brief  Set all duty cycles to 0% (low-side full conduction)
 * @note   Used for FD6288 bootstrap capacitor pre-charge.
 *         All three low-side MOSFETs conduct simultaneously to charge
 *         bootstrap capacitors before normal FOC operation.
 */
void Foc_Pwm_LowSides(void)
{
    set_dtc_a(0);
    set_dtc_b(0);
    set_dtc_c(0);

    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);
    HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_1);
    HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_2);
    HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_3);
}

/**
 * @brief  Write computed duty cycles to TIM1 CCR registers
 *
 * @note   With CCR preload enabled (OCxPE=1 in main.c), all 3 writes
 *         go to shadow registers and are transferred atomically to active
 *         registers at the next TIM1 UPDATE event (center-aligned: twice
 *         per PWM period). This prevents inter-phase duty cycle tearing.
 */
void SetPwm(FOC_DATA *foc)
{
    set_dtc_a((uint16_t)(foc->dtc_a * PWM_ARR));
    set_dtc_b((uint16_t)(foc->dtc_b * PWM_ARR));
    set_dtc_c((uint16_t)(foc->dtc_c * PWM_ARR));
}

/* ======================== Trigonometric Functions ========================== */

/**
 * @brief  Compute sin/cos of current theta
 * @note   Uses standard math library. TODO: Migrate to CORDIC accelerator.
 */
void Sin_Cos_Val(FOC_DATA *foc)
{
    foc->sin_val = arm_sin_f32(foc->theta);
    foc->cos_val = arm_cos_f32(foc->theta);
}

/* ======================== FOC Core Transforms ============================== */

/**
 * @brief  Clarke Transform: (a, b, c) -> (alpha, beta)
 *
 *         i_alpha = i_a
 *         i_beta  = (i_b - i_c) / sqrt(3)
 *
 *         Assumes balanced three-phase system (i_a + i_b + i_c = 0)
 */
void Clarke(FOC_DATA *foc)
{
    foc->i_alpha = foc->i_a;
    foc->i_beta  = (foc->i_b - foc->i_c) * ONE_BY_SQRT3;
}

/**
 * @brief  Inverse Clarke Transform: (alpha, beta) -> (a, b, c)
 *
 *         v_a = v_alpha
 *         v_b = -0.5*v_alpha + sqrt(3)/2 * v_beta
 *         v_c = -0.5*v_alpha - sqrt(3)/2 * v_beta
 */
void Inv_Clarke(FOC_DATA *foc)
{
    foc->v_a = foc->v_alpha;
    foc->v_b = -0.5f * foc->v_alpha + _SQRT3_2 * foc->v_beta;
    foc->v_c = -0.5f * foc->v_alpha - _SQRT3_2 * foc->v_beta;
}

/**
 * @brief  Park Transform: (alpha, beta) -> (d, q)
 *
 *         i_d =  i_alpha * cos(theta) + i_beta * sin(theta)
 *         i_q = -i_alpha * sin(theta) + i_beta * cos(theta)
 */
void Park(FOC_DATA *foc)
{
    foc->i_d =  foc->i_alpha * foc->cos_val + foc->i_beta * foc->sin_val;
    foc->i_q = -foc->i_alpha * foc->sin_val + foc->i_beta * foc->cos_val;
}

/**
 * @brief  Inverse Park Transform: (d, q) -> (alpha, beta)
 *
 *         v_alpha = v_d * cos(theta) - v_q * sin(theta)
 *         v_beta  = v_d * sin(theta) + v_q * cos(theta)
 */
void Inv_Park(FOC_DATA *foc)
{
    foc->v_alpha = foc->v_d * foc->cos_val - foc->v_q * foc->sin_val;
    foc->v_beta  = foc->v_d * foc->sin_val + foc->v_q * foc->cos_val;
}

/* ======================== SVPWM Modulation ================================= */

/**
 * @brief  7-segment SVPWM with midpoint injection
 *
 *         Converts alpha-beta voltages to three-phase PWM duty cycles.
 *         Steps:
 *         1. Normalize v_alpha/v_beta by bus voltage
 *         2. Inverse Clarke to get three-phase voltages
 *         3. Compute common-mode injection (midpoint)
 *         4. Apply bipolar-to-unipolar offset (+0.5) for center-aligned PWM
 *         5. Clamp to [0, 1]
 */
void Svpwm_Midpoint(FOC_DATA *foc)
{
    /* Normalize by bus voltage */
    foc->v_alpha = foc->inv_vbus * foc->v_alpha;
    foc->v_beta  = foc->inv_vbus * foc->v_beta;

    /* Inverse Clarke to get three-phase voltages */
    float va = foc->v_alpha;
    float vb = -0.5f * foc->v_alpha + _SQRT3_2 * foc->v_beta;
    float vc = -0.5f * foc->v_alpha - _SQRT3_2 * foc->v_beta;

    /* Compute common-mode injection (midpoint of min/max) */
    float vmax = max(max(va, vb), vc);
    float vmin = min(min(va, vb), vc);
    float vcom = (vmax + vmin) * 0.5f;

    /* Compute duty cycles with bipolar-to-unipolar offset (0.5) */
    foc->dtc_a = CLAMP((vcom - va) + 0.5f, 0.0f, 1.0f);
    foc->dtc_b = CLAMP((vcom - vb) + 0.5f, 0.0f, 1.0f);
    foc->dtc_c = CLAMP((vcom - vc) + 0.5f, 0.0f, 1.0f);
}

/**
 * @brief  7-segment SVPWM sector-based modulation (FalconFoc compatible)
 *
 *         Determines the sector from alpha-beta voltages, then computes
 *         vector timings and converts to center-aligned PWM duty cycles.
 *
 *         This is the primary SVPWM method used in FalconFoc.
 *         Svpwm_Midpoint is an alternative midpoint injection method.
 */
void Svpwm_Sector(FOC_DATA *foc)
{
    float ta = 0.0f, tb = 0.0f, tc = 0.0f;
    float k  = (TS * _SQRT3) * foc->inv_vbus;
    float va = foc->v_beta;
    float vb = (_SQRT3 * foc->v_alpha - foc->v_beta) * 0.5f;
    float vc = (-_SQRT3 * foc->v_alpha - foc->v_beta) * 0.5f;
    int a = (va > 0.0f) ? 1 : 0;
    int b = (vb > 0.0f) ? 1 : 0;
    int c = (vc > 0.0f) ? 1 : 0;
    int sextant = (c << 2) + (b << 1) + a;

    switch (sextant)
    {
    case 3: /* Sector 3 */
    {
        float t4 = k * vb;
        float t6 = k * va;
        float t0 = (TS - t4 - t6) * 0.5f;
        ta = t4 + t6 + t0;
        tb = t6 + t0;
        tc = t0;
    }
    break;
    case 1: /* Sector 1 */
    {
        float t6 = -k * vc;
        float t2 = -k * vb;
        float t0 = (TS - t2 - t6) * 0.5f;
        ta = t6 + t0;
        tb = t2 + t6 + t0;
        tc = t0;
    }
    break;
    case 5: /* Sector 5 */
    {
        float t2 = k * va;
        float t3 = k * vc;
        float t0 = (TS - t2 - t3) * 0.5f;
        ta = t0;
        tb = t2 + t3 + t0;
        tc = t3 + t0;
    }
    break;
    case 4: /* Sector 4 */
    {
        float t1 = -k * va;
        float t3 = -k * vb;
        float t0 = (TS - t1 - t3) * 0.5f;
        ta = t0;
        tb = t3 + t0;
        tc = t1 + t3 + t0;
    }
    break;
    case 6: /* Sector 6 */
    {
        float t1 = k * vc;
        float t5 = k * vb;
        float t0 = (TS - t1 - t5) * 0.5f;
        ta = t5 + t0;
        tb = t0;
        tc = t1 + t5 + t0;
    }
    break;
    case 2: /* Sector 2 */
    {
        float t4 = -k * vc;
        float t5 = -k * va;
        float t0 = (TS - t4 - t5) * 0.5f;
        ta = t4 + t5 + t0;
        tb = t0;
        tc = t5 + t0;
    }
    break;
    default:
        break;
    }

    /* Convert vector timings to duty cycles [0, 1]
     * Note: FalconFoc uses inverted logic: dtc = 1.0 - t_high
     * where t_high is the high-side conduction time fraction.
     * This matches TIM1 PWM1 mode where CNT < CCR → output HIGH. */
    foc->dtc_a = CLAMP(1.0f - ta, 0.0f, 1.0f);
    foc->dtc_b = CLAMP(1.0f - tb, 0.0f, 1.0f);
    foc->dtc_c = CLAMP(1.0f - tc, 0.0f, 1.0f);
}

/* ======================== FOC Lifecycle ==================================== */

/**
 * @brief  Reset FOC integrators and state (called on fault or mode change)
 */
void FOC_reset(FOC_DATA *foc)
{
    foc->i_d                 = 0.0f;
    foc->i_q                 = 0.0f;
    foc->current_ctrl_p_gain = 0.0f;
    foc->current_ctrl_i_gain = 0.0f;
}
