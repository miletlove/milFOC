/**
 ******************************************************************************
 * @file    bringup_task.c
 * @author  milFOC Team
 * @brief   FOC Bring-Up State Machine implementation.
 *
 *          Staged motor bring-up following industrial standard (TI/ST/Infineon):
 *          IDLE → PWM_TEST → LOCK → OPEN_LOOP → CURRENT → SPEED
 *
 *          Telemetry @ 100Hz via USB CDC, commands via USB CDC serial.
 *          All protection limits are enforced at every FOC step (20kHz).
 ******************************************************************************
 */

#include "bringup_task.h"
#include "bsp_dwt.h"
#include "bsp_adc.h"
#include "adc.h"
#include "vofa.h"
#include "usbd_cdc_if.h"
#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include <math.h>

/* ======================== Global Instance ================================== */

BringUp_t bringup;

/* ======================== Internal Helpers ================================= */

/* ─── ADC Reading ───────────────────────────────────────────────────── */

/**
 * @brief  Read ADC1 injected group registers directly
 *
 *         Channel mapping (CubeMX configured):
 *           JDR1 = CUR_A (PA0),  JDR2 = CUR_B (PA1),
 *           JDR3 = CUR_C (PA2),  JDR4 = VBUS  (PB1)
 *
 * @note   ADC1 injected group is triggered by TIM1 CH4 at PWM center.
 *         HAL_ADCEx_InjectedStart must have been called first.
 */
static void BringUp_ReadADC(BringUp_ADC_t *adc)
{
    if (adc == NULL) return;

    ADC_TypeDef *adc_reg = hadc1.Instance;
    if (adc_reg == NULL) return;

    /* Direct register read — minimum latency */
    adc->i_a_raw   = (uint16_t)adc_reg->JDR1;
    adc->i_b_raw   = (uint16_t)adc_reg->JDR2;
    adc->i_c_raw   = (uint16_t)adc_reg->JDR3;
    adc->vbus_raw  = (uint16_t)adc_reg->JDR4;

    /* Sanity check: ADC should be live (not stuck at 0 or 4095).
     * Stuck values indicate ADC trigger not firing or channel fault. */
    if (adc->i_a_raw < 100u || adc->i_a_raw > 4000u ||
        adc->i_b_raw < 100u || adc->i_b_raw > 4000u ||
        adc->i_c_raw < 100u || adc->i_c_raw > 4000u)
    {
        return;  /* Skip this sample — ADC may not be ready */
    }

    /* Raw → physical conversion */
    adc->i_a = ((float)adc->i_a_raw - adc->Ia_offset) * FAC_CURRENT;
    adc->i_b = ((float)adc->i_b_raw - adc->Ib_offset) * FAC_CURRENT;
    adc->i_c = ((float)adc->i_c_raw - adc->Ic_offset) * FAC_CURRENT;

    /* IIR low-pass filter — attenuates PWM switching noise.
     * α = BRINGUP_ADC_LPF_ALPHA (default 0.05 → ~160Hz cutoff @ 20kHz) */
    const float alpha = BRINGUP_ADC_LPF_ALPHA;
    if (adc->filter_init)
    {
        adc->i_a_filt += alpha * (adc->i_a - adc->i_a_filt);
        adc->i_b_filt += alpha * (adc->i_b - adc->i_b_filt);
        adc->i_c_filt += alpha * (adc->i_c - adc->i_c_filt);
    }
    else
    {
        adc->i_a_filt = adc->i_a;
        adc->i_b_filt = adc->i_b;
        adc->i_c_filt = adc->i_c;
        adc->filter_init = 1u;
    }

    adc->vbus = (float)adc->vbus_raw * VOLTAGE_TO_ADC_FACTOR;
}

/* ─── ADC Offset Calibration (non-blocking, first ~0.1s) ─────────────── */

/**
 * @brief  Non-blocking ADC offset calibration — call once per FOC step
 *
 *         Accumulates JDR readings over BRINGUP_ADC_CALIB_SAMPLES steps.
 *         After completion, stores Ia/Ib/Ic_offset and sets calib_done=1.
 *
 *         Each call takes <2µs (3 register reads + 3 adds).
 */
static void BringUp_CalibrateADC_Step(BringUp_ADC_t *adc)
{
    if (adc == NULL || adc->calib_done) return;

    ADC_TypeDef *adc_reg = hadc1.Instance;
    if (adc_reg == NULL) return;

    uint16_t raw_a = (uint16_t)adc_reg->JDR1;
    uint16_t raw_b = (uint16_t)adc_reg->JDR2;
    uint16_t raw_c = (uint16_t)adc_reg->JDR3;

    /* Sanity check: reject stuck-at-rail readings */
    if (raw_a < 100u || raw_a > 4000u) return;

    adc->calib_sum_a += (uint64_t)raw_a;
    adc->calib_sum_b += (uint64_t)raw_b;
    adc->calib_sum_c += (uint64_t)raw_c;
    adc->calib_cnt++;

    if (adc->calib_cnt >= BRINGUP_ADC_CALIB_SAMPLES)
    {
        adc->Ia_offset = (float)adc->calib_sum_a / (float)adc->calib_cnt;
        adc->Ib_offset = (float)adc->calib_sum_b / (float)adc->calib_cnt;
        adc->Ic_offset = (float)adc->calib_sum_c / (float)adc->calib_cnt;
        adc->calib_done = 1u;
    }
}

/* ─── Protection Checks ─────────────────────────────────────────────── */

/**
 * @brief  Run all protection checks
 * @return true if any fault is active
 */
static bool BringUp_ProtectionCheck(BringUp_t *bu)
{
    bool fault = false;

    /* Over-voltage */
    if (bu->adc.vbus > BRINGUP_VBUS_MAX)
    {
        bu->fault_ov = true;
        fault = true;
    }
    /* Under-voltage */
    if (bu->adc.vbus < BRINGUP_VBUS_MIN && bu->adc.vbus > 1.0f)
    {
        /* Only flag if Vbus is non-zero (sensor connected) */
        bu->fault_uv = true;
        fault = true;
    }
    /* Over-current (instantaneous, any phase) */
    if (fabsf(bu->adc.i_a_filt) > BRINGUP_OVER_CURRENT ||
        fabsf(bu->adc.i_b_filt) > BRINGUP_OVER_CURRENT ||
        fabsf(bu->adc.i_c_filt) > BRINGUP_OVER_CURRENT)
    {
        bu->fault_oc = true;
        fault = true;
    }
    /* ADC zero-sum check (Kirchhoff: Ia+Ib+Ic ≈ 0) */
    float i_sum = bu->adc.i_a_filt + bu->adc.i_b_filt + bu->adc.i_c_filt;
    if (fabsf(i_sum) > BRINGUP_ADC_SUM_THRESH)
    {
        bu->fault_adc_sum = true;
        /* Not a hard fault, but log it */
    }

    if (fault && bu->pwm_active)
    {
        BringUp_EmergencyStop();
    }

    return fault;
}

/* ─── Open-Loop Controller ──────────────────────────────────────────── */

/**
 * @brief  Update open-loop angle θ and ramped speed/Uq
 *
 *         θ += ω * Ts  (ramped speed)
 *         Uq ramps toward target
 */
static void OpenLoop_Update(OpenLoop_Ctrl_t *ol, float Ts)
{
    if (ol == NULL) return;

    /* Ramp speed toward target */
    float ramp_step = BRINGUP_SPEED_RAMP * Ts;
    float speed_err = ol->speed_elec_hz - ol->speed_current_hz;
    if (fabsf(speed_err) <= ramp_step)
        ol->speed_current_hz = ol->speed_elec_hz;
    else
        ol->speed_current_hz += (speed_err > 0.0f) ? ramp_step : -ramp_step;

    /* Ramp Uq toward target */
    float uq_step = BRINGUP_UQ_RAMP * Ts;
    float uq_err = ol->uq_target - ol->uq_current;
    if (fabsf(uq_err) <= uq_step)
        ol->uq_current = ol->uq_target;
    else
        ol->uq_current += (uq_err > 0.0f) ? uq_step : -uq_step;

    /* Advance electrical angle */
    ol->speed_elec_radps = ol->speed_current_hz * M_2PI;
    ol->theta_elec += ol->speed_elec_radps * Ts;

    /* Wrap to [0, 2π) */
    if (ol->theta_elec >= M_2PI)
        ol->theta_elec -= M_2PI;
    else if (ol->theta_elec < 0.0f)
        ol->theta_elec += M_2PI;

    /* Compute sin/cos (using math.h — TODO: CORDIC) */
    ol->sin_val = sinf(ol->theta_elec);
    ol->cos_val = cosf(ol->theta_elec);
}

/* ─── SVPWM Duty Cycle Computation ──────────────────────────────────── */

/**
 * @brief  Compute SVPWM duty cycles from Vd/Vq and theta
 *
 *         Standard SVPWM (7-segment) with zero-sequence injection.
 *         dtc_x ∈ [0, 1] for center-aligned PWM.
 */
static void BringUp_SVPWM(FOC_DATA *foc, float vd, float vq,
                          float sin_val, float cos_val)
{
    if (foc == NULL) return;

    /* Inverse Park: dq → αβ */
    float v_alpha = vd * cos_val - vq * sin_val;
    float v_beta  = vd * sin_val + vq * cos_val;

    /* Normalize to [0,1] domain */
    float inv_vbus = foc->inv_vbus;
    float v_alpha_norm = inv_vbus * v_alpha;
    float v_beta_norm  = inv_vbus * v_beta;

    /* Inverse Clarke: αβ → abc */
    float va_u = v_alpha_norm;
    float vb_u = -0.5f * v_alpha_norm + _SQRT3_2 * v_beta_norm;
    float vc_u = -0.5f * v_alpha_norm - _SQRT3_2 * v_beta_norm;

    /* Zero-sequence injection (min-max method) */
    float vmax_u = max(max(va_u, vb_u), vc_u);
    float vmin_u = min(min(va_u, vb_u), vc_u);
    float vcom_u = (vmax_u + vmin_u) * 0.5f;

    /* Duty cycles [0, 1] */
    foc->dtc_a = CLAMP(0.5f + va_u - vcom_u, 0.0f, 1.0f);
    foc->dtc_b = CLAMP(0.5f + vb_u - vcom_u, 0.0f, 1.0f);
    foc->dtc_c = CLAMP(0.5f + vc_u - vcom_u, 0.0f, 1.0f);

    /* Store intermediate values for debug */
    foc->v_alpha = v_alpha;
    foc->v_beta  = v_beta;
    foc->v_a = v_alpha;
    foc->v_b = -0.5f * v_alpha + _SQRT3_2 * v_beta;
    foc->v_c = -0.5f * v_alpha - _SQRT3_2 * v_beta;
    foc->v_d = vd;
    foc->v_q = vq;
    foc->theta   = atan2f(sin_val, cos_val);
    foc->sin_val = sin_val;
    foc->cos_val = cos_val;
}

/* ─── USB Command Parser ────────────────────────────────────────────── */

/**
 * @brief  Parse and execute a single command line
 */
static void BringUp_ParseCmd(BringUp_t *bu, const char *cmd)
{
    if (bu == NULL || cmd == NULL) return;

    char line[64];
    strncpy(line, cmd, sizeof(line) - 1);
    line[sizeof(line) - 1] = '\0';

    /* Trim trailing \r\n */
    char *p = line;
    while (*p && (*p == ' ' || *p == '\t')) p++;
    char *end = p + strlen(p) - 1;
    while (end > p && (*end == '\r' || *end == '\n' || *end == ' '))
        *end-- = '\0';

    /* ── mode <idle|pwm|lock|open> ── */
    if (strncmp(p, "mode ", 5) == 0)
    {
        p += 5;
        if (strcmp(p, "idle") == 0)
        {
            bu->prev_mode = bu->mode;
            bu->mode = FOC_MODE_IDLE;
            bu->pwm_active = false;
        }
        else if (strcmp(p, "pwm") == 0)
        {
            bu->prev_mode = bu->mode;
            bu->mode = FOC_MODE_PWM_TEST;
            bu->pwm_active = true;
        }
        else if (strcmp(p, "lock") == 0)
        {
            bu->prev_mode = bu->mode;
            bu->mode = FOC_MODE_LOCK;
            bu->ol.theta_elec = 0.0f;
            bu->ol.sin_val = 0.0f;
            bu->ol.cos_val = 1.0f;
            bu->ol.uq_target = BRINGUP_DEFAULT_UQ;
            bu->ol.uq_current = 0.0f;  /* Ramp from 0 */
            bu->pwm_active = true;
        }
        else if (strcmp(p, "open") == 0)
        {
            bu->prev_mode = bu->mode;
            bu->mode = FOC_MODE_OPEN_LOOP;
            bu->ol.speed_elec_hz = BRINGUP_DEFAULT_SPEED;
            bu->ol.speed_current_hz = 0.0f;  /* Ramp from 0 */
            bu->ol.uq_target = BRINGUP_DEFAULT_UQ;
            bu->ol.uq_current = 0.0f;
            bu->pwm_active = true;
        }
    }
    /* ── set uq <volts> ── */
    else if (strncmp(p, "set uq ", 7) == 0)
    {
        float val = (float)atof(p + 7);
        if (val < 0.0f) val = 0.0f;
        if (bu->mode == FOC_MODE_OPEN_LOOP)
        {
            if (val > BRINGUP_MAX_UQ_STAGE3) val = BRINGUP_MAX_UQ_STAGE3;
        }
        else
        {
            if (val > BRINGUP_MAX_UQ_STAGE0) val = BRINGUP_MAX_UQ_STAGE0;
        }
        bu->ol.uq_target = val;
    }
    /* ── set speed <Hz> ── */
    else if (strncmp(p, "set speed ", 10) == 0)
    {
        float val = (float)atof(p + 10);
        if (val < 0.0f) val = 0.0f;
        if (val > 200.0f) val = 200.0f;  /* Reasonable upper limit */
        bu->ol.speed_elec_hz = val;
    }
    /* ── set angle <deg> ── */
    else if (strncmp(p, "set angle ", 10) == 0)
    {
        float deg = (float)atof(p + 10);
        float rad = deg * (float)(M_PI / 180.0);
        bu->ol.theta_elec = fmodf(rad, M_2PI);
        if (bu->ol.theta_elec < 0.0f) bu->ol.theta_elec += M_2PI;
        bu->ol.sin_val = sinf(bu->ol.theta_elec);
        bu->ol.cos_val = cosf(bu->ol.theta_elec);
    }
    /* ── status ── */
    else if (strcmp(p, "status") == 0)
    {
        /* Use FireWater protocol for status data — avoids stack-heavy snprintf.
         * VOFA+ can display this single-frame burst as current values. */
        float st[10];
        st[0] = bu->adc.vbus;
        st[1] = bu->adc.i_a_filt;
        st[2] = bu->adc.i_b_filt;
        st[3] = bu->adc.i_c_filt;
        st[4] = bu->adc.i_a_filt + bu->adc.i_b_filt + bu->adc.i_c_filt;
        st[5] = (float)bu->mode;
        st[6] = bu->ol.uq_current;
        st[7] = bu->ol.speed_current_hz;
        st[8] = bu->ol.theta_elec * 57.29578f;
        st[9] = (float)bu->run_time_ms;
        vofa_firewater_send(st, 10);

        /* Plain text status line (no %f — safe on stack) */
        const char *mode_names[] = {"IDLE","PWM_TEST","LOCK","OPEN_LOOP","CURRENT","SPEED"};
        char line[64];
        int n = 0;
        n += snprintf(line + n, sizeof(line) - n, "Mode:%s ", mode_names[bu->mode]);
        n += snprintf(line + n, sizeof(line) - n, "OV:%d UV:%d OC:%d SUM:%d\r\n",
                      bu->fault_ov, bu->fault_uv, bu->fault_oc, bu->fault_adc_sum);
        CDC_Transmit_FS((uint8_t *)line, (uint16_t)n);
    }
    /* ── help ── */
    else if (strcmp(p, "help") == 0)
    {
        const char *help_msg =
            "\r\n=== milFOC Bring-Up Commands ===\r\n"
            "mode idle        — PWM off, ADC monitoring\r\n"
            "mode pwm         — 50%% fixed duty (scope)\r\n"
            "mode lock        — Rotor alignment\r\n"
            "mode open        — Open-loop rotation\r\n"
            "set uq <V>       — Set Uq voltage (0~2V safe)\r\n"
            "set speed <Hz>   — Set elec speed (0.5~5Hz)\r\n"
            "set angle <deg>  — Set lock angle\r\n"
            "status           — Print system status\r\n"
            "help             — This message\r\n";
        CDC_Transmit_FS((uint8_t *)help_msg, (uint16_t)strlen(help_msg));
    }
}

/* ======================== Public API ======================================= */

/* ─── Init ──────────────────────────────────────────────────────────── */

void BringUp_Init(void)
{
    BringUp_t *bu = &bringup;

    /* Zero-initialize the entire struct */
    memset(bu, 0, sizeof(BringUp_t));

    /* ── Mode: selected by BRINGUP_STAGE compile-time macro ── */
#if BRINGUP_STAGE == 0
    bu->mode      = FOC_MODE_IDLE;
    bu->pwm_active = false;
#elif BRINGUP_STAGE == 1
    bu->mode      = FOC_MODE_PWM_TEST;
    bu->pwm_active = true;
#elif BRINGUP_STAGE == 2
    bu->mode      = FOC_MODE_LOCK;
    bu->pwm_active = true;
#elif BRINGUP_STAGE == 3
    bu->mode      = FOC_MODE_OPEN_LOOP;
    bu->pwm_active = true;
#else
    bu->mode      = FOC_MODE_IDLE;
    bu->pwm_active = false;
#endif
    bu->prev_mode = bu->mode;

    /* ── ADC offsets: start with theoretical 1.65V = ~2048.
     *     Will be refined by automatic calibration over first ~0.1s.
     *     Calibration runs non-blocking in BringUp_Step(). ── */
    bu->adc.Ia_offset  = 2048.0f;
    bu->adc.Ib_offset  = 2048.0f;
    bu->adc.Ic_offset  = 2048.0f;
    bu->adc.filter_init = 0u;
    bu->adc.calib_done  = 0u;
    bu->adc.calib_cnt   = 0u;
    bu->adc.calib_sum_a = 0u;
    bu->adc.calib_sum_b = 0u;
    bu->adc.calib_sum_c = 0u;

    /* ── FOC data ── */
    bu->foc.vbus     = BRINGUP_VBUS;
    bu->foc.inv_vbus = 1.0f / BRINGUP_VBUS;
    bu->foc.theta    = 0.0f;
    bu->foc.sin_val  = 0.0f;
    bu->foc.cos_val  = 1.0f;
    bu->foc.dtc_a    = 0.50f;
    bu->foc.dtc_b    = 0.50f;
    bu->foc.dtc_c    = 0.50f;

    /* ── Open-loop controller defaults ── */
    bu->ol.pole_pairs       = BRINGUP_POLE_PAIRS;
    bu->ol.theta_elec       = BRINGUP_INIT_ANGLE * ((float)M_PI / 180.0f);
    bu->ol.speed_elec_hz    = BRINGUP_INIT_SPEED;
    bu->ol.sin_val          = sinf(bu->ol.theta_elec);
    bu->ol.cos_val          = cosf(bu->ol.theta_elec);
    bu->ol.uq_target        = BRINGUP_INIT_UQ;

#if BRINGUP_SKIP_RAMP
    /* Instant start — no ramp (for bring-up testing) */
    bu->ol.speed_current_hz = BRINGUP_INIT_SPEED;
#if BRINGUP_VF_MODE
    bu->ol.uq_current       = 0.0f;  /* V/F mode: ramp up from 0 toward curve */
#else
    bu->ol.uq_current       = BRINGUP_INIT_UQ;
#endif
#else
    /* Soft start — ramp from 0 (for production) */
    bu->ol.speed_current_hz = 0.0f;
    bu->ol.uq_current       = 0.0f;
#endif

    /* ── Start ADC1 injected group (polled, TIM1-triggered) ── */
    if (HAL_ADCEx_InjectedStart(&hadc1) != HAL_OK)
    {
        /* ADC start failed — stay in IDLE with safe defaults */
        bu->adc.vbus = 0.0f;
    }

    /* ── Print banner (no snprintf — saves ~600 bytes of stack) ── */
    CDC_Transmit_FS((uint8_t *)"\r\n[milFOC] Bring-Up Started\r\n", 30u);

    /* ── Seed step counter ── */
    bu->step_cnt = 0u;
}

/* ── Step (20kHz) ───────────────────────────────────────────────────── */

void BringUp_Step(void)
{
    BringUp_t *bu = &bringup;

    /* === 1. ADC offset calibration (first ~0.1s, non-blocking) ====== */
    BringUp_CalibrateADC_Step(&bu->adc);

    /* === 2. Read ADC (always) ========================================= */
    BringUp_ReadADC(&bu->adc);

    /* === 3. Protection checks (always) ================================ */
    BringUp_ProtectionCheck(bu);

    /* === 4. Mode-specific action ====================================== */
    switch (bu->mode)
    {
    case FOC_MODE_IDLE:
        /* No PWM output — TIM1 stays at 50% neutral for ADC trigger.
         * PWM phase outputs can be kept at 50% (zero net voltage).
         * If pwm_active was previously true, disable outputs. */
        if (bu->pwm_active)
        {
            /* Reset to neutral 50% */
            bu->foc.dtc_a = 0.50f;
            bu->foc.dtc_b = 0.50f;
            bu->foc.dtc_c = 0.50f;
            SetPwm(&bu->foc);
            bu->pwm_active = false;
        }
        break;

    case FOC_MODE_PWM_TEST:
        /* Fixed 50% duty on all phases */
        bu->foc.dtc_a = 0.50f;
        bu->foc.dtc_b = 0.50f;
        bu->foc.dtc_c = 0.50f;
        SetPwm(&bu->foc);
        break;

    case FOC_MODE_LOCK:
        /* Wait for ADC calibration before applying motor voltage.
         * Calibration needs zero-current state (PWM 50% neutral). */
        if (!bu->adc.calib_done) break;

        /* Fixed angle + ramped Uq */
        OpenLoop_Update(&bu->ol, BRINGUP_TS);  /* Ramp Uq only */
        /* Keep theta fixed (user-set angle) */
        BringUp_SVPWM(&bu->foc, 0.0f, bu->ol.uq_current,
                      bu->ol.sin_val, bu->ol.cos_val);
        SetPwm(&bu->foc);
        break;

    case FOC_MODE_OPEN_LOOP:
        /* Wait for ADC calibration — prevents measuring motor current
         * as the zero-current offset (would corrupt Ia/Ib/Ic readings). */
        if (!bu->adc.calib_done) break;

#if BRINGUP_VF_MODE
        /* V/F curve: Uq = Uq_base + ψ_f × ω_elec
         * This matches voltage to speed, minimizing I²R heating.
         * At 8Hz: Uq = 0.3 + 0.0021×50.3 = 0.41V → I≈3.4A → ~1.4W heat */
        bu->ol.uq_target = BRINGUP_UQ_BASE
                         + MOTOR_FLUX * bu->ol.speed_current_hz * M_2PI;
#endif

        /* Rotating voltage vector with ramped speed & Uq */
        OpenLoop_Update(&bu->ol, BRINGUP_TS);
        BringUp_SVPWM(&bu->foc, 0.0f, bu->ol.uq_current,
                      bu->ol.sin_val, bu->ol.cos_val);
        SetPwm(&bu->foc);
        break;

    case FOC_MODE_CURRENT:
    case FOC_MODE_SPEED:
        /* Future: closed-loop modes */
        break;

    default:
        break;
    }

    /* === 4. Step counter & runtime ==================================== */
    bu->step_cnt++;
    if ((bu->step_cnt % BRINGUP_VOFA_DIV) == 0u)
    {
        /* Every 100 steps @ 20kHz → 200Hz */
        bu->run_time_ms += 5u;  /* 100 × 50µs = 5ms */
    }
}

/* ── Process USB Command ────────────────────────────────────────────── */

void BringUp_ProcessCmd(void)
{
    BringUp_t *bu = &bringup;

    if (bu->cmd_ready)
    {
        BringUp_ParseCmd(bu, bu->cmd_buf);
        bu->cmd_len  = 0u;
        bu->cmd_ready = false;
        memset(bu->cmd_buf, 0, sizeof(bu->cmd_buf));
    }
}

/* ── Telemetry (100Hz) ──────────────────────────────────────────────── */

/**
 * @brief  Send telemetry in VOFA+ FireWater protocol format @ 100Hz
 *
 *         FireWater frame: comma-separated floats terminated by '\n'.
 *         VOFA+ config: 协议=FireWater, 端口=STM32 Virtual COM.
 *
 *         Channel map (12 channels):
 *           CH1:  Vbus         — Bus voltage [V]
 *           CH2:  Ia           — Phase A filtered current [A]
 *           CH3:  Ib           — Phase B filtered current [A]
 *           CH4:  Ic           — Phase C filtered current [A]
 *           CH5:  I_sum        — Ia+Ib+Ic [A] (Kirchhoff check, should ≈0)
 *           CH6:  Mode         — FOC mode enum (0=IDLE,1=PWM,2=LOCK,3=OPEN)
 *           CH7:  Uq           — Uq voltage command [V]
 *           CH8:  Theta        — Electrical angle [rad]
 *           CH9:  Speed        — Electrical speed [Hz]
 *           CH10: Runtime      — Runtime [s]
 *           CH11: Faults       — Fault bitmask (OV=1,UV=2,OC=4,SUM=8)
 *           CH12: Ia_raw       — ADC raw value (bias diagnostic)
 */
void BringUp_Telemetry(void)
{
    BringUp_t *bu = &bringup;

    float fw[12];

    fw[0]  = bu->adc.vbus;                                           /* CH1:  Vbus [V] */
    fw[1]  = bu->adc.i_a_filt;                                       /* CH2:  Ia [A] */
    fw[2]  = bu->adc.i_b_filt;                                       /* CH3:  Ib [A] */
    fw[3]  = bu->adc.i_c_filt;                                       /* CH4:  Ic [A] */
    fw[4]  = bu->adc.i_a_filt + bu->adc.i_b_filt + bu->adc.i_c_filt; /* CH5:  ΣI [A] */
    fw[5]  = (float)bu->mode;                                        /* CH6:  Mode */
    fw[6]  = bu->ol.uq_current;                                      /* CH7:  Uq [V] */
    fw[7]  = bu->ol.theta_elec;                                      /* CH8:  θ [rad] */
    fw[8]  = bu->ol.speed_current_hz;                                /* CH9:  Speed [Hz] */
    fw[9]  = (float)bu->run_time_ms * 0.001f;                        /* CH10: Time [s] */
    fw[10] = (float)((bu->fault_ov ? 1 : 0) | (bu->fault_uv ? 2 : 0) |
                     (bu->fault_oc ? 4 : 0) | (bu->fault_adc_sum ? 8 : 0)); /* CH11: Faults */
    fw[11] = (float)bu->adc.i_a_raw;                                 /* CH12: ADC raw */

    vofa_firewater_send(fw, 12);
}

/* ── USB CDC Feed ───────────────────────────────────────────────────── */

void BringUp_FeedChar(uint8_t data)
{
    BringUp_t *bu = &bringup;

    /* Ignore null bytes */
    if (data == 0) return;

    /* Newline or carriage return → command complete */
    if (data == '\n' || data == '\r')
    {
        if (bu->cmd_len > 0)
        {
            bu->cmd_buf[bu->cmd_len] = '\0';
            bu->cmd_ready = true;
        }
        return;
    }

    /* Backspace */
    if (data == 0x08 || data == 0x7F)
    {
        if (bu->cmd_len > 0)
            bu->cmd_len--;
        return;
    }

    /* Append character */
    if (bu->cmd_len < sizeof(bu->cmd_buf) - 1)
    {
        bu->cmd_buf[bu->cmd_len++] = (char)data;
    }
}

/* ── Emergency Stop ─────────────────────────────────────────────────── */

void BringUp_EmergencyStop(void)
{
    BringUp_t *bu = &bringup;

    bu->mode = FOC_MODE_IDLE;
    bu->pwm_active = false;

    /* Set all duty cycles to 50% neutral */
    bu->foc.dtc_a = 0.50f;
    bu->foc.dtc_b = 0.50f;
    bu->foc.dtc_c = 0.50f;
    SetPwm(&bu->foc);

    const char *msg = "\r\n[EMERGENCY] Fault detected — PWM disabled!\r\n";
    CDC_Transmit_FS((uint8_t *)msg, (uint16_t)strlen(msg));
}
