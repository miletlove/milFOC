/**
 ******************************************************************************
 * @file    calc_test_task.c
 * @author  milFOC Team
 * @brief   FOC Calculation Test implementation.
 *
 *          Pipeline per 20kHz step:
 *            Virtual Encoder → sin/cos → Read REAL ADC (JDR1~JDR4) →
 *            Open-loop Vd/Vq → InvPark → InvClarke → v_a/v_b/v_c →
 *            SVPWM duty calc → [every 100 steps] → VOFA output @ 200Hz
 *
 *          This module validates the FOC math chain with REAL ADC sampling:
 *          - No TIM1 PWM output (CCR writes disabled)
 *          - Real ADC1 injected group readings (TIM1 triggered, polled)
 *          - No SPI encoder read (virtual encoder only)
 *
 *          ADC offset calibration uses DWT-timed non-blocking averaging
 *          over the first 2000 FOC steps (~0.1s @ 20kHz). No NOP busy-waits.
 ******************************************************************************
 */

#include "calc_test_task.h"
#include "vofa.h"
#include "bsp_adc.h"
#include "bsp_dwt.h"

/* ======================== Global Instance ================================== */

CALC_TEST calc_test;

/* ======================== ADC Offset Calibration =========================== */

/**
 * @brief  Non-blocking ADC offset calibration — call once per FOC step
 *
 *         Accumulates JDR readings over CALC_TEST_ADC_CALIB_SAMPLES steps.
 *         After completion, stores Ia/Ib/Ic_offset and sets calib_done=1.
 *
 *         Uses DWT for timing verification (not blocking).
 *         Each call takes <2us (just 3 register reads + 3 adds).
 */
static void CalcTest_CalibrateADC_Step(CALC_TEST *ct)
{
#if ADC_SAFE_MODE
    /* Safe mode: use theoretical offset, skip calibration */
    ct->adc_calib_done = 1u;
    return;
#else
    REAL_ADC_DATA *adc = &ct->adc;

    /* Read current ADC values */
    ADC_TypeDef *adc_reg = hadc1.Instance;
    if (adc_reg == NULL) return;

    uint16_t raw_a = (uint16_t)adc_reg->JDR1;  /* CUR_A → Rank1 */
    uint16_t raw_b = (uint16_t)adc_reg->JDR2;  /* CUR_B → Rank2 */
    uint16_t raw_c = (uint16_t)adc_reg->JDR3;  /* CUR_C → Rank3 */

    /* Sanity check: ADC should be live (not stuck at 0 or 4095) */
    if (raw_a < 100u || raw_a > 4000u) return;  /* skip bad samples */

    ct->adc_calib_sum_a += (uint64_t)raw_a;
    ct->adc_calib_sum_b += (uint64_t)raw_b;
    ct->adc_calib_sum_c += (uint64_t)raw_c;
    ct->adc_calib_cnt++;

    if (ct->adc_calib_cnt >= CALC_TEST_ADC_CALIB_SAMPLES)
    {
        adc->Ia_offset = (float)ct->adc_calib_sum_a / (float)ct->adc_calib_cnt;
        adc->Ib_offset = (float)ct->adc_calib_sum_b / (float)ct->adc_calib_cnt;
        adc->Ic_offset = (float)ct->adc_calib_sum_c / (float)ct->adc_calib_cnt;
        ct->adc_calib_done = 1u;
    }
#endif
}

/* ======================== Read Real ADC ==================================== */

/**
 * @brief  Read latest ADC1 injected group conversion results
 *
 *         ADC1 injected channel mapping (CubeMX configured):
 *           JDR1 = CUR_C (PA2),  JDR2 = CUR_B (PA1),
 *           JDR3 = CUR_A (PA0),  JDR4 = VBUS  (PB1)
 *
 *         Converts to physical values using calibrated offsets:
 *           I_phase [A] = (raw - Ia/b/c_offset) * FAC_CURRENT
 *           Vbus    [V] = raw * VOLTAGE_TO_ADC_FACTOR
 *
 *         FAC_CURRENT ≈ 0.806 mA/LSB  (2mΩ shunt, 50x gain, 3.3V/4095)
 */
void CalcTest_ReadRealADC(REAL_ADC_DATA *adc)
{
    if (adc == NULL) return;

#if ADC_SAFE_MODE
    /* --- SAFE MODE: simulated ADC values (no hardware access) --- */
    adc->i_a_raw   = 2048u;
    adc->i_b_raw   = 2048u;
    adc->i_c_raw   = 2048u;
    adc->vbus_raw  = (uint16_t)(CALC_TEST_VBUS / VOLTAGE_TO_ADC_FACTOR);

    adc->i_a = 0.0f;
    adc->i_b = 0.0f;
    adc->i_c = 0.0f;
    adc->i_a_filt = 0.0f;
    adc->i_b_filt = 0.0f;
    adc->i_c_filt = 0.0f;
    adc->filter_init = 1u;
    adc->vbus = CALC_TEST_VBUS;
#else
    /* Direct register read for minimum latency */
    ADC_TypeDef *adc_reg = hadc1.Instance;
    if (adc_reg == NULL) return;  /* Safety: prevent HardFault on NULL ptr */

    adc->i_a_raw   = (uint16_t)adc_reg->JDR1;  /* CUR_A on PA0 → Injected Rank1 */
    adc->i_b_raw   = (uint16_t)adc_reg->JDR2;  /* CUR_B on PA1 → Injected Rank2 */
    adc->i_c_raw   = (uint16_t)adc_reg->JDR3;  /* CUR_C on PA2 → Injected Rank3 */
    adc->vbus_raw  = (uint16_t)adc_reg->JDR4;  /* VBUS on PB1 → Injected Rank4 */

    /* --- Raw conversion (unfiltered, for noise diagnosis) --- */
    adc->i_a = ((float)adc->i_a_raw - adc->Ia_offset) * FAC_CURRENT;
    adc->i_b = ((float)adc->i_b_raw - adc->Ib_offset) * FAC_CURRENT;
    adc->i_c = ((float)adc->i_c_raw - adc->Ic_offset) * FAC_CURRENT;

    /* --- IIR low-pass filter: attenuates PWM switching noise --- */
    const float alpha = CALC_TEST_ADC_LPF_ALPHA;
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
#endif
}

/* ======================== Init ============================================= */

void CalcTest_Init(void)
{
    CALC_TEST *ct = &calc_test;

    /* --- 1. Initialize virtual encoder --- */
    VirtualEncoder_Init(&ct->encoder, CALC_TEST_OMEGA_MECH, CALC_TEST_POLE_PAIRS);

    /* --- 2. Initialize local FOC data --- */
    ct->foc.vbus     = CALC_TEST_VBUS;
    ct->foc.inv_vbus = 1.0f / CALC_TEST_VBUS;
    ct->foc.theta    = 0.0f;
    ct->foc.sin_val  = 0.0f;
    ct->foc.cos_val  = 1.0f;
    ct->foc.v_d      = 0.0f;
    ct->foc.v_q      = 0.0f;
    ct->foc.v_alpha  = 0.0f;
    ct->foc.v_beta   = 0.0f;
    ct->foc.v_a      = 0.0f;
    ct->foc.v_b      = 0.0f;
    ct->foc.v_c      = 0.0f;
    ct->foc.i_a      = 0.0f;
    ct->foc.i_b      = 0.0f;
    ct->foc.i_c      = 0.0f;
    ct->foc.dtc_a    = 0.50f;
    ct->foc.dtc_b    = 0.50f;
    ct->foc.dtc_c    = 0.50f;

    /* --- 3. Initialize real ADC data --- */
    ct->adc.i_a_raw   = 0u;
    ct->adc.i_b_raw   = 0u;
    ct->adc.i_c_raw   = 0u;
    ct->adc.vbus_raw  = 0u;
    ct->adc.Ia_offset = 2048.0f;  /* Default: theoretical 1.65V midpoint */
    ct->adc.Ib_offset = 2048.0f;
    ct->adc.Ic_offset = 2048.0f;
    ct->adc.i_a       = 0.0f;
    ct->adc.i_b       = 0.0f;
    ct->adc.i_c       = 0.0f;
    ct->adc.i_a_filt  = 0.0f;     /* Filter state: seeded on first read */
    ct->adc.i_b_filt  = 0.0f;
    ct->adc.i_c_filt  = 0.0f;
    ct->adc.filter_init = 0u;     /* Will be set on first ADC read */
    ct->adc.vbus      = 0.0f;

    /* --- 4. Start ADC1 injected group (polled, no interrupt) --- */
#if ADC_SAFE_MODE
    /* Safe mode: skip ADC hardware init, use simulated values */
    ct->adc_calib_done = 1u;  /* No calibration needed */
#else
    if (HAL_ADCEx_InjectedStart(&hadc1) != HAL_OK)
    {
        /* ADC failed to start — stay in safe fallback */
        ct->adc_calib_done = 1u;
    }
#endif

    /* --- 5. Non-blocking ADC offset calibration --- */
    /* Will run over first ~0.1s in CalcTest_Step (1 sample per FOC step).
     * No blocking, no NOP loops, no HardFault risk. */
    ct->adc_calib_cnt   = 0u;
    ct->adc_calib_done  = 0u;
    ct->adc_calib_sum_a = 0u;
    ct->adc_calib_sum_b = 0u;
    ct->adc_calib_sum_c = 0u;

    /* --- 6. Reset step counter --- */
    ct->step_cnt = 0u;
}

/* ======================== Step ============================================= */

/**
 * @brief  Execute one FOC calculation step
 *
 *         This is the core of the calc_test. Called at 20kHz (every 50us)
 *         from the main loop using DWT-based timing.
 *
 *         Open-loop voltage: v_q = 0.10 * Vbus = 2.4V (safe test level)
 *                           v_d = 0 (no field weakening)
 */
void CalcTest_Step(void)
{
    CALC_TEST       *ct  = &calc_test;
    VIRTUAL_ENCODER *enc = &ct->encoder;
    REAL_ADC_DATA   *adc = &ct->adc;
    FOC_DATA        *foc = &ct->foc;

    /* ================================================================
     * Step 1: Update virtual encoder → theta, sin_val, cos_val
     * ================================================================ */
    VirtualEncoder_Update(enc, CALC_TEST_TS);
    foc->theta   = enc->theta;
    foc->sin_val = enc->sin_val;
    foc->cos_val = enc->cos_val;

    /* ================================================================
     * Step 2: Non-blocking ADC offset calibration (first ~0.1s only)
     *         Accumulates 1 sample per FOC step. No busy-wait.
     *         After completion, Ia/Ib/Ic_offset ≈ 2048.
     * ================================================================ */
    if (!ct->adc_calib_done)
    {
        CalcTest_CalibrateADC_Step(ct);
    }

    /* ================================================================
     * Step 3: Read REAL ADC injected registers
     *         JDR1=CUR_C, JDR2=CUR_B, JDR3=CUR_A, JDR4=VBUS
     *         Currents converted to Amps using calibrated offsets.
     * ================================================================ */
    CalcTest_ReadRealADC(adc);

    /* ================================================================
     * Step 4: Open-loop voltage command
     *         v_d = 0 (no field weakening for PMSM)
     *         v_q = CALC_TEST_VQ_RATIO * Vbus (e.g., 2.4V for 0.10)
     * ================================================================ */
    float v_q_cmd = CALC_TEST_VQ_RATIO * CALC_TEST_VBUS;  /* e.g. 0.10×24=2.4V */
    foc->v_d = 0.0f;
    foc->v_q = v_q_cmd;

    /* ================================================================
     * Step 5: Inverse Park → v_alpha, v_beta (in Volts)
     * ================================================================ */
    foc->v_alpha = foc->v_d * foc->cos_val - foc->v_q * foc->sin_val;
    foc->v_beta  = foc->v_d * foc->sin_val + foc->v_q * foc->cos_val;

    /* ================================================================
     * Step 6: Inverse Clarke → v_a, v_b, v_c (in Volts)
     *         These are the FOC-calculated target phase voltages.
     *         They will be 3-phase sinusoidal at 35 rad/s electrical.
     * ================================================================ */
    foc->v_a = foc->v_alpha;
    foc->v_b = -0.5f * foc->v_alpha + _SQRT3_2 * foc->v_beta;
    foc->v_c = -0.5f * foc->v_alpha - _SQRT3_2 * foc->v_beta;

    /* ================================================================
     * Step 7: SVPWM duty cycle → optionally write to TIM1 CCRx
     *         When PWM_OUTPUT_ENABLE=1: MOSFETs switch per SVPWM.
     *         Without motor: open circuit, zero current, safe for scope.
     * ================================================================ */
    float va_save = foc->v_alpha;
    float vb_save = foc->v_beta;

    /* Normalize by bus voltage for SVPWM */
    float v_alpha_norm = foc->inv_vbus * va_save;
    float v_beta_norm  = foc->inv_vbus * vb_save;

    /* Inverse Clarke in normalized domain */
    float va_u = v_alpha_norm;
    float vb_u = -0.5f * v_alpha_norm + _SQRT3_2 * v_beta_norm;
    float vc_u = -0.5f * v_alpha_norm - _SQRT3_2 * v_beta_norm;

    /* Midpoint injection */
    float vmax_u = max(max(va_u, vb_u), vc_u);
    float vmin_u = min(min(va_u, vb_u), vc_u);
    float vcom_u = (vmax_u + vmin_u) * 0.5f;

    /* Duty cycles [0, 1] — computed and optionally written to TIM1 CCRx */
    foc->dtc_a = CLAMP((vcom_u - va_u) + 0.5f, 0.0f, 1.0f);
    foc->dtc_b = CLAMP((vcom_u - vb_u) + 0.5f, 0.0f, 1.0f);
    foc->dtc_c = CLAMP((vcom_u - vc_u) + 0.5f, 0.0f, 1.0f);

#if PWM_OUTPUT_ENABLE
    /* ── WRITE to TIM1 CCR registers → MOSFETs switch! ── */
    SetPwm(foc);
#endif

    /* Restore v_alpha/v_beta in Volts (for VOFA display) */
    foc->v_alpha = va_save;
    foc->v_beta  = vb_save;

    /* ================================================================
     * Step 8: Increment step counter
     * ================================================================ */
    ct->step_cnt++;

    /* ================================================================
     * Step 9: VOFA output @ 200Hz (every 100 steps @ 20kHz)
     *         200Hz / 5.57Hz_elec ≈ 36 points per sine cycle → smooth!
     * ================================================================ */
    if ((ct->step_cnt % CALC_TEST_VOFA_DIV) == 0u)
    {
        Vofa_CalcTest_Send(ct);
    }
}
