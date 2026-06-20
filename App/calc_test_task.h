/**
 ******************************************************************************
 * @file    calc_test_task.h
 * @author  milFOC Team
 * @brief   Open-Loop FOC Test & Validation Task.
 *
 *          Provides the complete open-loop FOC execution pipeline:
 *          1. Virtual encoder — simulated rotor at configurable speed
 *          2. REAL ADC1 injected group sampling (TIM1-triggered, polled)
 *          3. IIR low-pass current filtering (α=0.05, ~160Hz cutoff)
 *          4. Open-loop FOC: Vd=0, Vq=configurable → InvPark → SVPWM
 *          5. Optional PWM output to TIM1 (PWM_OUTPUT_ENABLE switch)
 *          6. VOFA+ FireWater telemetry @ 200Hz
 *
 *          This is the foundation for closed-loop FOC — the Vd/Vq
 *          computation will be replaced by current PI controllers,
 *          and the virtual encoder by a real position sensor.
 *
 *          Timing (DWT-based, main loop):
 *            FOC step:     20 kHz (50 µs)
 *            ADC calib:    first 0.1 s (2000 steps, non-blocking)
 *            VOFA output:  200 Hz (every 100 steps)
 ******************************************************************************
 */

#ifndef CALC_TEST_TASK_H
#define CALC_TEST_TASK_H

#include "general_def.h"
#include "bldc_motor.h"
#include "virtual_encoder.h"
#include "adc.h"

/* ======================== Test Configuration =============================== */

#define CALC_TEST_VBUS         24.0f    /* Bus voltage [V] */
#define CALC_TEST_OMEGA_MECH   5.0f     /* Target mechanical angular velocity [rad/s] */
#define CALC_TEST_POLE_PAIRS   7u       /* Motor pole pairs */
#define CALC_TEST_FOC_FREQ     20000.0f /* FOC execution frequency [Hz] */
#define CALC_TEST_TS           (1.0f / CALC_TEST_FOC_FREQ)  /* 50us */
/*
 * VOFA output divider: 20000 / CALC_TEST_VOFA_DIV = VOFA frequency [Hz]
 *
 *   DIV | VOFA Hz | Points per elec cycle (35 rad/s ≈ 5.57 Hz)
 *   ----|---------|----------------------------------------------
 *   400 |   50    |  ~9   →  triangluar aliasing (欠采样三角波)
 *   100 |  200    | ~36   →  smooth sine (光滑正弦) ★推荐
 *    40 |  500    | ~90   →  perfect sine, higher USB load
 */
#define CALC_TEST_VOFA_DIV     100u     /* VOFA output every N FOC steps → 200Hz */

/* ======================== Open-Loop Voltage Command ======================== */
/*
 * v_q = CALC_TEST_VQ_RATIO * Vbus  →  determines phase voltage amplitude.
 *
 *   v_q_ratio |  v_q  | Phase amplitude | SVPWM modulation
 *   -----------|-------|-----------------|-------------------
 *      0.10    | 2.4V  |    ±2.4V        |  Safe idle test
 *      0.30    | 7.2V  |    ±7.2V        |  Medium load
 *      0.50    | 12.0V |    ±12.0V       |  Near max linear
 *      0.577   | 13.9V |    ±13.9V       |  Linear limit (=Vbus/√3)
 *
 *   v_a/v_b/v_c peak = v_q (when v_d=0).
 *   Increase this ratio to increase phase voltages.
 */
#define CALC_TEST_VQ_RATIO      0.10f   /* v_q = 10% of Vbus → 2.4V (safe start) */

/*
 * PWM Output Enable:
 *   0 = Calculate only, DO NOT write to TIM1 CCR (safe, no MOSFET switching)
 *   1 = Write SVPWM duty cycles to TIM1 CCRx → MOSFETs switch per FOC output
 *
 *   ⚠️ WARNING: Only enable PWM_OUTPUT when:
 *      - Motor is DISCONNECTED (for oscilloscope testing), OR
 *      - Bootstrap capacitors are pre-charged (Foc_Pwm_Start already done), AND
 *      - Current protection is active (future closed-loop mode)
 */
#define PWM_OUTPUT_ENABLE       0   /* 0=safe(default), 1=MOSFETs switch per SVPWM */

/* Number of ADC samples for offset calibration at startup */
#define CALC_TEST_ADC_CALIB_SAMPLES  2000u

/*
 * ADC Safety Mode:
 *   0 = Real ADC access (JDR registers) — normal operation
 *   1 = Simulated ADC (fixed 2048/2707) — for debugging HardFault
 *
 *   If HardFault occurs with mode=0 but NOT with mode=1,
 *   the fault is in the ADC register access path.
 */
#define ADC_SAFE_MODE  0  /* ← Set to 0 after HardFault is resolved */

/*
 * ADC current low-pass filter coefficient (IIR, first-order).
 *   filtered = filtered + α × (raw - filtered)
 *
 *   α = 0.0 → no filtering (raw data)
 *   α = 0.1 → moderate (cutoff ~320Hz @ 20kHz)
 *   α = 0.01 → heavy (cutoff ~32Hz @ 20kHz)
 *
 *   Recommend 0.05 for motor current sensing.
 *   Attenuates PWM-frequency (20kHz) noise while preserving
 *   the 5.57 Hz FOC current waveform.
 */
#define CALC_TEST_ADC_LPF_ALPHA  0.05f

/* ======================== Real ADC Data ==================================== */

/**
 * @brief Real ADC sampled data (from ADC1 injected group JDR1~JDR4).
 *
 *        ADC1 injected channel mapping (CubeMX):
 *          JDR1 = CUR_C (PA2), JDR2 = CUR_B (PA1),
 *          JDR3 = CUR_A (PA0), JDR4 = VBUS  (PB1)
 *
 *        At idle (no motor current):
 *          i_a/b/c_raw ≈ 2048 (1.65V midpoint @ 3.3V Vref)
 *          vbus_raw    = Vbus / VOLTAGE_TO_ADC_FACTOR
 */
typedef struct
{
    /* Raw ADC readings (12-bit, 0-4095) */
    uint16_t i_a_raw;
    uint16_t i_b_raw;
    uint16_t i_c_raw;
    uint16_t vbus_raw;

    /* Calibrated ADC offsets (set by non-blocking calibration) */
    float Ia_offset;    /* CUR_A zero-current offset [ADC raw] — should ≈2048 */
    float Ib_offset;    /* CUR_B zero-current offset [ADC raw] */
    float Ic_offset;    /* CUR_C zero-current offset [ADC raw] */

    /* Raw converted currents (unfiltered, for noise diagnosis) */
    float i_a;      /* Phase A current [A] = (raw - Ia_offset) * FAC_CURRENT */
    float i_b;      /* Phase B current [A] */
    float i_c;      /* Phase C current [A] */

    /* IIR low-pass filtered currents (smoothed, for control/display) */
    float i_a_filt; /* Filtered phase A current [A] */
    float i_b_filt; /* Filtered phase B current [A] */
    float i_c_filt; /* Filtered phase C current [A] */
    uint8_t filter_init; /* 1 = filter state valid (first sample loaded) */

    /* Bus voltage (low-pass filtered inherently by ADC input cap) */
    float vbus;     /* Bus voltage [V] = raw * VOLTAGE_TO_ADC_FACTOR */

} REAL_ADC_DATA;

/* ======================== Calc Test Instance =============================== */

typedef struct CALC_TEST
{
    VIRTUAL_ENCODER encoder;    /* Virtual encoder */
    REAL_ADC_DATA   adc;        /* Real ADC readings */
    FOC_DATA        foc;        /* Local FOC math data (not global foc_data!) */
    uint32_t        step_cnt;   /* FOC step counter */

    /* Non-blocking ADC offset calibration state */
    uint32_t adc_calib_cnt;     /* Calibration sample counter (0..CALIB_SAMPLES) */
    uint32_t adc_calib_done;    /* 1 = calibration complete */
    uint64_t adc_calib_sum_a;   /* Accumulator for CUR_A offset */
    uint64_t adc_calib_sum_b;   /* Accumulator for CUR_B offset */
    uint64_t adc_calib_sum_c;   /* Accumulator for CUR_C offset */

} CALC_TEST;

/* ======================== Global Instance ================================== */
extern CALC_TEST calc_test;

/* ======================== Public API ======================================= */

/**
 * @brief  Initialize the calculation test environment
 *
 *         Sets up:
 *         - Virtual encoder @ 5 rad/s mech, 7 pole pairs
 *         - FOC data structure (vbus=24V, initial theta=0)
 *         - REAL ADC injected group (no interrupt — polled via JDR registers)
 *         - Step counter
 *
 * @note   ADC injected group is triggered by TIM1 CH4 at PWM center.
 *         TIM1 must be initialized (MX_TIM1_Init) and PWM started before
 *         calling this function.
 */
void CalcTest_Init(void);

/**
 * @brief  Execute one FOC calculation step (call @ 20kHz from main loop)
 *
 *         Pipeline per step:
 *         1. VirtualEncoder_Update → theta, sin_val, cos_val
 *         2. Read REAL ADC injected registers (JDR1~JDR4)
 *         3. Open-loop FOC: v_d=0, v_q=test_voltage → InvPark → v_a/v_b/v_c
 *         4. SVPWM duty cycle calculation (NOT written to TIM1)
 *         5. Every CALC_TEST_VOFA_DIV steps: push data to VOFA
 *
 * @note   Does NOT write to TIM1 CCR registers (pure calculation).
 *         Uses REAL hardware ADC (TIM1-triggered, polled).
 *         Target execution: <10us @ 168MHz.
 */
void CalcTest_Step(void);

/**
 * @brief  Read latest ADC injected conversion results + convert to physical
 *
 *         Reads ADC1 JDR1~JDR4 directly, then converts:
 *           I_phase [A] = (raw - Ia/b/c_offset) * FAC_CURRENT
 *           Vbus    [V] = raw * VOLTAGE_TO_ADC_FACTOR
 *
 * @param  adc  Pointer to REAL_ADC_DATA to fill
 */
void CalcTest_ReadRealADC(REAL_ADC_DATA *adc);

#endif /* CALC_TEST_TASK_H */
