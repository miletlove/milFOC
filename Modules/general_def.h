/**
 ******************************************************************************
 * @file    general_def.h
 * @author  milFOC Team
 * @brief   General definitions and utility macros for the milFOC project.
 *          Provides math constants, inline utility functions, angle wrapping,
 *          and data conversion helpers used across all layers.
 ******************************************************************************
 */

#ifndef GENERAL_DEF_H
#define GENERAL_DEF_H

#include "main.h"
#include "stdint.h"
#include "string.h"
#include "stdlib.h"
#include "stdio.h"
#include "stdbool.h"
#include <math.h>

typedef unsigned char bool_t;

/* =========================== Math Constants =============================== */
#ifndef M_PI
#define M_PI        (3.14159265358979323846f)
#endif
#define M_2PI       (6.28318530717958647692f)
#define M_3PI_2     (4.71238898038469f)
#define _SQRT3      (1.7320508075688772935f)
#define _SQRT3_2    (0.86602540378443864f)
#define ONE_BY_SQRT2 (0.7071067811865475f)
#define ONE_BY_SQRT3 (0.57735026919f)
#define TWO_BY_SQRT3 (1.15470053838f)

/* =========================== Utility Macros =============================== */
#define SQ(x)       ((x) * (x))
#define NORM2_f(x, y)   (sqrtf(SQ(x) + SQ(y)))
#define DEG2RAD_f(deg)  ((deg) * (float)(M_PI / 180.0f))
#define RAD2DEG_f(rad)  ((rad) * (float)(180.0f / M_PI))
#define RPM2RADPS_f(rpm)    ((rpm) * (float)((2.0f * M_PI) / 60.0f))
#define RADPS2RPM_f(radps)  ((radps) * (float)(60.0f / (2.0f * M_PI)))

#define ABS(a)      ((a > 0.0f) ? (a) : (-a))
#define min(a, b)   (((a) < (b)) ? (a) : (b))
#define max(a, b)   (((a) > (b)) ? (a) : (b))
#define CLAMP(x, lower, upper)  (min(upper, max(x, lower)))
#define SIGN(x)     (((x) < 0.0f) ? -1.0f : 1.0f)

/* Low-pass filter macros */
#define UTILS_LP_FAST(value, sample, filter_constant) \
    (value -= (filter_constant) * (value - (sample)))
#define UTILS_LP_MOVING_AVG_APPROX(value, sample, N) \
    UTILS_LP_FAST(value, sample, 2.0f / ((N) + 1.0f))

/* =========================== Inline Utilities ============================= */

/** Integer modulus with positive result */
static inline int mod(int dividend, int divisor)
{
    int r = dividend % divisor;
    return (r < 0) ? (r + divisor) : r;
}

/* --- Byte-level data conversion (for CAN/serial protocols) --- */

static inline void int_to_data(int val, uint8_t *data)
{
    data[0] = *(((uint8_t *)(&val)) + 0);
    data[1] = *(((uint8_t *)(&val)) + 1);
    data[2] = *(((uint8_t *)(&val)) + 2);
    data[3] = *(((uint8_t *)(&val)) + 3);
}

static inline int data_to_int(uint8_t *data)
{
    int tmp_int;
    *(((uint8_t *)(&tmp_int)) + 0) = data[0];
    *(((uint8_t *)(&tmp_int)) + 1) = data[1];
    *(((uint8_t *)(&tmp_int)) + 2) = data[2];
    *(((uint8_t *)(&tmp_int)) + 3) = data[3];
    return tmp_int;
}

static inline void float_to_data(float val, uint8_t *data)
{
    data[0] = *(((uint8_t *)(&val)) + 0);
    data[1] = *(((uint8_t *)(&val)) + 1);
    data[2] = *(((uint8_t *)(&val)) + 2);
    data[3] = *(((uint8_t *)(&val)) + 3);
}

static inline float data_to_float(uint8_t *data)
{
    float tmp_float;
    *(((uint8_t *)(&tmp_float)) + 0) = data[0];
    *(((uint8_t *)(&tmp_float)) + 1) = data[1];
    *(((uint8_t *)(&tmp_float)) + 2) = data[2];
    *(((uint8_t *)(&tmp_float)) + 3) = data[3];
    return tmp_float;
}

/* --- Critical section management --- */
static inline uint32_t cpu_enter_critical(void)
{
    __disable_irq();
    return 1;
}

static inline void cpu_exit_critical(uint32_t priority_mask)
{
    (void)priority_mask;
    __enable_irq();
}

/* --- Angle utilities --- */

/** Positive floating-point modulus */
static inline float fmodf_pos(float x, float y)
{
    float out = fmodf(x, y);
    if (out < 0.0f) out += y;
    return out;
}

/** Wrap value to [-pm_range, +pm_range] */
static inline float wrap_pm(float x, float pm_range)
{
    return fmodf_pos(x + pm_range, 2.0f * pm_range) - pm_range;
}

/** Wrap angle to [-PI, +PI] */
static inline float wrap_pm_pi(float theta)
{
    return wrap_pm(theta, M_PI);
}

/**
 * @brief  Fast sine/cosine approximation (polynomial fit)
 * @note   Max error ~0.001, suitable for FOC where speed > absolute accuracy.
 *         For higher precision, use CORDIC or CMSIS-DSP arm_sin_f32/arm_cos_f32.
 */
static inline void fast_sincos(float angle, float *sin_val, float *cos_val)
{
    angle = wrap_pm_pi(angle);

    if (angle < 0.0f)
    {
        *sin_val = 1.27323954f * angle + 0.405284735f * angle * angle;
        if (*sin_val < 0.0f)
            *sin_val = 0.225f * (*sin_val * -*sin_val - *sin_val) + *sin_val;
        else
            *sin_val = 0.225f * (*sin_val * *sin_val - *sin_val) + *sin_val;
    }
    else
    {
        *sin_val = 1.27323954f * angle - 0.405284735f * angle * angle;
        if (*sin_val < 0.0f)
            *sin_val = 0.225f * (*sin_val * -*sin_val - *sin_val) + *sin_val;
        else
            *sin_val = 0.225f * (*sin_val * *sin_val - *sin_val) + *sin_val;
    }

    /* Compute cosine from sine: cos = sin(x + PI/2) */
    *cos_val = 1.27323954f * (angle + 1.57079632f) - 0.405284735f * (angle + 1.57079632f) * (angle + 1.57079632f);
    if (*cos_val < 0.0f)
        *cos_val = 0.225f * (*cos_val * -*cos_val - *cos_val) + *cos_val;
    else
        *cos_val = 0.225f * (*cos_val * *cos_val - *cos_val) + *cos_val;
}

#endif /* GENERAL_DEF_H */
