/**
 ******************************************************************************
 * @file    virtual_encoder.h
 * @author  milFOC Team
 * @brief   Virtual magnetic encoder for FOC calculation testing.
 *          Simulates a rotating rotor at configurable mechanical speed,
 *          generating electrical angle, raw MT6816-like counts, and
 *          sin/cos values for Park/InvPark transforms.
 *
 * @note    Used in calc_test branch for open-loop FOC verification.
 *          Replaces MT6816 hardware encoder in simulation mode.
 *          Pole pairs: 7 (5010 750KV motor)
 ******************************************************************************
 */

#ifndef VIRTUAL_ENCODER_H
#define VIRTUAL_ENCODER_H

#include "general_def.h"

/* ======================== Virtual Encoder Instance ========================= */

typedef struct
{
    /* --- Configuration --- */
    float   omega_mech;        /* Mechanical angular velocity [rad/s] (default 5.0) */
    uint8_t pole_pairs;        /* Motor pole pairs (default 7) */

    /* --- State --- */
    float   theta;             /* Electrical angle [rad], range [-PI, +PI] */
    float   mec_angle;         /* Mechanical angle [rad], range [0, 2*PI) */
    float   sin_val;           /* sin(theta) — cached for transforms */
    float   cos_val;           /* cos(theta) — cached for transforms */

    /* --- Simulated MT6816 raw output --- */
    uint32_t raw_count;        /* Raw 14-bit count [0, 16383], simulating MT6816 */
    uint32_t full_turns;       /* Accumulated full turns (multi-turn tracking) */
    float    pos_estimate;     /* Position estimate [turn] */
    float    vel_estimate;     /* Velocity estimate [turn/s] */

} VIRTUAL_ENCODER;

/* ======================== Global Instance ================================== */
extern VIRTUAL_ENCODER virtual_encoder;

/* ======================== Public API ======================================= */

/**
 * @brief  Initialize virtual encoder
 * @param  enc        Pointer to virtual encoder instance
 * @param  omega_mech Mechanical angular velocity [rad/s]
 * @param  pp         Motor pole pairs
 */
void VirtualEncoder_Init(VIRTUAL_ENCODER *enc, float omega_mech, uint8_t pp);

/**
 * @brief  Update virtual encoder state (call at FOC frequency, e.g. 20kHz)
 * @param  enc  Pointer to virtual encoder instance
 * @param  Ts   Sampling period [s] (typically 1/20000 = 50us)
 *
 * @note   Advances theta by omega_mech * pole_pairs * Ts.
 *         Updates mec_angle, sin_val, cos_val, raw_count, and position estimate.
 *         Execution target: <1us (no SPI, pure math).
 */
void VirtualEncoder_Update(VIRTUAL_ENCODER *enc, float Ts);

/**
 * @brief  Get current electrical angle [rad]
 */
static inline float VirtualEncoder_GetTheta(VIRTUAL_ENCODER *enc)
{
    return enc->theta;
}

#endif /* VIRTUAL_ENCODER_H */
