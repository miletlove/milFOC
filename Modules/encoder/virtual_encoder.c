/**
 ******************************************************************************
 * @file    virtual_encoder.c
 * @author  milFOC Team
 * @brief   Virtual magnetic encoder implementation.
 *          Pure software simulation of a rotating magnetic encoder.
 *
 * @note    Theta advances at omega_elec = omega_mech * pole_pairs.
 *          Raw count simulates MT6816 14-bit absolute encoder:
 *          count = floor(mec_angle / (2*PI) * 16384) % 16384
 ******************************************************************************
 */

#include "virtual_encoder.h"
#include "arm_math.h"

/* Local constants (matches MT6816 specs, avoids cross-dependency) */
#define VENC_CPR     16384u   /* Counts per mechanical revolution */
#define VENC_CPR_F   16384.0f

/* ======================== Global Instance ================================== */

VIRTUAL_ENCODER virtual_encoder;

/* ======================== Init ============================================= */

void VirtualEncoder_Init(VIRTUAL_ENCODER *enc, float omega_mech, uint8_t pp)
{
    if (enc == NULL) return;

    enc->omega_mech  = omega_mech;
    enc->pole_pairs  = (pp > 0) ? pp : 7u;

    enc->theta       = 0.0f;
    enc->mec_angle   = 0.0f;
    enc->sin_val     = 0.0f;
    enc->cos_val     = 1.0f;
    enc->raw_count   = 0u;
    enc->full_turns  = 0u;
    enc->pos_estimate = 0.0f;
    enc->vel_estimate = omega_mech / M_2PI;  /* rad/s → turn/s */
}

/* ======================== Update =========================================== */

void VirtualEncoder_Update(VIRTUAL_ENCODER *enc, float Ts)
{
    if (enc == NULL) return;
    if (Ts <= 0.0f || Ts > 0.1f) Ts = 5e-5f;  /* clamp to 50us default */

    /* --- Step 1: Advance mechanical angle --- */
    enc->mec_angle += enc->omega_mech * Ts;

    /* Wrap mechanical angle to [0, 2*PI) */
    int full_turns = (int)(enc->mec_angle / M_2PI);
    if (full_turns > 0)
    {
        enc->full_turns += (uint32_t)full_turns;
        enc->mec_angle -= (float)full_turns * M_2PI;
    }

    /* --- Step 2: Compute electrical angle --- */
    /* theta_elec = pole_pairs * theta_mech, wrapped to [-PI, +PI] */
    enc->theta = enc->pole_pairs * enc->mec_angle;
    enc->theta = wrap_pm_pi(enc->theta);

    /* --- Step 3: Cache sin/cos for Park/InvPark transforms --- */
    /* Using CMSIS-DSP for consistency with existing FOC code.
     * TODO: migrate to CORDIC hardware accelerator for zero-wait-state. */
    enc->sin_val = arm_sin_f32(enc->theta);
    enc->cos_val = arm_cos_f32(enc->theta);

    /* --- Step 4: Simulate MT6816 14-bit raw count --- */
    /* MT6816: 16384 CPR, absolute position within one mech revolution */
    enc->raw_count = (uint32_t)(enc->mec_angle / M_2PI * VENC_CPR_F);
    if (enc->raw_count >= VENC_CPR) enc->raw_count -= VENC_CPR;

    /* --- Step 5: Position/velocity estimates (for diagnostic) --- */
    enc->pos_estimate = (float)enc->full_turns + enc->mec_angle / M_2PI;  /* turn */
    enc->vel_estimate = enc->omega_mech / M_2PI;                           /* turn/s */
}
