/**
 ******************************************************************************
 * @file    mt6816_encoder.c
 * @author  milFOC Team
 * @brief   MT6816 encoder driver implementation.
 *          SPI read, PLL-based velocity estimation, and nonlinearity correction.
 *
 * @note    The PLL (Phase-Locked Loop) provides smooth velocity estimates
 *          from the discrete 14-bit position readings.
 *          PLL gains:
 *            pll_kp = 2 * bandwidth
 *            pll_ki = 0.25 * (pll_kp)^2
 *
 *          TODO: Implement SPI DMA for lower CPU overhead.
 ******************************************************************************
 */

#include "mt6816_encoder.h"
#include "bsp_dwt.h"

/* ======================== Global Encoder Instance ========================== */

ENCODER_DATA encoder_data = {
    .hspi                = &MT6816_SPI_Get_HSPI,
    .pole_pairs          = MPTOR_P,
    .dir                 = MOTOR_DIRECTION,
    .encoder_offset      = MOTOR_OFFSET,
    .calib_valid         = PRE_CALIBRATED,
};

/* PLL gains (computed from bandwidth) */
static float pll_kp_;
static float pll_ki_;

/**
 * @brief  Low-pass filter for encoder readings
 */
float low_pass_filter(float input)
{
    static float prev_output = 0.0f;
    const float alpha = 0.1f;  /* Filter coefficient */
    prev_output = alpha * input + (1.0f - alpha) * prev_output;
    return prev_output;
}

/**
 * @brief  Normalize angle to [0, 2*PI)
 */
float normalize_angle(float angle)
{
    float a = fmodf(angle, M_2PI);
    return (a < 0) ? (a + M_2PI) : a;
}

/**
 * @brief  Read MT6816 angle and update PLL state
 *
 *         Called at 20 kHz from ADC JEOC interrupt.
 *         Must execute quickly (< 5us target).
 */
void GetMotor_Angle(ENCODER_DATA *encoder)
{
    if (encoder == NULL) return;

    /* Compute PLL gains on first call */
    static int pll_init_done = 0;
    if (!pll_init_done)
    {
        pll_kp_ = 2.0f * ENCODER_PLL_BANDWIDTH;      /* 4000 */
        pll_ki_ = 0.25f * (pll_kp_ * pll_kp_);        /* 4e6 */
        pll_init_done = 1;
    }

    /* --- Step 1: Read raw angle from MT6816 via SPI --- */
    uint16_t tx_data = MT6816_Angle_Reg | 0x8000;  /* Read command + parity placeholder */
    uint16_t rx_data = 0;

    MT6816_SPI_CS_L();
    HAL_SPI_TransmitReceive(encoder->hspi, (uint8_t *)&tx_data,
                            (uint8_t *)&rx_data, 1, MT6816_MAX_DELAY);
    MT6816_SPI_CS_H();

    /* Extract 14-bit angle (bits 13:0) */
    encoder->raw = (int)(rx_data & 0x3FFF);

    /* TODO: Parity check on bits 15:14 */

    /* --- Step 2: Apply nonlinearity correction LUT --- */
    if (encoder->calib_valid)
    {
        int idx = (encoder->raw * OFFSET_LUT_NUM) / ENCODER_CPR;
        if (idx >= 0 && idx < OFFSET_LUT_NUM)
        {
            encoder->cnt = encoder->raw - encoder->offset_lut[idx];
        }
        else
        {
            encoder->cnt = encoder->raw;
        }
    }
    else
    {
        encoder->cnt = encoder->raw;
    }

    /* Apply direction */
    encoder->cnt *= encoder->dir;

    /* --- Step 3: Multi-turn tracking --- */
    encoder->count_in_cpr_ = encoder->cnt % ENCODER_CPR;
    if (encoder->count_in_cpr_ < 0) encoder->count_in_cpr_ += ENCODER_CPR;

    /* Track revolutions (simple delta detection) */
    int delta = encoder->count_in_cpr_ - encoder->count_in_cpr_prev;
    if (delta > ENCODER_CPR_DIV)
        encoder->shadow_count_ -= ENCODER_CPR;
    else if (delta < -ENCODER_CPR_DIV)
        encoder->shadow_count_ += ENCODER_CPR;
    encoder->count_in_cpr_prev = encoder->count_in_cpr_;
    encoder->count = encoder->shadow_count_ + encoder->count_in_cpr_;

    /* --- Step 4: PLL update --- */
    /* Position estimate from raw count */
    float pos_estimate_counts = (float)encoder->count;

    /* PLL correction */
    float delta_pos = pos_estimate_counts - encoder->pos_estimate_counts_;
    encoder->pos_estimate_counts_ += encoder->vel_estimate_counts_ * PWM_MEASURE_PERIOD;
    encoder->vel_estimate_counts_ += pll_ki_ * delta_pos * PWM_MEASURE_PERIOD;
    encoder->pos_estimate_counts_ += pll_kp_ * delta_pos * PWM_MEASURE_PERIOD;

    /* Convert to turns */
    encoder->pos_estimate_ = encoder->pos_estimate_counts_ / ENCODER_CPR_F;
    encoder->vel_estimate_ = encoder->vel_estimate_counts_ / ENCODER_CPR_F;
    encoder->pos_cpr_      = (float)encoder->count_in_cpr_ / ENCODER_CPR_F;

    /* --- Step 5: Electrical angle computation --- */
    /* Mechanical angle: position within one revolution */
    encoder->mec_angle = encoder->pos_cpr_ * M_2PI;

    /* Apply encoder offset */
    float mec_angle_offset = encoder->mec_angle
                           - (float)encoder->encoder_offset * M_2PI / ENCODER_CPR_F;
    mec_angle_offset = normalize_angle(mec_angle_offset);

    /* Electrical angle = mechanical angle * pole_pairs */
    if (encoder->pole_pairs > 0)
    {
        encoder->elec_angle = normalize_angle(mec_angle_offset * encoder->pole_pairs);
    }
    else
    {
        encoder->elec_angle = mec_angle_offset;
    }

    /* Phase: map to [-PI, PI] for Park/InvPark transforms */
    encoder->phase_ = wrap_pm_pi(encoder->elec_angle);

    /* Mechanical speed */
    encoder->speed = encoder->vel_estimate_ * M_2PI;  /* [rad/s] */
}

/**
 * @brief  Accumulate electrical angle for open-loop control
 */
void Theta_ADD(ENCODER_DATA *encoder)
{
    if (encoder == NULL) return;
    /* theta_acc is incremented externally by the open-loop controller */
}
