/**
 ******************************************************************************
 * @file    mt6816_encoder.h
 * @author  milFOC Team
 * @brief   MT6816 14-bit absolute magnetic encoder driver.
 *          SPI interface with PLL-based velocity estimation and
 *          nonlinearity correction via LUT.
 *
 * @note    Hardware: SPI1 on PA5(SCK), PA6(MISO), PA7(MOSI)
 *          Resolution: 16384 CPR (14-bit)
 *          PLL bandwidth: 2000 rad/s (configurable)
 ******************************************************************************
 */

#ifndef MT6816_ENCODER_H
#define MT6816_ENCODER_H

#include "general_def.h"
#include "gpio.h"
#include "spi.h"
#include <stm32g431xx.h>

/* ======================== Timing =========================================== */
#define MT6816_MAX_DELAY    (10)       /* SPI read timeout [us] */

/* ======================== SPI Chip Select Macros =========================== */
/* milFOC uses SPI1 for MT6816 (PA5=SCK, PA6=MISO, PA7=MOSI, PB12=CSN) */
#define MT6816_SPI_Get_HSPI  (hspi1)
/* CS pin: PB12 (SPI1_CS), defined in CubeMX main.h */
#define MT6816_SPI_CS_L()    HAL_GPIO_WritePin(SPI1_CS_GPIO_Port, SPI1_CS_Pin, GPIO_PIN_RESET)
#define MT6816_SPI_CS_H()    HAL_GPIO_WritePin(SPI1_CS_GPIO_Port, SPI1_CS_Pin, GPIO_PIN_SET)

/* ======================== MT6816 Register Addresses ======================== */
#define MT6816_Init_Reg       (0x00)       /* Initialization register */
#define MT6816_Angle_Reg      (0x80 | 0x03) /* Angle data register (with parity) */
#define MT6816_Warning_Reg    (0x80 | 0x04) /* Warning register */
#define MT6816_Over_Speed_Reg (0x80 | 0x05) /* Over-speed register */

/* ======================== Encoder Parameters =============================== */
#define ENCODER_PLL_BANDWIDTH  2000.0f   /* PLL bandwidth [rad/s] */
#define ENCODER_CPR            16384u    /* Counts per revolution [uint] */
#define ENCODER_CPR_F          16384.0f  /* Counts per revolution [float] */
#define ENCODER_CPR_DIV        (ENCODER_CPR >> 1)  /* Half CPR */
#define MAX_ANGLE              360.0f
#define MAX_ANGLE_HALF         180.0f

/* ======================== Direction Enum =================================== */
typedef enum
{
    CW      = 1,    /* Clockwise */
    CCW     = -1,   /* Counter-clockwise */
    UNKNOWN = 0,    /* Unknown / invalid */
} Direction;

/* ======================== Encoder Data Structure =========================== */

/**
 * @brief MT6816 encoder instance data
 *
 * Uses a software PLL (Phase-Locked Loop) for velocity estimation
 * and position filtering from the raw 14-bit absolute angle readings.
 *
 * Key outputs:
 *   - phase_: electrical angle [rad] (used by Park/InvPark transforms)
 *   - vel_estimate_: mechanical velocity [turn/s]
 *   - pos_estimate_: accumulated position [turn]
 */
typedef struct ENCODER_DATA
{
    SPI_HandleTypeDef *hspi;
    uint8_t  tx_dma[3];           /* DMA TX buffer: {0x83, 0x00, 0x00} */
    uint8_t  rx_dma[3];           /* DMA RX buffer: [dummy, angle_hi, angle_lo] */

    uint32_t angle;             /* Raw 14-bit angle from MT6816 */
    uint8_t rx_err_count;       /* SPI read error counter */
    uint8_t check_err_count;    /* Parity check error counter */

    int32_t offset_lut[128];    /* Nonlinearity correction LUT [128 entries] */
    int32_t count;              /* Current corrected count value */

    /* Encoder configuration */
    uint8_t pole_pairs;         /* Motor pole pairs */
    int cnt;                    /* Corrected encoder count */
    int raw;                    /* Raw encoder count */
    int dir;                    /* Rotation direction: +1=CW, -1=CCW */
    int pos_abs_;               /* Absolute position */
    int count_in_cpr_;          /* Count within one revolution [0, CPR-1] */
    int count_in_cpr_prev;      /* Previous count_in_cpr_ */
    int shadow_count_;          /* Multi-turn accumulated count */
    int EncoderInit_Flag;       /* Init complete flag */

    /* PLL state variables */
    float pos_estimate_counts_;  /* Position estimate [counts] */
    float vel_estimate_counts_;  /* Velocity estimate [counts/s] */
    float pos_cpr_counts_;       /* Position within one revolution [counts] */

    /* PLL outputs (primary interface) */
    float pos_estimate_;         /* Position [turn] */
    float vel_estimate_;         /* Velocity [turn/s] */
    float pos_cpr_;              /* Position within one revolution [0, 1) */
    float phase_;                /* Electrical angle [rad], range [-PI, PI] */
    float interpolation_;        /* Interpolation factor [0, 1] */

    /* Calibration */
    int calib_valid;             /* Calibration valid flag */
    float mec_angle;             /* Mechanical angle [rad], range [0, 2*PI) */
    float elec_angle;            /* Electrical angle [rad], range [0, 2*PI) */
    float encoder_offset;        /* Encoder zero offset (from calibration) */
    float speed;                 /* Speed [rad/s] */
    float last_angle;            /* Previous angle for differentiation */

    /* Open-loop angle accumulator */
    float theta_acc;             /* Accumulated electrical angle step */
} ENCODER_DATA;

/* ======================== Global Encoder Instance ========================== */
extern ENCODER_DATA encoder_data;

/* ======================== Public API ====================================== */

/** Low-pass filter for encoder readings */
float low_pass_filter(float input);

/** Normalize angle to [0, 2*PI) */
float normalize_angle(float angle);

/**
 * @brief  Read MT6816 angle via SPI and update PLL
 * @note   Called at 20 kHz from ADC JEOC interrupt.
 *         Performs:
 *         1. SPI read of 14-bit absolute angle
 *         2. Parity/noise check
 *         3. Nonlinear LUT correction
 *         4. PLL update for position & velocity estimation
 *         5. Electrical angle computation
 */
void GetMotor_Angle(ENCODER_DATA *encoder);

/**
 * @brief  Accumulate electrical angle for open-loop control
 */
void Theta_ADD(ENCODER_DATA *encoder);

#endif /* MT6816_ENCODER_H */
