/**
 ******************************************************************************
 * @file    pid.h
 * @author  milFOC Team
 * @brief   Discrete PID controller module.
 *          Supports position-form and delta-form PID with anti-windup
 *          (integral clamping) and output limiting.
 *
 * @note    Used for:
 *          - Current loop (IqPID, IdPID) @ 20 kHz
 *          - Speed loop  (VelPID)        @ 1 kHz
 *          - Position loop (PosPID)      @ 100 Hz
 ******************************************************************************
 */

#ifndef PID_H
#define PID_H

#include "general_def.h"

/** PID calculation mode */
enum PID_MODE
{
    PID_POSITION = 0,   /* Position-form PID */
    PID_DELTA           /* Delta (incremental) PID */
};

/**
 * @brief PID controller instance structure
 */
typedef struct
{
    uint8_t mode;       /* PID_POSITION or PID_DELTA */

    /* Gains */
    float Kp;
    float Ki;
    float Kd;

    /* Limits */
    float max_out;      /* Output saturation limit */
    float max_iout;     /* Integral term saturation limit (anti-windup) */

    /* I/O */
    float set;          /* Setpoint */
    float fdb;          /* Feedback */

    float out;          /* Total output */
    float Pout;         /* Proportional term */
    float Iout;         /* Integral term */
    float Dout;         /* Derivative term */
    float Dbuf[3];      /* Derivative buffer: [0]=curr, [1]=last, [2]=prev */
    float error[3];     /* Error buffer:      [0]=curr, [1]=last, [2]=prev */

} PidTypeDef;

/* --- Public API --- */

/**
 * @brief  Initialize PID controller instance
 * @param  pid:     pointer to PID instance
 * @param  mode:    PID_POSITION or PID_DELTA
 * @param  PID:     array of 3 floats: {Kp, Ki, Kd}
 * @param  max_out: output saturation limit
 * @param  max_iout: integral term saturation limit
 */
void PID_Init(PidTypeDef *pid, uint8_t mode, const float PID[3],
              float max_out, float max_iout);

/**
 * @brief  Calculate PID output
 * @param  pid: pointer to PID instance
 * @param  ref: current feedback value
 * @param  set: target setpoint
 * @return PID output (clamped to max_out)
 */
float PID_Calc(PidTypeDef *pid, float ref, float set);

/**
 * @brief  Clear PID integrator and error history
 * @param  pid: pointer to PID instance
 */
void PID_Clear(PidTypeDef *pid);

#endif /* PID_H */
