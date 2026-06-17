/**
 ******************************************************************************
 * @file    pid.c
 * @author  milFOC Team
 * @brief   PID controller implementation.
 *          Position-form PID with integral anti-windup via output clamping.
 *
 * @note    The derivative term uses "derivative on measurement" to avoid
 *          "derivative kick" on setpoint changes.
 ******************************************************************************
 */

#include "pid.h"
#include "string.h"

/**
 * @brief  Initialize PID controller
 */
void PID_Init(PidTypeDef *pid, uint8_t mode, const float PID[3],
              float max_out, float max_iout)
{
    if (pid == NULL) return;

    memset(pid, 0, sizeof(PidTypeDef));

    pid->mode     = mode;
    pid->Kp       = PID[0];
    pid->Ki       = PID[1];
    pid->Kd       = PID[2];
    pid->max_out  = max_out;
    pid->max_iout = max_iout;
}

/**
 * @brief  Calculate PID output (position-form)
 *
 *         u(t) = Kp * e(t) + Ki * integral(e) + Kd * de/dt
 *
 *         Anti-windup: integral term is clamped to max_iout.
 *         Output is clamped to [-max_out, +max_out].
 */
float PID_Calc(PidTypeDef *pid, float ref, float set)
{
    if (pid == NULL) return 0.0f;

    pid->set = set;
    pid->fdb = ref;

    /* Update error history */
    pid->error[2] = pid->error[1];
    pid->error[1] = pid->error[0];
    pid->error[0] = set - ref;

    if (pid->mode == PID_POSITION)
    {
        /* Proportional term */
        pid->Pout = pid->Kp * pid->error[0];

        /* Integral term with anti-windup clamping */
        pid->Iout += pid->Ki * pid->error[0];
        if (pid->Iout >  pid->max_iout) pid->Iout =  pid->max_iout;
        if (pid->Iout < -pid->max_iout) pid->Iout = -pid->max_iout;

        /* Derivative term (on measurement, not error, to avoid kick) */
        pid->Dbuf[2] = pid->Dbuf[1];
        pid->Dbuf[1] = pid->Dbuf[0];
        pid->Dbuf[0] = ref;
        pid->Dout = pid->Kd * (pid->Dbuf[1] - pid->Dbuf[0]);

        /* Total output */
        pid->out = pid->Pout + pid->Iout + pid->Dout;
    }
    else /* PID_DELTA (incremental form) */
    {
        pid->Pout = pid->Kp * (pid->error[0] - pid->error[1]);
        pid->Iout = pid->Ki * pid->error[0];
        pid->Dbuf[2] = pid->Dbuf[1];
        pid->Dbuf[1] = pid->Dbuf[0];
        pid->Dbuf[0] = ref;
        pid->Dout = pid->Kd * (pid->Dbuf[2] - 2.0f * pid->Dbuf[1] + pid->Dbuf[0]);
        pid->out += pid->Pout + pid->Iout + pid->Dout;
    }

    /* Output clamping */
    if (pid->out >  pid->max_out) pid->out =  pid->max_out;
    if (pid->out < -pid->max_out) pid->out = -pid->max_out;

    return pid->out;
}

/**
 * @brief  Clear PID integrator and error/differential buffers
 */
void PID_Clear(PidTypeDef *pid)
{
    if (pid == NULL) return;
    pid->error[0] = pid->error[1] = pid->error[2] = 0.0f;
    pid->Dbuf[0]  = pid->Dbuf[1]  = pid->Dbuf[2]  = 0.0f;
    pid->Pout     = 0.0f;
    pid->Iout     = 0.0f;
    pid->Dout     = 0.0f;
    pid->out      = 0.0f;
}
