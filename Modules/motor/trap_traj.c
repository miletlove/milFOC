/**
 ******************************************************************************
 * @file    trap_traj.c
 * @author  milFOC Team
 * @brief   Trapezoidal trajectory planner implementation.
 *
 * @note    Generates a velocity trapezoid with three phases:
 *          Phase 1: Constant acceleration to max velocity
 *          Phase 2: Constant velocity cruise
 *          Phase 3: Constant deceleration to target
 *
 *          If the displacement is too short, the profile degenerates
 *          to a triangular profile (no cruise phase).
 ******************************************************************************
 */

#include "trap_traj.h"
#include "string.h"

void traj_init(tTraj *traj)
{
    memset(traj, 0, sizeof(tTraj));
}

void traj_plan(tTraj *traj, float start_pos, float end_pos,
               float max_vel, float accel, float decel)
{
    traj->Y   = start_pos;
    traj->Yd  = 0.0f;
    traj->Ydd = 0.0f;
    traj->t   = 0.0f;

    float displacement = end_pos - start_pos;
    float abs_disp = ABS(displacement);

    /* Ensure positive parameters */
    max_vel = ABS(max_vel);
    accel   = ABS(accel);
    decel   = ABS(decel);

    /* Time to accelerate to max velocity */
    float t_accel = max_vel / accel;
    float t_decel = max_vel / decel;

    /* Distance covered during accel + decel */
    float d_accel = 0.5f * accel * t_accel * t_accel;
    float d_decel = 0.5f * decel * t_decel * t_decel;

    /* Check if we can reach max velocity (trapezoidal) or
     * only triangular profile */
    if ((d_accel + d_decel) < abs_disp)
    {
        /* Full trapezoidal profile */
        float d_cruise = abs_disp - d_accel - d_decel;
        float t_cruise = d_cruise / max_vel;
        traj->Tf_ = t_accel + t_cruise + t_decel;
    }
    else
    {
        /* Triangular profile: reduce max velocity */
        /* t_peak = sqrt(2 * disp * accel * decel / (accel + decel)) */
        float t_peak = sqrtf(2.0f * abs_disp * accel * decel / (accel + decel));
        traj->Tf_ = t_peak * (1.0f + accel / decel);
    }

    traj->trajectory_done = 0;
}

float traj_eval(tTraj *traj, float Ts)
{
    if (traj->trajectory_done) return traj->Y;

    traj->t += Ts;

    /* TODO: Implement proper phase-based evaluation with
     * acceleration / cruise / deceleration phases.
     * Current stub: return constant position. */

    if (traj->t >= traj->Tf_)
    {
        traj->trajectory_done = 1;
        traj->Yd  = 0.0f;
        traj->Ydd = 0.0f;
    }

    return traj->Y;
}
