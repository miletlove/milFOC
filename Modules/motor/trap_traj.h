/**
 ******************************************************************************
 * @file    trap_traj.h
 * @author  milFOC Team
 * @brief   Trapezoidal trajectory planner for smooth position/velocity control.
 *          Generates S-curve acceleration/deceleration profiles.
 *
 * @note    Used in CONTROL_MODE_POSITION_RAMP mode.
 *          Plans trajectories with configurable max velocity, acceleration,
 *          and deceleration for industrial point-to-point (PTP) motion.
 ******************************************************************************
 */

#ifndef TRAP_TRAJ_H
#define TRAP_TRAJ_H

#include "general_def.h"

/**
 * @brief Trapezoidal trajectory planner instance
 */
typedef struct
{
    float Y;        /* Current position [turn] */
    float Yd;       /* Current velocity [turn/s] */
    float Ydd;      /* Current acceleration [turn/s^2] */

    float Tf_;      /* Total motion time [s] */
    float t;        /* Current timestamp [s] */

    int trajectory_done;    /* Trajectory complete flag */
} tTraj;

/**
 * @brief  Initialize trajectory planner
 * @param  traj: pointer to trajectory instance
 */
void traj_init(tTraj *traj);

/**
 * @brief  Plan a new trapezoidal trajectory
 * @param  traj: pointer to trajectory instance
 * @param  start_pos: starting position [turn]
 * @param  end_pos: target position [turn]
 * @param  max_vel: maximum velocity [turn/s]
 * @param  accel: acceleration [turn/s^2]
 * @param  decel: deceleration [turn/s^2]
 */
void traj_plan(tTraj *traj, float start_pos, float end_pos,
               float max_vel, float accel, float decel);

/**
 * @brief  Evaluate trajectory at current time step
 * @param  traj: pointer to trajectory instance
 * @param  Ts: sampling period [s]
 * @return Current position setpoint [turn]
 */
float traj_eval(tTraj *traj, float Ts);

#endif /* TRAP_TRAJ_H */
