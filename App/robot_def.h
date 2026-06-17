/**
 ******************************************************************************
 * @file    robot_def.h
 * @author  milFOC Team
 * @brief   Robot/Motor application data definitions.
 *          Contains CAN command IDs, protocol frame structures,
 *          and motor identifiers.
 *
 * @note    Edit this file to configure CAN IDs, command protocols,
 *          and multi-motor setups.
 ******************************************************************************
 */

#ifndef ROBOT_DEF_H
#define ROBOT_DEF_H

#include "stdint.h"

/* ======================== Motor CAN ID ===================================== */
#define FC_MOTOR_ID  0x001   /* milFOC motor CAN identifier */

/* ======================== FDCAN Frame IDs ================================== */
typedef enum
{
    FDCAN_M1_ID = 0x100,    /* Motor 1 (front-left?) */
    FDCAN_M2_ID = 0x1FF,    /* Motor 2 (front-right?) */
    FDCAN_M3_ID = 0x200,    /* Motor 3 (rear-left?) */
    FDCAN_M4_ID = 0x2FF,    /* Motor 4 (rear-right?) */
} FDCAN_ID_e;

/* ======================== Command IDs ====================================== */
typedef enum
{
    CMD_ID_GET_POSITION = 0x01,     /* Set position target */
    CMD_ID_GET_VELOCITY = 0x02,     /* Set velocity target */
    CMD_ID_GET_TORQUE   = 0x03,     /* Set torque target */
    CMD_ID_CLEAR_ERRORS = 0x04,     /* Clear fault state */
    CMD_ID_GET_ENABLED  = 0x05,     /* Enable motor (pre-charge) */
    CMD_ID_GET_STOP     = 0x06,     /* Emergency stop */
    CMD_ID_SET_PID      = 0x07,     /* Set PID parameters */
    CMD_ID_GET_STATUS   = 0x08,     /* Query motor status */
} Cmd_Id_e;

/* ======================== Protocol Frames ================================== */
#pragma pack(1)

/**
 * @brief Receive command frame (from host to motor)
 */
typedef struct
{
    int cmd_id;             /* Command ID (see Cmd_Id_e) */
    float cmd_rx_vel;       /* Velocity setpoint [turn/s] */
    float cmd_rx_pos;       /* Position setpoint [turn] */
    float cmd_rx_torque;    /* Torque setpoint [Nm] */
} Cmd_Rx_s;

/**
 * @brief Transmit telemetry frame (from motor to host)
 */
typedef struct
{
    int cmd_tx_state;       /* Motor state (FAULT_STATE + STATE_MODE) */
    float cmd_tx_vel;       /* Current velocity [turn/s] */
    float cmd_tx_pos;       /* Current position [turn] */
    float cmd_tx_iq;        /* Q-axis current [A] */
    float cmd_tx_vbus;      /* Bus voltage [V] */
} Cmd_Tx_s;

#pragma pack()

#endif /* ROBOT_DEF_H */
