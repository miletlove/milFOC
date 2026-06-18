/**
 ******************************************************************************
 * @file    cmd_task.c
 * @author  milFOC Team
 * @brief   Command dispatch task implementation.
 *          Receives commands via CAN bus and dispatches to motor control.
 *
 * @note    Command protocol: 8-byte CAN frames.
 *          Frame format: [cmd_id:4B][data:4B]
 ******************************************************************************
 */

#include "cmd_task.h"
#include "robot_def.h"
#include "can_driver.h"
#include "foc_motor.h"
#include "bldc_motor.h"
#include "mt6816_encoder.h"
#include "bsp_log.h"
#include "led.h"
#include "string.h"
#include "stdbool.h"

static FDCANCommInstance *ins;
static Cmd_Rx_s cmd_rx;
static Cmd_Tx_s cmd_tx;
static float last_position = 0.0f;

void RobotCMDInit(void)
{
    FDCANComm_Init_Config_s config = {
        .can_config = {
            .fdcan_handle = &hfdcan1,
            .tx_id        = FDCAN_M4_ID + FC_MOTOR_ID,
            .rx_id        = FDCAN_M4_ID,
        },
        .send_data_len = sizeof(cmd_tx),
        .recv_data_len = sizeof(cmd_rx),
        .daemon_count  = 10,
    };
    ins = FDCANCommInit(&config);
    LOGINFO("[CMD] Command dispatch initialized (CAN ID=0x%03X)", FDCAN_M4_ID);
}

void RobotCMDTask(void)
{
    if (ins == NULL) return;

    cmd_rx = *(Cmd_Rx_s *)FDCANCommGet(ins);

    switch (cmd_rx.cmd_id)
    {
    case CMD_ID_GET_POSITION:
        motor_data.state.Control_Mode = CONTROL_MODE_POSITION_RAMP;
        if (cmd_rx.cmd_rx_pos != last_position)
        {
            motor_data.Controller.input_velocity = cmd_rx.cmd_rx_vel;
            motor_data.Controller.input_position = cmd_rx.cmd_rx_pos;
            motor_data.Controller.input_updated  = true;
            last_position = cmd_rx.cmd_rx_pos;
        }
        else
        {
            motor_data.Controller.input_updated = false;
        }
        LOGDEBUG("[CMD] Position cmd: pos=%.2f, vel=%.2f",
                  cmd_rx.cmd_rx_pos, cmd_rx.cmd_rx_vel);
        break;

    case CMD_ID_GET_VELOCITY:
        motor_data.state.Control_Mode = CONTROL_MODE_VELOCITY_RAMP;
        motor_data.Controller.input_velocity = cmd_rx.cmd_rx_vel;
        motor_data.Controller.input_position = cmd_rx.cmd_rx_pos;
        LOGDEBUG("[CMD] Velocity cmd: vel=%.2f", cmd_rx.cmd_rx_vel);
        break;

    case CMD_ID_GET_TORQUE:
        motor_data.state.Control_Mode = CONTROL_MODE_TORQUE;
        motor_data.Controller.input_torque = cmd_rx.cmd_rx_torque;
        LOGDEBUG("[CMD] Torque cmd: torque=%.3f Nm", cmd_rx.cmd_rx_torque);
        break;

    case CMD_ID_CLEAR_ERRORS:
        motor_data.state.State_Mode  = STATE_MODE_IDLE;
        motor_data.state.Fault_State = FAULT_STATE_NORMAL;
        RGB_DisplayColorById(1);
        LOGINFO("[CMD] Errors cleared, state reset to IDLE");
        break;

    case CMD_ID_GET_ENABLED:
        Foc_Pwm_LowSides();
        LOGINFO("[CMD] Motor enabled (bootstrap pre-charge)");
        break;

    case CMD_ID_GET_STOP:
        Foc_Pwm_LowSides();
        motor_data.state.Control_Mode = CONTROL_MODE_OPEN;
        RGB_DisplayColorById(0);
        LOGINFO("[CMD] Emergency stop");
        break;

    default:
        break;
    }

    cmd_tx.cmd_tx_state = (int)motor_data.state.Fault_State;
    cmd_tx.cmd_tx_vel   = motor_data.components.encoder->vel_estimate_;
    cmd_tx.cmd_tx_pos   = motor_data.components.encoder->pos_estimate_;
    cmd_tx.cmd_tx_iq    = motor_data.components.foc->i_q;
    cmd_tx.cmd_tx_vbus  = motor_data.components.foc->vbus;

    FDCANCommSend(ins, (uint8_t *)&cmd_tx);
    memset(&cmd_rx, 0, sizeof(Cmd_Rx_s));
}
