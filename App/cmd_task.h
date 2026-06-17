/**
 ******************************************************************************
 * @file    cmd_task.h
 * @author  milFOC Team
 * @brief   Command dispatch task header.
 *          Parses incoming CAN/USB commands and dispatches to motor control.
 *
 * @note    Called at ~200Hz (or whenever new command arrives).
 ******************************************************************************
 */

#ifndef CMD_TASK_H
#define CMD_TASK_H

/**
 * @brief  Initialize command dispatch (CAN driver registration)
 */
void RobotCMDInit(void);

/**
 * @brief  Command dispatch task - parse and execute incoming commands
 * @note   Run at ~200Hz (must be faster than host command rate)
 */
void RobotCMDTask(void);

#endif /* CMD_TASK_H */
