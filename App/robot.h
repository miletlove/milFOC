/**
 ******************************************************************************
 * @file    robot.h
 * @author  milFOC Team
 * @brief   Robot/Motor main application entry point.
 *          Provides RobotInit() for hardware initialization and
 *          RobotTask() for periodic scheduling (when using RTOS).
 *
 * @note    RobotInit() must be called in main() BEFORE any RTOS starts.
 *          RobotTask() should be called periodically in RTOS or main loop.
 ******************************************************************************
 */

#ifndef ROBOT_H
#define ROBOT_H

/**
 * @brief  Robot/motor global initialization.
 *         Call once in main() before enabling interrupts or starting RTOS.
 *         Performs:
 *         - DWT initialization
 *         - Log system init
 *         - ADC BSP init
 *         - PWM start
 *         - LED init
 */
void RobotInit(void);

/**
 * @brief  Robot/motor periodic task.
 *         Call at fixed frequency (e.g. 1kHz) in RTOS or main loop.
 *         Dispatches to:
 *         - RobotCMDTask(): command parsing
 *         - RobotMotorTask(): motor state machine (if not interrupt-driven)
 */
void RobotTask(void);

#endif /* ROBOT_H */
