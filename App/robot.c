/**
 ******************************************************************************
 * @file    robot.c
 * @author  milFOC Team
 * @brief   Robot/Motor application implementation.
 *          Initializes all BSP components, motor control modules,
 *          and communication tasks.
 *
 * @note    This is the glue layer that wires together BSP, Modules, and App.
 *          All hardware initialization happens here (with IRQs disabled).
 ******************************************************************************
 */

#include "bsp_init.h"
#include "bsp_log.h"
#include "robot.h"
#include "robot_def.h"
#include "cmd_task.h"
#include "motor_task.h"
#include "led.h"

/**
 * @brief  Global robot/motor initialization
 *
 *         Call order is critical:
 *         1. DWT must be initialized first (used for timing)
 *         2. Wait for MT6816 encoder power-up (16ms no-output period)
 *         3. BSPInit configures DWT
 *         4. LogInit binds USART1 for debug output
 *         5. adc_bsp_init configures ADC injected group
 *         6. Foc_Pwm_Start enables PWM outputs (50% duty, safe state)
 *         7. LED indicates ready state
 *
 *         All done with IRQs disabled to prevent partial initialization.
 */
void RobotInit(void)
{
    /* Disable interrupts during initialization */
    __disable_irq();

    /* Wait for MT6816 encoder power-up (16ms no-output period) */
    DWT_Delay(0.016f);

    /* Initialize BSP layer (DWT first) */
    BSPInit();

    /* Initialize log system on USART1 */
    LogInit(&huart1);

    /* Initialize ADC for FOC current sampling */
    adc_bsp_init();

    /* Start PWM outputs (50% duty, safe pre-charge state) */
    Foc_Pwm_Start();

    /* LED: indicate ready */
    RGB_DisplayColorById(0);  /* Red = initialized, waiting for command */

    LOGINFO("[APP] RobotInit complete, IRQs enabled");

    /* Re-enable interrupts */
    __enable_irq();
}

/**
 * @brief  Periodic robot task
 * @note   Called from main loop or RTOS task at ~1kHz.
 *          Dispatches to command parsing and motor state management.
 */
void RobotTask(void)
{
    /* Command dispatch (from CAN/USB) */
    RobotCMDTask();

    /* Motor guard (protection checks) */
    MotorGuardTask(&motor_data);
}
