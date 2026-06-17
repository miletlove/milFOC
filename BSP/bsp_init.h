/**
 ******************************************************************************
 * @file    bsp_init.h
 * @author  milFOC Team
 * @brief   BSP layer unified initialization entry point.
 *          Initializes only essential BSP components (DWT).
 *          Other BSP peripherals (CAN, USART, etc.) are initialized by their
 *          respective module registrations.
 *
 * @note    Must be called BEFORE RTOS starts and BEFORE any interrupts enabled.
 *          Currently called by RobotInit() in App layer.
 ******************************************************************************
 */

#ifndef BSP_INIT_H
#define BSP_INIT_H

#include "bsp_dwt.h"
#include "bsp_adc.h"
#include "bsp_can.h"
#include "bsp_usart.h"
#include "bsp_log.h"
#include "bsp_flash.h"

/**
 * @brief  Unified BSP initialization
 * @note   Initializes DWT only. Other BSP components are initialized
 *         on-demand via their respective Register/Init functions.
 */
static inline void BSPInit(void)
{
    /* Initialize DWT with 168 MHz core clock */
    DWT_Init(168);
}

#endif /* BSP_INIT_H */
