/**
 ******************************************************************************
 * @file    bsp_init.h
 * @author  milFOC Team
 * @brief   Unified BSP hardware initialization entry point.
 *
 *          Consolidates all post-CubeMX hardware setup that was previously
 *          scattered in main.c:
 *          - DWT cycle counter (μs-precision timing)
 *          - TIM1 PWM start + CCR preload (atomic 3-phase update)
 *          - ADC1 injected group start (TIM1-triggered, polled)
 *
 * @note    Call BSP_Init() once after all MX_*_Init() in main.c.
 ******************************************************************************
 */

#ifndef BSP_INIT_H
#define BSP_INIT_H

#include "main.h"
#include "bsp_dwt.h"

void BSP_Init(void);

#endif /* BSP_INIT_H */
