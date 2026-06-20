/**
 ******************************************************************************
 * @file    motor_task.h
 * @author  milFOC Team
 * @brief   Motor real-time task header.
 *          Provides the ADC injected conversion complete callback (JEOC)
 *          which is the highest-priority real-time FOC execution context.
 *
 * @note    This is where the 20kHz FOC current loop executes.
 *          The callback fires when ADC1 injected group conversion completes.
 *          Execution time budget: <15us @ 20kHz.
 ******************************************************************************
 */

#ifndef MOTOR_TASK_H
#define MOTOR_TASK_H

#include "general_def.h"
#include "foc_motor.h"
#include "bldc_motor.h"
#include "mt6816_encoder.h"
#include "motor_adc.h"

#define NO_ENCODER       1    /* 1=virtual angle, 0=MT6816 (PLL) */

/**
 * @brief  ADC1 Injected Conversion Complete Callback
 *
 *         Called by HAL when ADC1 injected group conversion finishes.
 *         This ISR is the core FOC execution context at 20kHz.
 *
 *         Execution order:
 *         1. Read ADC injected data (phase currents + bus voltage)
 *         2. Read MT6816 encoder angle (update PLL)
 *         3. Run motor state machine (FOC control loops)
 *
 * @note   This callback overrides the weak HAL default.
 *         Preemption priority must be 0 (highest).
 *         Do NOT add debug logging or blocking operations here!
 */
void HAL_ADCEx_InjectedConvCpltCallback(ADC_HandleTypeDef *hadc);

#endif /* MOTOR_TASK_H */
