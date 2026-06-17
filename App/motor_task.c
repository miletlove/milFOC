/**
 ******************************************************************************
 * @file    motor_task.c
 * @author  milFOC Team
 * @brief   Motor real-time task implementation.
 *          ADC injected conversion complete callback - the highest priority
 *          real-time FOC execution context at 20kHz.
 *
 * @note    CRITICAL: This ISR must complete within ~15us at 20kHz.
 *          - Priority: PreemptionPriority = 0 (highest in system)
 *          - No blocking calls, no printf, no malloc
 *          - Direct register access for minimum latency
 ******************************************************************************
 */

#include "motor_task.h"

/* Calibration mode flag (enable for debug/calibration, disable for production) */
#define ADJUST_EN 0

/**
 * @brief  ADC1 Injected Conversion Complete Callback
 *
 *         This is the heartbeat of the FOC system.
 *         Called automatically by HAL when ADC1 injected group
 *         conversion completes (triggered by TIM1 at PWM center-aligned peak).
 *
 *         Execution frequency: 20 kHz (PWM frequency)
 *         Execution time budget: <15 us
 *
 *         Data flow:
 *         ADC raw values -> phase currents (A) + bus voltage (V)
 *         -> MT6816 angle read + PLL update
 *         -> MotorStateTask() (state machine + FOC control loops)
 */
void HAL_ADCEx_InjectedConvCpltCallback(ADC_HandleTypeDef *hadc)
{
    /* Validate that this is our ADC (ADC1) */
    if (hadc != &hadc1)
    {
        return;
    }

#if ADJUST_EN
    /* Debug/calibration mode: sample only, no FOC calculation */
    GetMotorADC1PhaseCurrent(&motor_data);
    GetMotor_Angle(motor_data.components.encoder);
#else
    /* Production mode: full FOC control loop */

    /* Step 1: Read phase currents and bus voltage from ADC */
    GetMotorADC1PhaseCurrent(&motor_data);

    /* Step 2: Read encoder angle and update PLL */
    GetMotor_Angle(motor_data.components.encoder);

    /* Update FOC electrical angle from encoder */
    motor_data.components.foc->theta = motor_data.components.encoder->phase_;

    /* Step 3: Run FOC state machine and control loops */
    MotorStateTask(&motor_data);
#endif
}
