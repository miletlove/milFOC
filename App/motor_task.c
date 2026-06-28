/**
 * @file    motor_task.c
 * @brief   Motor ISR — ADC JEOC callback @ 20kHz.
 *
 *          In bringup IDLE/PWM_TEST/LOCK/OPEN_LOOP modes:
 *          ADC is polled from main loop (JEOC interrupt disabled),
 *          so this callback acts as a no-op.
 */

#include "motor_task.h"

void HAL_ADCEx_InjectedConvCpltCallback(ADC_HandleTypeDef *hadc)
{
    (void)hadc;
    /* Bring-Up mode: ADC is polled in BringUp_Step() via main loop.
     * JEOC interrupt is disabled. This callback is a no-op safety net. */
}