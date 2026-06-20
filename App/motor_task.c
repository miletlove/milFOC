/**
 * @file    motor_task.c
 * @brief   Motor ISR — ADC JEOC callback @ 20kHz.
 *
 *          In CALC_TEST_MODE: JEOC interrupt is disabled (ADC started
 *          without IT), so this callback should never fire. If it does
 *          (e.g. debug build with IT enabled), it's a no-op.
 */

#include "motor_task.h"

void HAL_ADCEx_InjectedConvCpltCallback(ADC_HandleTypeDef *hadc)
{
#if !CALC_TEST_MODE
    if (hadc != &hadc1) return;

    GetMotorADC1PhaseCurrent(&motor_data);
#if !NO_ENCODER
    GetMotor_Angle(motor_data.components.encoder);
#endif

    MotorStateTask(&motor_data);
#else
    (void)hadc;
    /* CALC_TEST_MODE: ADC is polled in CalcTest_Step(), no ISR action needed.
     * TIM1 PWM runs at 50% duty for ADC trigger generation only.
     * FOC control loops are NOT executed from ISR context. */
#endif
}