/**
 * @file    motor_task.c
 * @brief   Motor ISR — ADC JEOC callback @ 20kHz.
 */

#include "motor_task.h"

void HAL_ADCEx_InjectedConvCpltCallback(ADC_HandleTypeDef *hadc)
{
    if (hadc != &hadc1) return;

    GetMotorADC1PhaseCurrent(&motor_data);
#if !NO_ENCODER
    GetMotor_Angle(motor_data.components.encoder);
#endif

    MotorStateTask(&motor_data);
}