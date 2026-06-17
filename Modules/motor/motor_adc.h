/**
 ******************************************************************************
 * @file    motor_adc.h
 * @author  milFOC Team
 * @brief   Motor ADC physical quantity mapping module.
 *          Converts ADC raw values to actual currents, bus voltage,
 *          and temperature. Manages calibration offsets.
 *
 * @note    ADC1 injected group channel mapping:
 *          JDR1 = CUR_C (PA2), JDR2 = CUR_B (PA1),
 *          JDR3 = CUR_A (PA0), JDR4 = VBUS (PB1)
 ******************************************************************************
 */

#ifndef MOTOR_ADC_H
#define MOTOR_ADC_H

#include "general_def.h"
#include "adc.h"

/**
 * @brief Current measurement data structure
 */
typedef struct
{
    ADC_HandleTypeDef *hadc;        /* ADC handle (hadc1) */

    /* Calibrated offsets (set during CURRENT_CALIBRATING) */
    float Ia_offset;
    float Ib_offset;
    float Ic_offset;

    /* Offset calibration accumulators */
    float current_offset_sum_a;
    float current_offset_sum_b;
    float current_offset_sum_c;

    /* Temperature */
    float Temp_Result;              /* NTC temperature [degC] */

} CURRENT_DATA;

extern CURRENT_DATA current_data;

/**
 * @brief  Convert NTC ADC raw value to temperature
 * @param  adc_value: raw ADC reading
 * @param  temp: output temperature [degC]
 */
void GetTempNtc(uint16_t adc_value, float *temp);

#endif /* MOTOR_ADC_H */
