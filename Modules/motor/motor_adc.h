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
#include "bsp_adc.h"

/* ADC channel number enums (FalconFoc compatible) */
typedef enum
{
    adc1_ch1 = 0,
    adc1_ch2 = 1,
    adc1_ch3 = 2,
    adc1_ch4 = 3
} adc1_num;

typedef enum
{
    adc2_ch12 = 0,
} adc2_num;

/**
 * @brief Current measurement data structure
 */
typedef struct CURRENT_DATA
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

/**
 * @brief  ADC1 median filter (noise rejection)
 * @param  channel: ADC1 channel index (adc1_ch1..ch4)
 * @return filtered ADC value
 */
uint16_t adc1_median_filter(uint8_t channel);

/**
 * @brief  ADC1 average filter (ripple reduction)
 * @param  channel: ADC1 channel index
 * @return filtered ADC value
 */
uint16_t adc1_avg_filter(uint8_t channel);

/**
 * @brief  ADC2 median filter
 */
uint16_t adc2_median_filter(uint8_t channel);

/**
 * @brief  ADC2 average filter
 */
uint16_t adc2_avg_filter(uint8_t channel);

#endif /* MOTOR_ADC_H */
