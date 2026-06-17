/**
 ******************************************************************************
 * @file    motor_adc.c
 * @author  milFOC Team
 * @brief   Motor ADC physical quantity mapping implementation.
 ******************************************************************************
 */

#include "motor_adc.h"

CURRENT_DATA current_data = {
    .hadc         = &hadc1,
    .Ia_offset    = 0.0f,
    .Ib_offset    = 0.0f,
    .Ic_offset    = 0.0f,
    .Temp_Result  = 25.0f,
};

/**
 * @brief  Convert NTC ADC reading to temperature
 * @note   Uses Steinhart-Hart equation or lookup table.
 *         TODO: Calibrate with actual NTC thermistor parameters.
 */
void GetTempNtc(uint16_t adc_value, float *temp)
{
    /* Placeholder: linear approximation
     * TODO: Implement proper NTC Steinhart-Hart or LUT */
    float voltage = (float)adc_value * 3.3f / 4096.0f;
    *temp = 25.0f + (voltage - 1.65f) * 50.0f;  /* Rough estimate */
}
