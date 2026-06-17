/**
 ******************************************************************************
 * @file    bsp_adc.h
 * @author  milFOC Team
 * @brief   ADC Board Support Package - Injected group configuration for FOC
 *          current sampling on STM32G431.
 *
 * @note    Hardware mapping:
 *          ADC1: CUR_A (PA0), CUR_B (PA1), CUR_C (PA2), VBUS (PB1)
 *          Triggered by TIM1 for synchronous center-aligned sampling.
 *          Dual ADC (ADC1 + ADC2) parallel sampling supported.
 ******************************************************************************
 */

#ifndef BSP_ADC_H
#define BSP_ADC_H

#include "general_def.h"
#include "adc.h"
#include "tim.h"

/* ADC Injected mode enable flag */
#define ADC_INJECTED_ENABLE 1

/* ADC1 DMA buffer parameters */
#define ADC1_SAMPLES 5    /* Oversampling count per channel */
#define ADC1_CHANNELS 1   /* Number of channels */
#define ADC1_BUF_LEN (ADC1_SAMPLES * ADC1_CHANNELS)

/* ADC2 buffer parameters (for temperature NTC, reserved) */
#define ADC2_SAMPLES 5
#define ADC2_CHANNELS 1
#define ADC2_BUF_LEN (ADC2_SAMPLES * ADC2_CHANNELS)

/* ADC conversion factors (to be tuned per hardware) */
#define FAC_CURRENT       (3.3f / 4096.0f / 0.01f)  /* ADC raw -> Amps (via shunt R & opamp gain) */
#define VOLTAGE_TO_ADC_FACTOR (3.3f / 4096.0f * 11.0f) /* ADC raw -> Bus voltage */

extern uint16_t adc1_dma_value[ADC1_SAMPLES][ADC1_CHANNELS];
extern uint16_t adc2_dma_value[ADC2_SAMPLES][ADC2_CHANNELS];

/**
 * @brief  Initialize ADC for FOC current sampling
 *         Configures injected group, trigger source (TIM1), and DMA.
 */
void adc_bsp_init(void);

#endif /* BSP_ADC_H */
