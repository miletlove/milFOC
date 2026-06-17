/**
 ******************************************************************************
 * @file    bsp_adc.c
 * @author  milFOC Team
 * @brief   ADC BSP implementation for FOC current sampling.
 *          Configures ADC1/ADC2 injected group triggered by TIM1.
 ******************************************************************************
 */

#include "bsp_adc.h"
#include "bsp_log.h"

/* DMA buffers for ADC1 and ADC2 */
uint16_t adc1_dma_value[ADC1_SAMPLES][ADC1_CHANNELS];
uint16_t adc2_dma_value[ADC2_SAMPLES][ADC2_CHANNELS];

/**
 * @brief  Initialize ADC for FOC current sampling
 * @note   Call once before starting FOC state machine.
 *         Configures:
 *         - ADC1 injected group for Ia, Ib, Ic, Vbus on JDR1~JDR4
 *         - TIM1 TRGO as trigger source
 *         - Injected conversion interrupt (JEOC) for FOC scheduling
 */
void adc_bsp_init(void)
{
    /* Start ADC1 DMA for regular conversions (if used for monitoring) */
    HAL_ADC_Start_DMA(&hadc1, (uint32_t *)adc1_dma_value, ADC1_BUF_LEN);

#if ADC_INJECTED_ENABLE
    /* Configure ADC1 injected group:
     * JDR1 = CUR_C (PA2)
     * JDR2 = CUR_B (PA1)
     * JDR3 = CUR_A (PA0)
     * JDR4 = VBUS  (PB1)
     * Triggered by TIM1 TRGO at PWM center-aligned peak
     */
    // Note: The injected channel mapping is done in CubeMX (.ioc file)
    // This function starts the injected conversion and enables JEOC interrupt.

    HAL_ADCEx_InjectedStart_IT(&hadc1);
#endif

    LOGINFO("[ADC] ADC BSP initialized, injected mode ready");
}
