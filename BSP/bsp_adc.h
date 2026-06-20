/*
 * @Author: Yangzhi_Liu 3068126392@qq.com
 * @Date: 2026-06-18 00:55:25
 * @LastEditors: Yangzhi_Liu 3068126392@qq.com
 * @LastEditTime: 2026-06-20 03:16:27
 * @FilePath: \milFOC\BSP\bsp_adc.h
 * @Description: 这是默认设置,请设置`customMade`, 打开koroFileHeader查看配置 进行设置: https://github.com/OBKoro1/koro1FileHeader/wiki/%E9%85%8D%E7%BD%AE
 */
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

/* ADC hardware parameters (FalconFoc compatible) */
/* --- Current Sensing --- */
#define V_REG             1.65f             /* ADC reference midpoint [V] */
#define CURRENT_SHUNT_RES 0.002f            /* Shunt resistor [Ohm] — 2mΩ */
#define CURRENT_AMP_GAIN  50.0f             /* Op-amp gain [V/V] */
#define FAC_CURRENT       ((3.3f / 4095.0f) / (CURRENT_SHUNT_RES * CURRENT_AMP_GAIN))
                                            /* ADC raw -> Amps: ~0.806 mA/LSB */

/* --- Bus Voltage Sensing (Resistive Divider) --- */
#define VIN_R1            1000.0f           /* Top resistor [Ohm] */
#define VIN_R2            10000.0f          /* Bottom resistor [Ohm] */
#define VOLTAGE_TO_ADC_FACTOR (((VIN_R2 + VIN_R1) / VIN_R1) * (3.3f / 4095.0f))
                                            /* ADC raw -> Bus voltage [V] */

extern uint16_t adc1_dma_value[ADC1_SAMPLES][ADC1_CHANNELS];
extern uint16_t adc2_dma_value[ADC2_SAMPLES][ADC2_CHANNELS];

/**
 * @brief  Initialize ADC for FOC current sampling
 *         Configures injected group, trigger source (TIM1), and DMA.
 */
void adc_bsp_init(void);

#endif /* BSP_ADC_H */
