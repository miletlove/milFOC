/**
 ******************************************************************************
 * @file    bsp_init.c
 * @author  milFOC Team
 * @brief   Unified BSP hardware initialization implementation.
 *
 *          Consolidates initialization that was previously in main.c:
 *          DWT, TIM1 PWM, CCR preload, ADC injected group.
 ******************************************************************************
 */

#include "bsp_init.h"
#include "bldc_motor.h"
#include "adc.h"
#include "tim.h"

/**
 * @brief  Unified BSP initialization — call once after MX_*_Init()
 */
void BSP_Init(void)
{
    /* ── 1. DWT cycle counter for μs-precision timing ── */
    DWT_Init(168);  /* STM32G431 @ 168 MHz */

    /* ── 2. Start TIM1 PWM @ 50% neutral ──
     * CH1/CH2/CH3: motor phases, start at 50% (zero net voltage)
     * CH4: OC_REF triggers ADC1 injected group at PWM center */
    Foc_Pwm_Start();

    /* ── 3. Enable TIM1 CCR preload for atomic 3-phase update ──
     * Without preload (OCxPE=0): sequential CCR1→CCR2→CCR3 writes
     *   can span a PWM UPDATE event → phase tearing (duty mismatch).
     * With preload (OCxPE=1): writes go to shadow registers,
     *   transferred atomically at next UPDATE event.
     *
     * CCMR1 bit3=OC1PE(CH1), bit11=OC2PE(CH2)
     * CCMR2 bit3=OC3PE(CH3) */
    TIM1->CCMR1 |= TIM_CCMR1_OC1PE | TIM_CCMR1_OC2PE;
    TIM1->CCMR2 |= TIM_CCMR2_OC3PE;
}
