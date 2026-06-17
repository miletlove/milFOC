/**
 ******************************************************************************
 * @file    bsp_dwt.h
 * @author  Wang Hongxi (original) / milFOC Team (adapted)
 * @brief   DWT (Data Watchpoint and Trigger) high-precision cycle counter driver
 *          Provides nanosecond-level non-blocking delay and algorithm profiling.
 * @note    Core clock: 168 MHz for STM32G431
 ******************************************************************************
 */

#ifndef BSP_DWT_H
#define BSP_DWT_H

#include "main.h"
#include "stdint.h"

typedef struct
{
    uint32_t s;
    uint16_t ms;
    uint16_t us;
} DWT_Time_t;

void DWT_Init(uint32_t CPU_Freq_mHz);
float DWT_GetDeltaT(uint32_t *cnt_last);
double DWT_GetDeltaT64(uint32_t *cnt_last);
float DWT_GetTimeline_s(void);
float DWT_GetTimeline_ms(void);
uint64_t DWT_GetTimeline_us(void);
void DWT_Delay(float Delay);
void DWT_SysTimeUpdate(void);

extern DWT_Time_t SysTime;

#endif /* BSP_DWT_H */
