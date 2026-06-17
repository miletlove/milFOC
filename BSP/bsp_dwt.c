/**
 ******************************************************************************
 * @file    bsp_dwt.c
 * @author  Wang Hongxi (original) / milFOC Team (adapted)
 * @brief   DWT high-precision timer implementation.
 *          Enables DWT CYCCNT for sub-microsecond timing.
 * @note    Must be initialized before any delay/timing usage.
 ******************************************************************************
 */

#include "bsp_dwt.h"

DWT_Time_t SysTime;
static uint32_t CPU_FREQ_Hz, CPU_FREQ_Hz_ms, CPU_FREQ_Hz_us;
static uint32_t CYCCNT_RountCount;
static uint32_t CYCCNT_LAST;
uint64_t CYCCNT64;
static void DWT_CNT_Update(void);

/**
 * @brief  Initialize DWT cycle counter
 * @param  CPU_Freq_mHz: CPU core frequency in MHz (e.g. 168 for STM32G431)
 */
void DWT_Init(uint32_t CPU_Freq_mHz)
{
    /* Enable DWT peripheral */
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;

    /* Clear DWT CYCCNT register */
    DWT->CYCCNT       = (uint32_t)0u;

    /* Enable Cortex-M DWT CYCCNT register */
    DWT->CTRL        |= DWT_CTRL_CYCCNTENA_Msk;
    CPU_FREQ_Hz       = CPU_Freq_mHz * 1000000;
    CPU_FREQ_Hz_ms    = CPU_FREQ_Hz / 1000;
    CPU_FREQ_Hz_us    = CPU_FREQ_Hz / 1000000;
    CYCCNT_RountCount = 0;
}

/**
 * @brief  Update internal overflow counter for 64-bit timebase
 */
static void DWT_CNT_Update(void)
{
    volatile uint32_t cnt_now = DWT->CYCCNT;
    if (cnt_now < CYCCNT_LAST)
    {
        CYCCNT_RountCount++;
    }
    CYCCNT_LAST = cnt_now;
}

/**
 * @brief  Get time delta since last call (float, seconds)
 */
float DWT_GetDeltaT(uint32_t *cnt_last)
{
    volatile uint32_t cnt_now = DWT->CYCCNT;
    float dt = ((uint32_t)(cnt_now - *cnt_last)) / ((float)(CPU_FREQ_Hz));
    *cnt_last = cnt_now;
    DWT_CNT_Update();
    return dt;
}

/**
 * @brief  Get time delta since last call (double, seconds)
 */
double DWT_GetDeltaT64(uint32_t *cnt_last)
{
    volatile uint32_t cnt_now = DWT->CYCCNT;
    double dt = ((uint32_t)(cnt_now - *cnt_last)) / ((double)(CPU_FREQ_Hz));
    *cnt_last = cnt_now;
    DWT_CNT_Update();
    return dt;
}

/**
 * @brief  Update system time structure (seconds, ms, us)
 */
void DWT_SysTimeUpdate(void)
{
    volatile uint32_t cnt_now = DWT->CYCCNT;
    static uint64_t CNT_TEMP1, CNT_TEMP2, CNT_TEMP3;

    DWT_CNT_Update();

    CYCCNT64   = (uint64_t)CYCCNT_RountCount * (uint64_t)UINT32_MAX + (uint64_t)cnt_now;
    CNT_TEMP1  = CYCCNT64 / CPU_FREQ_Hz;
    CNT_TEMP2  = CYCCNT64 - CNT_TEMP1 * CPU_FREQ_Hz;
    SysTime.s  = CNT_TEMP1;
    SysTime.ms = CNT_TEMP2 / CPU_FREQ_Hz_ms;
    CNT_TEMP3  = CNT_TEMP2 - SysTime.ms * CPU_FREQ_Hz_ms;
    SysTime.us = CNT_TEMP3 / CPU_FREQ_Hz_us;
}

float DWT_GetTimeline_s(void)
{
    DWT_SysTimeUpdate();
    return SysTime.s + SysTime.ms * 0.001f + SysTime.us * 0.000001f;
}

float DWT_GetTimeline_ms(void)
{
    DWT_SysTimeUpdate();
    return SysTime.s * 1000 + SysTime.ms + SysTime.us * 0.001f;
}

uint64_t DWT_GetTimeline_us(void)
{
    DWT_SysTimeUpdate();
    return SysTime.s * 1000000 + SysTime.ms * 1000 + SysTime.us;
}

/**
 * @brief  Non-blocking microsecond delay using DWT cycle counter
 * @param  Delay: delay time in microseconds (us)
 */
void DWT_Delay(float Delay)
{
    uint32_t cnt_last = DWT->CYCCNT;
    float cur_delay = 0.0f;
    while (cur_delay < Delay)
    {
        cur_delay += DWT_GetDeltaT(&cnt_last);
        cur_delay *= 1e6f; /* convert from s to us */
    }
}
