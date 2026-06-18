/**
 ******************************************************************************
 * @file    vofa_test.c
 * @author  milFOC Team
 * @brief   VOFA+ connectivity test — sends synthetic sine waves via USB CDC.
 *
 *          Data format (JustFloat, little-endian):
 *            CH1: sin(2pi * 10Hz * t)        [float, 4B]
 *            CH2: sin(2pi * 5Hz  * t)        [float, 4B]
 *            CH3: sawtooth 0~3.3             [float, 4B]
 *            CH4: 3.3 (constant reference)    [float, 4B]
 *            tail: 0x00 0x00 0x80 0x7F
 *
 *          Frame rate: ~100Hz (every 10ms call from main loop)
 *          Frame size: 5 x 4B = 20 bytes
 ******************************************************************************
 */

#include "vofa_test.h"
#include "usbd_cdc_if.h"
#include "main.h"
#include "math.h"
#include "string.h"

#define VOFA_FRAME_RATE_HZ 100

static float t_sec = 0.0f;

void VOFA_Test_Init(void)
{
    t_sec = 0.0f;
}

void VOFA_Test_Task(void)
{
    static uint32_t last_tick = 0;
    uint32_t now = HAL_GetTick();

    if (now - last_tick < (1000 / VOFA_FRAME_RATE_HZ))
        return;
    last_tick = now;
    t_sec += 1.0f / (float)VOFA_FRAME_RATE_HZ;

    /* Generate test waveforms */
    float ch1 = sinf(2.0f * 3.14159265f * 10.0f * t_sec);    /* 10 Hz sine */
    float ch2 = sinf(2.0f * 3.14159265f * 5.0f  * t_sec);    /*  5 Hz sine */
    float ch3 = fmodf(t_sec * 3.3f, 3.3f);                   /* Sawtooth    */
    float ch4 = 3.3f;                                         /* Constant    */
    float tail = 0.0f;

    /* Encode JustFloat frame tail: 0x00 0x00 0x80 0x7F */
    ((uint8_t *)&tail)[0] = 0x00;
    ((uint8_t *)&tail)[1] = 0x00;
    ((uint8_t *)&tail)[2] = 0x80;
    ((uint8_t *)&tail)[3] = 0x7F;

    /* Pack frame: 4 channels + tail = 20 bytes */
    uint8_t frame[20];
    memcpy(frame +  0, &ch1,  4);
    memcpy(frame +  4, &ch2,  4);
    memcpy(frame +  8, &ch3,  4);
    memcpy(frame + 12, &ch4,  4);
    memcpy(frame + 16, &tail, 4);

    CDC_Transmit_FS(frame, sizeof(frame));
    HAL_GPIO_TogglePin(LED_GPIO_Port, LED_Pin);
}
