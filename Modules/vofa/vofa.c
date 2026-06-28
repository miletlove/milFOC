/**
 * @file    vofa.c
 * @brief   VOFA+ FireWater — USART1 blocking TX (PB6) or USB CDC.
 */

#include "vofa.h"
#include "foc_motor.h"
#include "bldc_motor.h"
#include "mt6816_encoder.h"
#include "motor_task.h"
#include "usbd_cdc_if.h"
#include "string.h"

/* 0=USB CDC (PA11/PA12), 1=USART1 blocking TX (PB6) */
#define VOFA_USE_USART1   0

static char vofa_buf[VOFA_FW_BUF_SIZE];

static int f32_to_str(char *buf, size_t size, float val)
{
    if (buf == NULL || size < 8) return 0;
    int neg = (val < 0.0f);
    if (neg) val = -val;
    if (val > 99999.0f) val = 99999.0f;
    uint32_t i = (uint32_t)val;
    uint32_t f = (uint32_t)((val - (float)i) * 10000.0f + 0.5f);
    if (f >= 10000) { i++; f = 0; }
    char t[16]; int n = 0;
    if (neg) t[n++] = '-';
    if (i == 0) t[n++] = '0';
    else { char r[8]; uint8_t k = 0;
           while (i) { r[k++] = '0'+(i%10); i/=10; }
           while (k) t[n++] = r[--k]; }
    t[n++] = '.'; t[n++] = '0'+(f/1000%10); t[n++] = '0'+(f/100%10);
    t[n++] = '0'+(f/10%10); t[n++] = '0'+(f%10); t[n] = '\0';
    size_t c = (n < (int)size-1) ? n : size-1;
    memcpy(buf, t, c); buf[c] = '\0';
    return (int)c;
}

void vofa_firewater_send(float *data, uint8_t count)
{
    if (!data || !count) return;
    char *p = vofa_buf, *e = vofa_buf + sizeof(vofa_buf) - 1;
    for (uint8_t i = 0; i < count; i++) {
        if (p >= e - 16) break;
        int n = f32_to_str(p, e - p, data[i]);
        if (n <= 0) n = 1;
        p += n; if (p < e) *p++ = ',';
    }
    if (p > vofa_buf && *(p-1) == ',') *(p-1) = '\n';
    else if (p < e) *p++ = '\n';

#if VOFA_USE_USART1
    HAL_UART_Transmit(&huart1, (uint8_t *)vofa_buf, p - vofa_buf, 10);
#else
    CDC_Transmit_FS((uint8_t *)vofa_buf, p - vofa_buf);
#endif
}

void Vofa_Packet(void)
{
#if ENCODER_TEST
    /* Encoder diagnostic: raw angle + parity check */
    float fw[6];
    fw[0] = (float)motor_data.components.encoder->raw;            /* CH1: raw angle 0~16383 */
    fw[1] = (float)motor_data.components.encoder->check_err_count;/* CH2: parity err */
    fw[2] = (float)motor_data.components.encoder->rx_err_count;   /* CH3: SPI err */
    fw[3] = motor_data.components.foc->v_a;                       /* CH4: v_a */
    fw[4] = motor_data.components.foc->i_d;                       /* CH5: Park d */
    fw[5] = motor_data.components.foc->i_q;                       /* CH6: Park q */
    vofa_firewater_send(fw, 6);
#else
    float fw[6];
    fw[0] = motor_data.components.foc->v_a;         /* CH1: A相电压 */
    fw[1] = motor_data.components.foc->v_b;         /* CH2: B相电压 */
    fw[2] = motor_data.components.foc->v_c;         /* CH3: C相电压 */
    fw[3] = motor_data.components.foc->i_d;         /* CH4: Park d */
    fw[4] = motor_data.components.foc->i_q;         /* CH5: Park q */
    fw[5] = motor_data.components.foc->vbus;        /* CH6: Bus voltage */
    vofa_firewater_send(fw, 6);
#endif
}

void vofa_Receive(uint8_t *buf, uint16_t len)
{
    (void)buf;
    (void)len;
}
