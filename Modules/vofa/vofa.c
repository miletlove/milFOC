/**
 ******************************************************************************
 * @file    vofa.c
 * @author  milFOC Team
 * @brief   VOFA+ data visualization implementation.
 *
 * @note    Uses USB CDC (Virtual COM Port) for data transmission.
 *          JustFloat protocol: each float is 4 bytes, little-endian.
 *          Frame tail: 0x00 0x00 0x80 0x7F marks end of frame.
 *
 *          Typical data packet (per frame):
 *          [i_d][i_q][v_d][v_q][theta][velocity][vbus][tail]
 *           4B   4B   4B   4B   4B      4B      4B    4B = 32 bytes
 *
 *          TODO: Implement USB CDC sending function.
 ******************************************************************************
 */

#include "vofa.h"
#include "foc_motor.h"
#include "bldc_motor.h"
#include "mt6816_encoder.h"
#include "string.h"

/* USB CDC transmit buffer */
static uint8_t vofa_tx_buf[128];

void vofa_start(void)
{
    memset(vofa_tx_buf, 0, sizeof(vofa_tx_buf));
}

void vofa_send_data(uint8_t num, float data)
{
    (void)num;
    /* TODO: Implement USB CDC send:
     * CDC_Transmit_FS((uint8_t *)&data, 4);
     */
    (void)data;
}

void vofa_sendframetail(void)
{
    float tail = 0.0f;
    /* Set tail marker bytes for VOFA JustFloat protocol */
    ((uint8_t *)&tail)[3] = 0x7F;
    ((uint8_t *)&tail)[2] = 0x80;
    ((uint8_t *)&tail)[1] = 0x00;
    ((uint8_t *)&tail)[0] = 0x00;
    /* TODO: CDC_Transmit_FS((uint8_t *)&tail, 4); */
}

/**
 * @brief  Send complete FOC debug data packet to VOFA
 *
 *         Data channels:
 *         CH1: i_d (D-axis current)
 *         CH2: i_q (Q-axis current)
 *         CH3: v_d (D-axis voltage)
 *         CH4: v_q (Q-axis voltage)
 *         CH5: theta (electrical angle)
 *         CH6: velocity (mechanical speed)
 *         CH7: vbus (bus voltage)
 */
void Vofa_Packet(void)
{
    /* Send key FOC variables for real-time visualization */
    vofa_send_data(0, motor_data.components.foc->i_d);
    vofa_send_data(1, motor_data.components.foc->i_q);
    vofa_send_data(2, motor_data.components.foc->v_d);
    vofa_send_data(3, motor_data.components.foc->v_q);
    vofa_send_data(4, motor_data.components.encoder->phase_);
    vofa_send_data(5, motor_data.components.encoder->vel_estimate_);
    vofa_send_data(6, motor_data.components.foc->vbus);
    vofa_sendframetail();
}

void vofa_Receive(uint8_t *buf, uint16_t len)
{
    /* TODO: Parse incoming VOFA commands for parameter tuning */
    (void)buf;
    (void)len;
}
