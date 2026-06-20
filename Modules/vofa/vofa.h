/**
 * @brief  VOFA+ FireWater protocol interface.
 *          Sends FOC transform results (v_a/v_b/v_c/i_alpha/i_beta/i_q)
 *          to VOFA+上位机 via USB CDC at ~50Hz.
 *
 *          VOFA+ config: 协议 = FireWater, 端口 = STM32 Virtual COM.
 */

#ifndef VOFA_H
#define VOFA_H

#include "general_def.h"
#include "usart.h"
#include "usbd_cdc_if.h"

#define VOFA_FW_BUF_SIZE   128

void vofa_firewater_send(float *data, uint8_t count);
void Vofa_Packet(void);
void vofa_Receive(uint8_t *buf, uint16_t len);

#endif /* VOFA_H */
