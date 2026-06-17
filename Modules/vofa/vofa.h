/**
 ******************************************************************************
 * @file    vofa.h
 * @author  milFOC Team
 * @brief   VOFA+ data visualization interface.
 *          Sends real-time FOC variables to VOFA+上位机 via USART/USB CDC
 *          using JustFloat protocol for high-speed data streaming.
 *
 * @note    Protocol: 4-byte float, little-endian, no frame header.
 *          Frame tail: 0x00 0x00 0x80 0x7F (optional).
 ******************************************************************************
 */

#ifndef VOFA_H
#define VOFA_H

#include "general_def.h"
#include "usart.h"
#include "usbd_cdc_if.h"

/* Byte extraction macros for JustFloat protocol */
#define byte0(dw_temp)  (*(char *)(&dw_temp))
#define byte1(dw_temp)  (*((char *)(&dw_temp) + 1))
#define byte2(dw_temp)  (*((char *)(&dw_temp) + 2))
#define byte3(dw_temp)  (*((char *)(&dw_temp) + 3))

/**
 * @brief  Initialize VOFA data streaming
 */
void vofa_start(void);

/**
 * @brief  Send a single float value via VOFA
 * @param  num: channel number (reserved)
 * @param  data: float value to send
 */
void vofa_send_data(uint8_t num, float data);

/**
 * @brief  Send frame tail marker
 */
void vofa_sendframetail(void);

/**
 * @brief  Send a complete VOFA data packet (all FOC variables)
 */
void Vofa_Packet(void);

/**
 * @brief  Receive data from VOFA (for parameter tuning)
 * @param  buf: received data buffer
 * @param  len: data length
 */
void vofa_Receive(uint8_t *buf, uint16_t len);

#endif /* VOFA_H */
