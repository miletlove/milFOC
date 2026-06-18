/**
 ******************************************************************************
 * @file    usart_test.h
 * @author  milFOC Team
 * @brief   USART loopback test — sends test string + echoes received bytes.
 *          Uses USART1 (PB6=TX, PB7=RX, 115200bps) with FalconFoc USART API.
 *
 * @note    USART1 is shared with bsp_log (debug output). Test messages are
 *          sent via direct USARTSend() to avoid log format overhead.
 *          Use a USB-TTL adapter to PC, open serial monitor at 115200 8N1.
 ******************************************************************************
 */

#ifndef USART_TEST_H
#define USART_TEST_H

void USART_Test_Init(void);
void USART_Test_Task(void);

#endif /* USART_TEST_H */
