/**
 ******************************************************************************
 * @file    bsp_usart.h
 * @author  Zhang jia ming (FalconFoc) / milFOC Team
 * @brief   USART peripheral driver — multi-instance registration with DMA/IT
 *          receive callback and non-blocking send.
 ******************************************************************************
 */

#ifndef BSP_USART_H
#define BSP_USART_H

#include <stdint.h>
#include "usart.h"
#include "main.h"

#define DEVICE_USART_CNT   5
#define USART_RXBUFF_LIMIT 256

typedef void (*usart_module_callback)(void);

typedef enum
{
    USART_TRANSFER_NONE = 0,
    USART_TRANSFER_BLOCKING,
    USART_TRANSFER_IT,
    USART_TRANSFER_DMA,
} USART_TRANSFER_MODE;

typedef struct
{
    uint8_t  recv_buff[USART_RXBUFF_LIMIT];
    uint8_t  recv_buff_size;
    UART_HandleTypeDef *usart_handle;
    usart_module_callback module_callback;
} USARTInstance;

typedef struct
{
    uint8_t  recv_buff_size;
    UART_HandleTypeDef *usart_handle;
    usart_module_callback module_callback;
} USART_Init_Config_s;

void USARTServiceInit(USARTInstance *_instance);
USARTInstance *USARTRegister(USART_Init_Config_s *USART_config);
void USARTSend(USARTInstance *_instance, uint8_t *send_buf, uint16_t send_size, USART_TRANSFER_MODE mode);
uint8_t USARTIsReady(USARTInstance *_instance);

#endif /* BSP_USART_H */
