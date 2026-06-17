/**
 ******************************************************************************
 * @file    bsp_usart.h
 * @author  milFOC Team
 * @brief   USART Board Support Package - Multi-instance serial abstraction.
 *          Supports DMA and IT transfer modes.
 *
 * @note    Hardware: USART1 on PB6 (TX) / PB7 (RX)
 ******************************************************************************
 */

#ifndef BSP_USART_H
#define BSP_USART_H

#include "general_def.h"
#include "usart.h"

/* Transfer mode enumeration */
typedef enum
{
    USART_TRANSFER_NONE = 0,
    USART_TRANSFER_IT,      /* Interrupt-based transfer */
    USART_TRANSFER_DMA,     /* DMA-based transfer */
} USART_TransferMode;

/* USART instance structure */
typedef struct
{
    UART_HandleTypeDef *usart_handle;
    USART_TransferMode tx_mode;
    USART_TransferMode rx_mode;
    uint8_t rx_buff[256];
    uint16_t rx_len;
} USARTInstance;

/* USART init configuration */
typedef struct
{
    UART_HandleTypeDef *usart_handle;
} USART_Init_Config_s;

/* --- Public API --- */

/**
 * @brief  Register a USART instance
 * @param  config: pointer to init configuration
 * @return Pointer to registered USARTInstance
 */
USARTInstance *USARTRegister(USART_Init_Config_s *config);

/**
 * @brief  Send data via USART
 * @param  instance: target USART instance
 * @param  data: pointer to data buffer
 * @param  len: data length in bytes
 * @param  mode: transfer mode (IT or DMA)
 */
void USARTSend(USARTInstance *instance, uint8_t *data, uint16_t len, USART_TransferMode mode);

#endif /* BSP_USART_H */
