/**
 ******************************************************************************
 * @file    bsp_usart.c
 * @author  milFOC Team
 * @brief   USART BSP implementation - Multi-instance serial abstraction.
 *          Adapted from FalconFoc BSP layer.
 ******************************************************************************
 */

#include "bsp_usart.h"
#include "bsp_log.h"
#include "string.h"

#define USART_MX_REGISTER_CNT 4
static USARTInstance *usart_instance[USART_MX_REGISTER_CNT] = {NULL};
static uint8_t usart_idx;

/**
 * @brief  Register a USART instance
 */
USARTInstance *USARTRegister(USART_Init_Config_s *config)
{
    if (usart_idx >= USART_MX_REGISTER_CNT)
    {
        LOGERROR("[USART] Max instances reached (%d)", USART_MX_REGISTER_CNT);
        return NULL;
    }

    USARTInstance *ins = (USARTInstance *)malloc(sizeof(USARTInstance));
    memset(ins, 0, sizeof(USARTInstance));
    ins->usart_handle = config->usart_handle;
    ins->tx_mode = USART_TRANSFER_NONE;
    ins->rx_mode = USART_TRANSFER_NONE;

    usart_instance[usart_idx++] = ins;
    return ins;
}

/**
 * @brief  Send data via USART (IT or DMA mode)
 */
void USARTSend(USARTInstance *instance, uint8_t *data, uint16_t len, USART_TransferMode mode)
{
    if (instance == NULL || data == NULL || len == 0) return;

    switch (mode)
    {
    case USART_TRANSFER_IT:
        HAL_UART_Transmit_IT(instance->usart_handle, data, len);
        break;
    case USART_TRANSFER_DMA:
        HAL_UART_Transmit_DMA(instance->usart_handle, data, len);
        break;
    default:
        break;
    }
}
