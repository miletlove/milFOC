/**
 ******************************************************************************
 * @file    bsp_usart.c
 * @author  Zhang jia ming (FalconFoc) / milFOC Team
 * @brief   USART BSP implementation — multi-instance DMA/IT/BLOCKING send.
 ******************************************************************************
 */

#include "bsp_usart.h"
#include "bsp_log.h"
#include <stdlib.h>
#include <string.h>

static uint8_t idx;
static USARTInstance *usart_instance[DEVICE_USART_CNT] = {NULL};

void USARTServiceInit(USARTInstance *_instance)
{
    HAL_UARTEx_ReceiveToIdle_DMA(_instance->usart_handle, _instance->recv_buff, _instance->recv_buff_size);
    __HAL_DMA_DISABLE_IT(_instance->usart_handle->hdmarx, DMA_IT_HT);
}

USARTInstance *USARTRegister(USART_Init_Config_s *USART_config)
{
    if (idx >= DEVICE_USART_CNT)
    {
        while (1)
            LOGERROR("[bsp_usart] USART exceed max instance count!");
    }
    for (uint8_t i = 0; i < idx; i++)
    {
        if (usart_instance[i]->usart_handle == USART_config->usart_handle)
        {
            while (1)
                LOGERROR("[bsp_usart] USART instance already registered!");
        }
    }

    USARTInstance *usart = (USARTInstance *)malloc(sizeof(USARTInstance));
    memset(usart, 0, sizeof(USARTInstance));

    usart->usart_handle    = USART_config->usart_handle;
    usart->recv_buff_size  = USART_config->recv_buff_size;
    usart->module_callback = USART_config->module_callback;

    usart_instance[idx++] = usart;
    USARTServiceInit(usart);
    return usart;
}

void USARTSend(USARTInstance *_instance, uint8_t *send_buf, uint16_t send_size, USART_TRANSFER_MODE mode)
{
    switch (mode)
    {
    case USART_TRANSFER_BLOCKING:
        HAL_UART_Transmit(_instance->usart_handle, send_buf, send_size, 100);
        break;
    case USART_TRANSFER_IT:
        HAL_UART_Transmit_IT(_instance->usart_handle, send_buf, send_size);
        break;
    case USART_TRANSFER_DMA:
        HAL_UART_Transmit_DMA(_instance->usart_handle, send_buf, send_size);
        break;
    default:
        break;
    }
}

uint8_t USARTIsReady(USARTInstance *_instance)
{
    if (_instance->usart_handle->gState & HAL_UART_STATE_BUSY_TX)
        return 0;
    else
        return 1;
}

void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
{
    (void)Size;
    for (uint8_t i = 0; i < idx; ++i)
    {
        if (huart == usart_instance[i]->usart_handle)
        {
            if (usart_instance[i]->module_callback != NULL)
                usart_instance[i]->module_callback();
            USARTServiceInit(usart_instance[i]);
            return;
        }
    }
}
