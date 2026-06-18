/**
 ******************************************************************************
 * @file    usart_test.c
 * @author  milFOC Team
 * @brief   USART TX/RX loopback test using FalconFoc BSP USART API.
 *
 *          Test procedure:
 *          1. PC serial monitor: 115200 8N1, connect to USART1 (PB6/PB7)
 *          2. Board sends "milFOC USART OK\r\n" every 1s (heartbeat)
 *          3. Any byte received is echoed back immediately
 *
 *          Hardware: USB-TTL adapter → PB6(TX), PB7(RX), GND
 ******************************************************************************
 */

#include "usart_test.h"
#include "bsp_usart.h"
#include "main.h"
#include "string.h"
#include "stdio.h"

static USARTInstance *test_usart;

/**
 * @brief  USART RX callback — echo received byte back.
 */
static void usart_test_callback(void)
{
    /* Echo the received data byte-by-byte.
     * FalconFoc USARTServiceInit uses DMA + IDLE, so we get the full packet.
     * For simplicity here, just send back what was received. */
    USARTSend(test_usart, test_usart->recv_buff,
              test_usart->recv_buff_size, USART_TRANSFER_DMA);
}

void USART_Test_Init(void)
{
    USART_Init_Config_s config = {
        .usart_handle    = &huart1,
        .recv_buff_size  = 64,       /* max expected packet size */
        .module_callback = usart_test_callback,
    };
    test_usart = USARTRegister(&config);
}

void USART_Test_Task(void)
{
    static uint32_t last_tx = 0;
    static uint32_t counter = 0;

    /* Heartbeat: send counter every 1 second */
    if (HAL_GetTick() - last_tx >= 1000)
    {
        last_tx = HAL_GetTick();

        char msg[64];
        int len = snprintf(msg, sizeof(msg),
                           "milFOC USART OK  cnt=%lu\r\n", counter++);
        USARTSend(test_usart, (uint8_t *)msg, (uint16_t)len,
                  USART_TRANSFER_DMA);

        HAL_GPIO_TogglePin(LED_GPIO_Port, LED_Pin);  /* flash on TX */
    }
}
