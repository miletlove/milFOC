/**
 ******************************************************************************
 * @file    bsp_can.c
 * @author  milFOC Team
 * @brief   FDCAN BSP implementation for STM32G431.
 *          Provides low-level CAN send/receive/init using HAL FDCAN driver.
 *
 * @note    Tested with: FDCAN1, standard 11-bit ID, classic CAN, 8-byte payload.
 *          RX callback copies data to global rx_data1[8] for upper layer.
 ******************************************************************************
 */

#include "bsp_can.h"
#include "string.h"

/* Global RX buffer (read by can_driver / upper layer) */
uint8_t rx_data1[8] = {0};
uint16_t rec_id1;

/**
 * @brief  Initialize FDCAN1: filter, start, enable RX FIFO0 interrupt.
 */
void bsp_can_init(void)
{
    can_filter_init();
    if (HAL_FDCAN_Start(&hfdcan1) != HAL_OK)
    {
        Error_Handler();
    }
    if (HAL_FDCAN_ActivateNotification(&hfdcan1, FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 0) != HAL_OK)
    {
        Error_Handler();
    }
}

/**
 * @brief  Configure FDCAN filter: mask mode, pass all standard IDs to FIFO0.
 */
void can_filter_init(void)
{
    FDCAN_FilterTypeDef fdcan_filter;

    fdcan_filter.IdType       = FDCAN_STANDARD_ID;
    fdcan_filter.FilterIndex  = 0;
    fdcan_filter.FilterType   = FDCAN_FILTER_MASK;
    fdcan_filter.FilterConfig = FDCAN_FILTER_TO_RXFIFO0;
    fdcan_filter.FilterID1    = 0x00;
    fdcan_filter.FilterID2    = 0x00;

    HAL_FDCAN_ConfigFilter(&hfdcan1, &fdcan_filter);
}

/**
 * @brief  Send a CAN data frame via FIFO queue.
 * @param  hfdcan : FDCAN handle
 * @param  id     : 11-bit standard ID
 * @param  data   : payload pointer
 * @param  len    : payload length (<= 8 for classic CAN)
 * @return 0 = success, 1 = failure
 */
uint8_t fdcanx_send_data(FDCAN_HandleTypeDef *hfdcan, uint16_t id, uint8_t *data, uint32_t len)
{
    FDCAN_TxHeaderTypeDef pTxHeader;

    pTxHeader.Identifier           = id;
    pTxHeader.IdType               = FDCAN_STANDARD_ID;
    pTxHeader.TxFrameType          = FDCAN_DATA_FRAME;
    pTxHeader.DataLength           = (len <= 8) ? len : 8;
    pTxHeader.ErrorStateIndicator  = FDCAN_ESI_ACTIVE;
    pTxHeader.BitRateSwitch        = FDCAN_BRS_OFF;
    pTxHeader.FDFormat             = FDCAN_CLASSIC_CAN;
    pTxHeader.TxEventFifoControl   = FDCAN_NO_TX_EVENTS;
    pTxHeader.MessageMarker        = 0;

    if (HAL_FDCAN_AddMessageToTxFifoQ(hfdcan, &pTxHeader, data) != HAL_OK)
        return 1;
    return 0;
}

/**
 * @brief  Receive a CAN data frame from RX FIFO0.
 * @param  hfdcan : FDCAN handle
 * @param  rec_id : out - received CAN ID
 * @param  buf    : out - received payload
 * @return received data length (bytes), 0 if no message
 */
uint8_t fdcanx_receive(FDCAN_HandleTypeDef *hfdcan, uint16_t *rec_id, uint8_t *buf)
{
    FDCAN_RxHeaderTypeDef pRxHeader;
    uint8_t len;

    if (HAL_FDCAN_GetRxMessage(hfdcan, FDCAN_RX_FIFO0, &pRxHeader, buf) == HAL_OK)
    {
        *rec_id = pRxHeader.Identifier;

        /* Map DLC to byte length */
        if      (pRxHeader.DataLength <= FDCAN_DLC_BYTES_8)   len = 8;
        else if (pRxHeader.DataLength <= FDCAN_DLC_BYTES_12)  len = 12;
        else if (pRxHeader.DataLength <= FDCAN_DLC_BYTES_16)  len = 16;
        else if (pRxHeader.DataLength <= FDCAN_DLC_BYTES_20)  len = 20;
        else if (pRxHeader.DataLength <= FDCAN_DLC_BYTES_24)  len = 24;
        else if (pRxHeader.DataLength <= FDCAN_DLC_BYTES_32)  len = 32;
        else if (pRxHeader.DataLength <= FDCAN_DLC_BYTES_48)  len = 48;
        else                                                   len = 64;

        return len;
    }
    return 0;
}

/**
 * @brief  FDCAN1 RX callback - called from HAL_FDCAN_RxFifo0Callback ISR.
 */
void fdcan1_rx_callback(void)
{
    fdcanx_receive(&hfdcan1, &rec_id1, rx_data1);
}

/**
 * @brief  HAL FDCAN RX FIFO0 interrupt callback (overrides weak function).
 */
void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo0ITs)
{
    (void)RxFifo0ITs;
    if (hfdcan == &hfdcan1)
    {
        fdcan1_rx_callback();
    }
}
