/**
 ******************************************************************************
 * @file    bsp_can.c
 * @author  Zhang jia ming (FalconFoc) / milFOC Team
 * @brief   FDCAN BSP implementation — multi-instance registration + direct API.
 ******************************************************************************
 */

#include "bsp_can.h"
#include "bsp_dwt.h"
#include "bsp_log.h"
#include "string.h"
#include "stdlib.h"
#include "stdbool.h"

/* ======================== Multi-Instance Pool ============================== */

static FDCANInstance *fdcan_instance[FDCAN_MX_REGISTER_CNT] = {NULL};
static uint8_t idx;

static bool FDCANAddFilter(FDCANInstance *_instance)
{
    FDCAN_FilterTypeDef fdcan_filter_conf;

    fdcan_filter_conf.IdType       = FDCAN_STANDARD_ID;
    fdcan_filter_conf.FilterIndex  = 0;
    fdcan_filter_conf.FilterType   = FDCAN_FILTER_RANGE;
    fdcan_filter_conf.FilterConfig = FDCAN_FILTER_TO_RXFIFO0;
    fdcan_filter_conf.FilterID1    = 0x0000;
    fdcan_filter_conf.FilterID2    = 0x0000;

    return HAL_FDCAN_ConfigFilter(_instance->fdcan_handle, &fdcan_filter_conf) == HAL_OK;
}

static bool FDCANServiceInit(void)
{
    HAL_StatusTypeDef result;
    result  = HAL_FDCAN_Start(&hfdcan1);
    result |= HAL_FDCAN_ActivateNotification(&hfdcan1, FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 0);
    result |= HAL_FDCAN_ActivateNotification(&hfdcan1, FDCAN_IT_RX_FIFO1_NEW_MESSAGE, 0);
    return result == HAL_OK;
}

FDCANInstance *FDCANRegister(FDCAN_Init_Config_s *config)
{
    if (!idx)
        FDCANServiceInit();

    if (idx >= FDCAN_MX_REGISTER_CNT)
    {
        while (1)
            LOGERROR("[bsp_can] CAN instance exceeded MAX num");
    }
    for (size_t i = 0; i < idx; i++)
    {
        if (fdcan_instance[i]->rx_id == config->rx_id &&
            fdcan_instance[i]->fdcan_handle == config->fdcan_handle)
        {
            while (1)
                LOGERROR("[bsp_can] CAN id crash, already registered");
        }
    }

    FDCANInstance *instance = (FDCANInstance *)malloc(sizeof(FDCANInstance));
    memset(instance, 0, sizeof(FDCANInstance));

    instance->txconf.Identifier          = config->tx_id;
    instance->txconf.IdType              = FDCAN_STANDARD_ID;
    instance->txconf.TxFrameType         = FDCAN_DATA_FRAME;
    instance->txconf.DataLength          = 0x08;
    instance->txconf.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
    instance->txconf.BitRateSwitch       = FDCAN_BRS_OFF;
    instance->txconf.FDFormat            = FDCAN_CLASSIC_CAN;
    instance->txconf.TxEventFifoControl  = FDCAN_NO_TX_EVENTS;
    instance->txconf.MessageMarker       = 0x0;

    instance->fdcan_handle          = config->fdcan_handle;
    instance->tx_id                 = config->tx_id;
    instance->rx_id                 = config->rx_id;
    instance->fdcan_module_callback = config->fdcan_module_callback;
    instance->id                    = config->id;

    FDCANAddFilter(instance);
    fdcan_instance[idx++] = instance;

    return instance;
}

uint8_t FDCANTransmit(FDCANInstance *_instance, float timeout)
{
    static uint32_t busy_count;
    float dwt_start = DWT_GetTimeline_ms();
    while (HAL_FDCAN_GetTxFifoFreeLevel(_instance->fdcan_handle) == 0)
    {
        if (DWT_GetTimeline_ms() - dwt_start > timeout)
        {
            LOGWARNING("[bsp_can] CAN MAILbox full, busy_cnt=%d", busy_count);
            busy_count++;
            return 0;
        }
    }
    if (HAL_FDCAN_AddMessageToTxFifoQ(_instance->fdcan_handle,
                                       &_instance->txconf,
                                       _instance->tx_buff) != HAL_OK)
    {
        LOGWARNING("[bsp_can] CAN TX failed, busy_cnt=%d", busy_count);
        busy_count++;
        return 0;
    }
    return 1;
}

void FDCANSetDLC(FDCANInstance *_instance, uint8_t length)
{
    if (length > 8 || length == 0)
    {
        while (1)
            LOGERROR("[bsp_can] CAN DLC error, length=%d", length);
    }
    _instance->txconf.DataLength = length;
}

/* --- HAL RX Callback --- */
void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo0ITs)
{
    (void)RxFifo0ITs;
    for (uint8_t i = 0; i < idx; i++)
    {
        if (fdcan_instance[i]->fdcan_handle == hfdcan)
        {
            FDCAN_RxHeaderTypeDef rxHeader;
            HAL_FDCAN_GetRxMessage(hfdcan, FDCAN_RX_FIFO0, &rxHeader,
                                   fdcan_instance[i]->rx_buff);
            fdcan_instance[i]->rx_len = rxHeader.DataLength >> 16;
            fdcan_instance[i]->rx_id  = rxHeader.Identifier;
            if (fdcan_instance[i]->fdcan_module_callback)
                fdcan_instance[i]->fdcan_module_callback(fdcan_instance[i]);
        }
    }
}

/* ======================== Simple Direct API (milFOC) ======================= */

uint8_t  rx_data1[8] = {0};
uint16_t rec_id1;

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

void bsp_can_init(void)
{
    can_filter_init();
    if (HAL_FDCAN_Start(&hfdcan1) != HAL_OK)
        Error_Handler();
    if (HAL_FDCAN_ActivateNotification(&hfdcan1, FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 0) != HAL_OK)
        Error_Handler();
}

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

uint8_t fdcanx_receive(FDCAN_HandleTypeDef *hfdcan, uint16_t *rec_id, uint8_t *buf)
{
    FDCAN_RxHeaderTypeDef pRxHeader;
    uint8_t len;

    if (HAL_FDCAN_GetRxMessage(hfdcan, FDCAN_RX_FIFO0, &pRxHeader, buf) == HAL_OK)
    {
        *rec_id = pRxHeader.Identifier;
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

void fdcan1_rx_callback(void)
{
    fdcanx_receive(&hfdcan1, &rec_id1, rx_data1);
}
