/**
 ******************************************************************************
 * @file    bsp_can.c
 * @author  milFOC Team
 * @brief   CAN/FDCAN BSP implementation for STM32G431.
 *          Provides multi-instance CAN bus management.
 *
 * @note    Call FDCANRegister() to obtain a CAN instance before using.
 *          This file is adapted from FalconFoc's BSP layer for milFOC hardware.
 ******************************************************************************
 */

#include "bsp_can.h"
#include "bsp_dwt.h"
#include "bsp_log.h"
#include "string.h"

/* Registered CAN instance pool */
static FDCANInstance *fdcan_instance[FDCAN_MX_REGISTER_CNT] = {NULL};
static uint8_t idx;

/**
 * @brief  Register a FDCAN instance for a module
 */
FDCANInstance *FDCANRegister(FDCAN_Init_Config_s *config)
{
    if (idx >= FDCAN_MX_REGISTER_CNT)
    {
        LOGERROR("[CAN] Max CAN instances reached (%d)", FDCAN_MX_REGISTER_CNT);
        return NULL;
    }

    FDCANInstance *ins = (FDCANInstance *)malloc(sizeof(FDCANInstance));
    memset(ins, 0, sizeof(FDCANInstance));

    ins->fdcan_handle          = config->fdcan_handle;
    ins->tx_id                 = config->tx_id;
    ins->rx_id                 = config->rx_id;
    ins->fdcan_module_callback = config->fdcan_module_callback;
    ins->id                    = config->id;

    /* Configure TX header defaults */
    ins->txconf.Identifier          = config->tx_id;
    ins->txconf.IdType              = FDCAN_STANDARD_ID;
    ins->txconf.TxFrameType         = FDCAN_DATA_FRAME;
    ins->txconf.DataLength          = FDCAN_DLC_BYTES_8;
    ins->txconf.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
    ins->txconf.BitRateSwitch       = FDCAN_BRS_OFF;
    ins->txconf.FDFormat            = FDCAN_CLASSIC_CAN;
    ins->txconf.TxEventFifoControl  = FDCAN_NO_TX_EVENTS;
    ins->txconf.MessageMarker       = 0;

    /* Start CAN peripheral if not already started */
    HAL_FDCAN_Start(ins->fdcan_handle);

    fdcan_instance[idx++] = ins;
    LOGINFO("[CAN] Instance registered, tx_id=0x%03X, rx_id=0x%03X", config->tx_id, config->rx_id);
    return ins;
}

/**
 * @brief  Set TX data length code
 */
void FDCANSetDLC(FDCANInstance *_instance, uint8_t length)
{
    if (length > 8) length = 8;
    switch (length)
    {
        case 0:  _instance->txconf.DataLength = FDCAN_DLC_BYTES_0;  break;
        case 1:  _instance->txconf.DataLength = FDCAN_DLC_BYTES_1;  break;
        case 2:  _instance->txconf.DataLength = FDCAN_DLC_BYTES_2;  break;
        case 3:  _instance->txconf.DataLength = FDCAN_DLC_BYTES_3;  break;
        case 4:  _instance->txconf.DataLength = FDCAN_DLC_BYTES_4;  break;
        case 5:  _instance->txconf.DataLength = FDCAN_DLC_BYTES_5;  break;
        case 6:  _instance->txconf.DataLength = FDCAN_DLC_BYTES_6;  break;
        case 7:  _instance->txconf.DataLength = FDCAN_DLC_BYTES_7;  break;
        case 8:
        default: _instance->txconf.DataLength = FDCAN_DLC_BYTES_8;  break;
    }
}

/**
 * @brief  Transmit a CAN message with timeout
 */
uint8_t FDCANTransmit(FDCANInstance *_instance, float timeout)
{
    uint32_t start = DWT->CYCCNT;
    uint32_t tick  = HAL_GetTick();

    while (HAL_FDCAN_GetTxFifoFreeLevel(_instance->fdcan_handle) == 0)
    {
        if ((HAL_GetTick() - tick) > (uint32_t)(timeout * 1000))
        {
            LOGWARNING("[CAN] TX timeout for tx_id=0x%03X", _instance->tx_id);
            return 1;
        }
    }

    if (HAL_FDCAN_AddMessageToTxFifoQ(_instance->fdcan_handle,
                                       &_instance->txconf,
                                       _instance->tx_buff) != HAL_OK)
    {
        LOGERROR("[CAN] TX failed for tx_id=0x%03X", _instance->tx_id);
        return 2;
    }

    return 0;
}
