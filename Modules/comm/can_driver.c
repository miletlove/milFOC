/**
 ******************************************************************************
 * @file    can_driver.c
 * @author  milFOC Team
 * @brief   CAN communication protocol driver implementation.
 *          Built on top of bsp_can (tested BSP layer).
 ******************************************************************************
 */

#include "can_driver.h"
#include "string.h"

FDCANCommInstance *FDCANCommInit(FDCANComm_Init_Config_s *config)
{
    FDCANCommInstance *ins = (FDCANCommInstance *)malloc(sizeof(FDCANCommInstance));
    memset(ins, 0, sizeof(FDCANCommInstance));

    ins->fdcan_handle  = config->fdcan_handle;
    ins->tx_id         = config->tx_id;
    ins->rx_id         = config->rx_id;
    ins->send_data_len = config->send_data_len;
    ins->recv_data_len = config->recv_data_len;

    /* Initialize CAN hardware (filter + start + RX interrupt) */
    bsp_can_init();

    return ins;
}

void *FDCANCommGet(FDCANCommInstance *ins)
{
    (void)ins;
    return (void *)rx_data1;
}

int FDCANCommSend(FDCANCommInstance *ins, void *data)
{
    if (ins == NULL || data == NULL) return -1;
    return fdcanx_send_data(ins->fdcan_handle, ins->tx_id, (uint8_t *)data, ins->send_data_len);
}
