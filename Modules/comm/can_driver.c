/**
 ******************************************************************************
 * @file    can_driver.c
 * @author  milFOC Team
 * @brief   CAN communication protocol driver implementation.
 ******************************************************************************
 */

#include "can_driver.h"
#include "string.h"

FDCANCommInstance *FDCANCommInit(FDCANComm_Init_Config_s *config)
{
    FDCANCommInstance *ins = (FDCANCommInstance *)malloc(sizeof(FDCANCommInstance));
    memset(ins, 0, sizeof(FDCANCommInstance));

    ins->can_ins       = FDCANRegister(&config->can_config);
    ins->send_data_len = config->send_data_len;
    ins->recv_data_len = config->recv_data_len;

    return ins;
}

void *FDCANCommGet(FDCANCommInstance *ins)
{
    if (ins == NULL || ins->can_ins == NULL) return NULL;
    return (void *)ins->can_ins->rx_buff;
}

int FDCANCommSend(FDCANCommInstance *ins, void *data)
{
    if (ins == NULL || ins->can_ins == NULL || data == NULL) return -1;

    memcpy(ins->can_ins->tx_buff, data, ins->send_data_len);
    FDCANSetDLC(ins->can_ins, ins->send_data_len);
    return FDCANTransmit(ins->can_ins, 10.0f);
}
