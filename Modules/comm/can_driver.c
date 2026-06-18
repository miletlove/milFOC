/**
 ******************************************************************************
 * @file    can_driver.c
 * @author  Zhang jia ming (FalconFoc) / milFOC Team
 * @brief   CAN communication protocol driver — frame packing with CRC8,
 *          daemon online detection, and multi-frame transmit (8-byte CAN limit).
 ******************************************************************************
 */

#include "can_driver.h"
#include "string.h"
#include "stdlib.h"
#include "crc8.h"

static void FDCANCommResetRx(FDCANCommInstance *ins)
{
    memset(ins->raw_recvbuf, 0, ins->cur_recv_len);
    ins->recv_state   = 0;
    ins->cur_recv_len = 0;
}

static void FDCANCommRxCallback(FDCANInstance *_instance)
{
    FDCANCommInstance *comm = (FDCANCommInstance *)_instance->id;

    if (_instance->rx_buff[0] == FDCAN_COMM_HEADER && comm->recv_state == 0)
    {
        if (_instance->rx_buff[1] == comm->recv_data_len)
            comm->recv_state = 1;
        else
            return;
    }

    if (comm->recv_state)
    {
        if (comm->cur_recv_len + _instance->rx_len > comm->recv_buf_len)
        {
            FDCANCommResetRx(comm);
            return;
        }

        memcpy(comm->raw_recvbuf + comm->cur_recv_len, _instance->rx_buff, _instance->rx_len);
        comm->cur_recv_len += _instance->rx_len;

        if (comm->cur_recv_len == comm->recv_buf_len)
        {
            if (comm->raw_recvbuf[comm->recv_buf_len - 1] == FDCAN_COMM_TAIL)
            {
                if (comm->raw_recvbuf[comm->recv_buf_len - 2] ==
                    crc_8(comm->raw_recvbuf + 2, comm->recv_data_len))
                {
                    memcpy(comm->unpacked_recv_data, comm->raw_recvbuf + 2, comm->recv_data_len);
                    comm->update_flag = 1;
                    DaemonReload(comm->comm_daemon);
                }
            }
            FDCANCommResetRx(comm);
            return;
        }
    }
}

static void FDCANCommLostCallback(void *cancomm)
{
    FDCANCommInstance *comm = (FDCANCommInstance *)cancomm;
    FDCANCommResetRx(comm);
}

FDCANCommInstance *FDCANCommInit(FDCANComm_Init_Config_s *comm_config)
{
    FDCANCommInstance *ins = (FDCANCommInstance *)malloc(sizeof(FDCANCommInstance));
    memset(ins, 0, sizeof(FDCANCommInstance));

    ins->recv_data_len  = comm_config->recv_data_len;
    ins->recv_buf_len   = comm_config->recv_data_len + FDCAN_COMM_OFFSET_BYTES;
    ins->send_data_len  = comm_config->send_data_len;
    ins->send_buf_len   = comm_config->send_data_len + FDCAN_COMM_OFFSET_BYTES;
    ins->raw_sendbuf[0] = FDCAN_COMM_HEADER;
    ins->raw_sendbuf[1] = comm_config->send_data_len;
    ins->raw_sendbuf[comm_config->send_data_len + FDCAN_COMM_OFFSET_BYTES - 1] = FDCAN_COMM_TAIL;

    comm_config->can_config.id                         = ins;
    comm_config->can_config.fdcan_module_callback      = FDCANCommRxCallback;
    ins->can_ins = FDCANRegister(&comm_config->can_config);

    Daemon_Init_Config_s daemon_config = {
        .callback     = FDCANCommLostCallback,
        .owner_id     = (void *)ins,
        .reload_count = comm_config->daemon_count,
    };
    ins->comm_daemon = DaemonRegister(&daemon_config);
    return ins;
}

void FDCANCommSend(FDCANCommInstance *instance, uint8_t *data)
{
    uint8_t crc8_val, send_len;

    memcpy(instance->raw_sendbuf + 2, data, instance->send_data_len);
    crc8_val = crc_8(data, instance->send_data_len);
    instance->raw_sendbuf[2 + instance->send_data_len] = crc8_val;

    for (size_t i = 0; i < instance->send_buf_len; i += 8)
    {
        send_len = instance->send_buf_len - i >= 8 ? 8 : instance->send_buf_len - i;
        FDCANSetDLC(instance->can_ins, send_len);
        memcpy(instance->can_ins->tx_buff, instance->raw_sendbuf + i, send_len);
        FDCANTransmit(instance->can_ins, 2);
    }
}

void *FDCANCommGet(FDCANCommInstance *instance)
{
    instance->update_flag = 0;
    return instance->unpacked_recv_data;
}

uint8_t FDCANCommIsOnline(FDCANCommInstance *instance)
{
    return DaemonIsOnline(instance->comm_daemon);
}
