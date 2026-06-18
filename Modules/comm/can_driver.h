/**
 ******************************************************************************
 * @file    can_driver.h
 * @author  Zhang jia ming (FalconFoc) / milFOC Team
 * @brief   CAN communication protocol driver with frame packing (header/tail/CRC8)
 *          and daemon integration for online detection.
 ******************************************************************************
 */

#ifndef CAN_DRIVER_H
#define CAN_DRIVER_H

#include "daemon.h"
#include "bsp_can.h"

#define MX_FDCAN_COMM_COUNT      4
#define FDCAN_COMM_MAX_BUFFSIZE  200
#define FDCAN_COMM_HEADER        's'
#define FDCAN_COMM_TAIL          'e'
#define FDCAN_COMM_OFFSET_BYTES  4

#pragma pack(1)
typedef struct
{
    FDCANInstance *can_ins;

    uint8_t send_data_len;
    uint8_t send_buf_len;
    uint8_t raw_sendbuf[FDCAN_COMM_MAX_BUFFSIZE + FDCAN_COMM_OFFSET_BYTES];

    uint8_t recv_data_len;
    uint8_t recv_buf_len;
    uint8_t raw_recvbuf[FDCAN_COMM_MAX_BUFFSIZE + FDCAN_COMM_OFFSET_BYTES];
    uint8_t unpacked_recv_data[FDCAN_COMM_MAX_BUFFSIZE];

    uint8_t recv_state;
    uint8_t cur_recv_len;
    uint8_t update_flag;

    DaemonInstance *comm_daemon;
} FDCANCommInstance;
#pragma pack()

typedef struct
{
    FDCAN_Init_Config_s can_config;
    uint8_t  send_data_len;
    uint8_t  recv_data_len;
    uint16_t daemon_count;
} FDCANComm_Init_Config_s;

FDCANCommInstance *FDCANCommInit(FDCANComm_Init_Config_s *comm_config);
void    FDCANCommSend(FDCANCommInstance *instance, uint8_t *data);
void   *FDCANCommGet(FDCANCommInstance *instance);
uint8_t FDCANCommIsOnline(FDCANCommInstance *instance);

#endif /* CAN_DRIVER_H */
