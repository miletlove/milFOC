/**
 ******************************************************************************
 * @file    bsp_can.h
 * @author  Zhang jia ming (FalconFoc) / milFOC Team
 * @brief   FDCAN Board Support Package — multi-instance registration with
 *          module callback, timeout-based TX, and DLC management.
 *
 *          Also provides a lightweight direct-HAL API (bsp_can_init /
 *          fdcanx_send_data / fdcanx_receive) for simple single-CAN use.
 ******************************************************************************
 */

#ifndef BSP_CAN_H
#define BSP_CAN_H

#include "general_def.h"
#include "main.h"
#include "fdcan.h"
#include "stdint.h"

#define FDCAN_MX_REGISTER_CNT 6
#define DEVICE_CAN_CNT        1

/* ======================== Multi-Instance API (FalconFoc) =================== */

typedef struct fdcaninstance
{
    FDCAN_HandleTypeDef *fdcan_handle;
    FDCAN_TxHeaderTypeDef txconf;
    uint32_t tx_id;
    uint32_t tx_mailbox;
    uint8_t  tx_buff[8];
    uint8_t  rx_buff[8];
    uint32_t rx_id;
    uint8_t  rx_len;
    void   (*fdcan_module_callback)(struct fdcaninstance *);
    void    *id;
} FDCANInstance;

typedef struct
{
    FDCAN_HandleTypeDef *fdcan_handle;
    uint32_t tx_id;
    uint32_t rx_id;
    void   (*fdcan_module_callback)(FDCANInstance *);
    void    *id;
} FDCAN_Init_Config_s;

FDCANInstance *FDCANRegister(FDCAN_Init_Config_s *config);
void    FDCANSetDLC(FDCANInstance *_instance, uint8_t length);
uint8_t FDCANTransmit(FDCANInstance *_instance, float timeout);

/* ======================== Simple Direct API (milFOC) ======================= */

void    bsp_can_init(void);
void    can_filter_init(void);
uint8_t fdcanx_send_data(FDCAN_HandleTypeDef *hfdcan, uint16_t id, uint8_t *data, uint32_t len);
uint8_t fdcanx_receive(FDCAN_HandleTypeDef *hfdcan, uint16_t *rec_id, uint8_t *buf);
void    fdcan1_rx_callback(void);

extern uint8_t  rx_data1[8];
extern uint16_t rec_id1;

#endif /* BSP_CAN_H */
