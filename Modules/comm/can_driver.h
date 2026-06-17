/**
 ******************************************************************************
 * @file    can_driver.h
 * @author  milFOC Team
 * @brief   CAN communication protocol driver.
 *          Handles command frame parsing and telemetry frame packing
 *          on top of the BSP CAN layer (bsp_can.h).
 *
 * @note    Protocol frame format (up to 8 bytes):
 *          CMD frame:  [cmd_id:4B][data:4B]
 *          TELEM frame: [state:4B][velocity:2B][position:2B]
 ******************************************************************************
 */

#ifndef CAN_DRIVER_H
#define CAN_DRIVER_H

#include "general_def.h"
#include "bsp_can.h"
#include "stdint.h"

/**
 * @brief CAN communication instance (wraps BSP CAN with protocol layer)
 */
typedef struct
{
    FDCAN_HandleTypeDef *fdcan_handle;  /* CAN peripheral handle */
    uint16_t tx_id;                     /* TX message ID */
    uint16_t rx_id;                     /* RX filter ID */
    uint8_t send_data_len;              /* TX payload length */
    uint8_t recv_data_len;              /* RX payload length */
} FDCANCommInstance;

/**
 * @brief CAN communication initialization config
 */
typedef struct
{
    FDCAN_HandleTypeDef *fdcan_handle;  /* CAN peripheral handle */
    uint16_t tx_id;                     /* TX message ID */
    uint16_t rx_id;                     /* RX filter ID */
    uint8_t send_data_len;              /* TX data length */
    uint8_t recv_data_len;              /* RX data length */
} FDCANComm_Init_Config_s;

/* --- Public API --- */

FDCANCommInstance *FDCANCommInit(FDCANComm_Init_Config_s *config);
void *FDCANCommGet(FDCANCommInstance *ins);
int FDCANCommSend(FDCANCommInstance *ins, void *data);

#endif /* CAN_DRIVER_H */
